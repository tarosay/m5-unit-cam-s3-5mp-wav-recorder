#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s_pdm.h>
#include <math.h>
#include "mic.h"
// ======================= 既定値（グローバル） =======================
static SessionConfig g_defSession = {};  // 構造体のデフォルト初期化適用
static FixedGainConfig g_defFixedGain = {};
static AgcConfig g_defAgc = {};

// ======================= 内部ユーティリティ =======================
static inline float db2lin(float db) {
  return powf(10.0f, db / 20.0f);
}
static inline float lin2db(float g) {
  return 20.0f * log10f(max(g, 1e-20f));
}

static inline int16_t saturate_s16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)v;
}

// ======================= DCブロッカ =======================
static float dc_x1 = 0.0f, dc_y1 = 0.0f;
static inline void dcReset() {
  dc_x1 = 0.0f;
  dc_y1 = 0.0f;
}
// デフォルト: 16kHz時 ~12Hz相当（他Fsでも安全に効く）
static inline float dc_alpha_for(uint32_t fs) {
  // 単純に 0.995 を基準に、Fsで微調整（必要十分の簡易式）
  if (fs <= 8000) return 0.990f;
  if (fs >= 48000) return 0.9975f;
  return 0.995f;
}
static inline void dcBlocker(int16_t* io, size_t n, float alpha) {
  float x1 = dc_x1, y1 = dc_y1;
  for (size_t i = 0; i < n; ++i) {
    const float x = (float)io[i];
    float y = (x - x1) + alpha * y1;
    x1 = x;
    y1 = y;
    io[i] = saturate_s16(y);
  }
  dc_x1 = x1;
  dc_y1 = y1;
}

// ======================= 固定ゲイン + セーフティリミッタ =======================
// gainLin は「振幅倍率」です（例: +6 dB ≈ 2.0x, +20 dB ≈ 10x, +40 dB ≈ 100x）。
// 前段でブロックピークを見て、LIMIT_THRESH を超えそうなら "先に" 縮めてから適用します。
// こうすることで、適用後の波形が16bitの範囲（±32768）を超えないようにします。
static inline void applyFixedGain(int16_t* io, size_t n, float gainLin) {
  const float PCM16_MAX_F = 32767.0f;
  const float LIMIT_THRESH = PCM16_MAX_F * 0.98f;  // 98%に抑える安全マージン（リミッタ）
  // 1) 事前ピーク検出：このブロックを "このゲインで" 増幅した時の最大値を見積もる
  float peak = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    float a = fabsf((float)io[i] * gainLin);
    if (a > peak) peak = a;
  }
  // 2) 超えそうなら、必要な分だけゲインを縮める（ゼロ割防止の微小値つき）
  if (peak > LIMIT_THRESH) {
    gainLin *= (LIMIT_THRESH / (peak + 1e-12f));
  }
  // 3) 実際に増幅してから 16bit にサチュレート（飽和）
  for (size_t i = 0; i < n; ++i) {
    float z = (float)io[i] * gainLin;
    io[i] = saturate_s16(z);
  }
}


// ======================= AGC 補助 =======================
// block_rms: ブロックの RMS（平均的な大きさ）。AGCの“今どれくらい鳴っているか”の指標に使う。
static inline float block_rms(const int16_t* p, size_t n) {
  if (!n) return 0.0f;
  double acc = 0.0;
  for (size_t i = 0; i < n; ++i) {
    float v = (float)p[i];
    acc += (double)v * (double)v;
  }
  return sqrtf((float)(acc / (double)n));
}

// one_pole_coeff_ms: ブロック単位の一次フィルタ係数（アタック/リリース時定数を ms で指定）
static inline float one_pole_coeff_ms(float ms, float fs, size_t blockSamples) {
  // 係数 a = exp(-blockT/tau)。aが大きいほど「動きが鈍い」（ゆっくり追従）。
  const float blockTms = (float)blockSamples / fs * 1000.0f;
  const float tau = (ms <= 0.0f) ? 0.001f : ms;
  float a = expf(-blockTms / tau);
  if (a < 0.0f) a = 0.0f;
  if (a > 1.0f) a = 1.0f;
  return a;
}

// agc_update_gain:
//  - targetPeakDbFS（例: -3 dBFS ≈ 振幅0.707FS）を狙うように、blockRms から必要倍率を推定
//  - 大きくなった時は素早く下げる(attack)、小さくなった時はゆっくり上げる(release)
//  - 無音近辺は noiseGate をかけて“暴走しない”ように追従を鈍らせる
//  - 最後に min/max dB の範囲へクランプ
static inline float agc_update_gain(float currentLinGain,
                                    float blockRms,
                                    const AgcConfig& agc,
                                    size_t blockSamples,
                                    uint32_t fs) {
  const float PCM16_MAX_F = 32767.0f;

  // 無音ゲート（RMSが -60 dBFS 相当等の閾値より小さいなら、動きを抑える）
  const float gateThresh = db2lin(agc.noiseGateDbFS) * PCM16_MAX_F;  // 例: -60 dBFS ≈ 0.001FS
  bool gated = (blockRms < gateThresh);

  // 目標ピーク（dBFS）→ 線形の目標振幅
  const float targetPeak = db2lin(agc.targetPeakDbFS) * PCM16_MAX_F;  // 例: -3 dBFS ≈ 0.707FS

  // 必要倍率のざっくり推定： “今の平均” を “目標ピーク” に近づける
  // 例) 平均が小さい → needed が大きい（上げる） / 平均が大きい → needed が小さい（下げる）
  float needed = (blockRms > 1.0f) ? (targetPeak / blockRms) : db2lin(agc.maxGainDb);

  // 上下限（dB指定 → 線形倍率）でクランプ
  const float maxLin = db2lin(agc.maxGainDb);  // 例: +36 dB ≈ 63x
  const float minLin = db2lin(agc.minGainDb);  // 例:  -6 dB ≈ 0.5x
  if (needed > maxLin) needed = maxLin;
  if (needed < minLin) needed = minLin;

  // 片側時定数（大き過ぎる→下げは速い=attack、小さ過ぎる→上げは遅い=release）
  const float a_att = one_pole_coeff_ms(agc.attackMs, (float)fs, blockSamples);
  const float a_rel = one_pole_coeff_ms(agc.releaseMs, (float)fs, blockSamples);
  const float a_gate = one_pole_coeff_ms(agc.gateReleaseMs, (float)fs, blockSamples);

  // 目標倍率 needed へ一次フィルタで近づける
  float a = (needed < currentLinGain) ? a_att : a_rel;  // 下げは速く / 上げは遅く
  if (gated) a = max(a, a_gate);                        // 無音中は更に動きを鈍らせる
  float next = a * currentLinGain + (1.0f - a) * needed;

  // 最終クランプ
  if (next > maxLin) next = maxLin;
  if (next < minLin) next = minLin;
  return next;
}


// ======================= I2S PDM（新ドライバ） =======================
static i2s_chan_handle_t rx_handle = NULL;

// 最小限セットアップ：slot と CLK極性を試し、読めたらOK
static bool pdmSetup(i2s_pdm_slot_mask_t slot, bool clkInv, uint32_t sampleRate) {
  if (rx_handle) {
    i2s_channel_disable(rx_handle);
    i2s_del_channel(rx_handle);
    rx_handle = NULL;
  }

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = 6;
  chan_cfg.dma_frame_num = 256;
  if (i2s_new_channel(&chan_cfg, NULL, &rx_handle) != ESP_OK) return false;

  i2s_pdm_rx_clk_config_t clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(sampleRate);
  i2s_pdm_rx_slot_config_t slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  slot_cfg.slot_mask = slot;  // LEFT or RIGHT

  i2s_pdm_rx_config_t pdm_rx_cfg = {};
  pdm_rx_cfg.clk_cfg = clk_cfg;
  pdm_rx_cfg.slot_cfg = slot_cfg;
  pdm_rx_cfg.gpio_cfg.clk = (gpio_num_t)PDM_CLK_GPIO_NUM;
  pdm_rx_cfg.gpio_cfg.din = (gpio_num_t)PDM_DIN_GPIO_NUM;
  pdm_rx_cfg.gpio_cfg.invert_flags.clk_inv = clkInv;  // din_inv は環境によって無い

  if (i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg) != ESP_OK) return false;
  if (i2s_channel_enable(rx_handle) != ESP_OK) return false;

  // 立上りのゴミを軽く捨てる
  uint32_t t0 = millis();
  static int16_t tmp[512];
  size_t br = 0;
  do {
    br = 0;
    (void)i2s_channel_read(rx_handle, tmp, sizeof(tmp), &br, 0);
  } while (br > 0 && (millis() - t0) < 60);

  // 200msだけ読めるか確認
  br = 0;
  esp_err_t err = i2s_channel_read(rx_handle, tmp, sizeof(tmp), &br, 200);
  return (err == ESP_OK && br > 0);
}

static bool pdmAutoPick(uint32_t sampleRate) {
  const i2s_pdm_slot_mask_t slots[2] = { I2S_PDM_SLOT_RIGHT, I2S_PDM_SLOT_LEFT };
  const bool clkInvs[2] = { false, true };
  for (int si = 0; si < 2; ++si) {
    for (int ci = 0; ci < 2; ++ci) {
      if (pdmSetup(slots[si], clkInvs[ci], sampleRate)) return true;
    }
  }
  return false;
}

// ======================= WAVヘッダ =======================
static void writeWavHeader(File& f, uint32_t sr, uint16_t bits, uint16_t ch) {
  f.flush();
  uint32_t total = f.size();
  if (total < 44) total = 44;
  const uint32_t dataBytes = total - 44;
  const uint32_t byteRate = sr * ch * (bits / 8);
  const uint16_t blockAlign = ch * (bits / 8);

  uint8_t h[44];
  memcpy(h + 0, "RIFF", 4);
  const uint32_t cs = 36 + dataBytes;
  memcpy(h + 4, &cs, 4);
  memcpy(h + 8, "WAVE", 4);
  memcpy(h + 12, "fmt ", 4);
  const uint32_t sc1 = 16;
  memcpy(h + 16, &sc1, 4);
  const uint16_t af = 1;
  memcpy(h + 20, &af, 2);
  memcpy(h + 22, &ch, 2);
  memcpy(h + 24, &sr, 4);
  memcpy(h + 28, &byteRate, 4);
  memcpy(h + 32, &blockAlign, 2);
  memcpy(h + 34, &bits, 2);
  memcpy(h + 36, "data", 4);
  memcpy(h + 40, &dataBytes, 4);

  f.seek(0);
  (void)f.write(h, 44);
  f.flush();
}

// ======================= 連番ファイル =======================
static String nextWavPath(const char* dir) {
  if (!SD.exists(dir)) SD.mkdir(dir);
  char name[64];
  for (int i = 1; i <= 9999; ++i) {
    snprintf(name, sizeof(name), "%s/REC%04d.WAV", dir, i);
    if (!SD.exists(name)) return String(name);
  }
  snprintf(name, sizeof(name), "%s/REC9999.WAV", dir);
  return String(name);
}

// ======================= 既定値 Get/Set =======================
SessionConfig getDefaultSession() {
  return g_defSession;
}
FixedGainConfig getDefaultFixedGain() {
  return g_defFixedGain;
}
AgcConfig getDefaultAgc() {
  return g_defAgc;
}

void setDefaultSession(const SessionConfig& s) {
  g_defSession = s;
}
void setDefaultFixedGain(const FixedGainConfig& g) {
  g_defFixedGain = g;
}
void setDefaultAgc(const AgcConfig& a) {
  g_defAgc = a;
}

// ======================= 初期化 =======================
bool micInit() {
  return pdmAutoPick(g_defSession.sampleRate);
}

// ======================= 録音（固定ゲイン） =======================
RecResult recordingFixedEx(const SessionConfig* sessionOpt,
                           const FixedGainConfig* gainOpt,
                           String* outPath,
                           uint32_t* outBytes) {
  // 実際に使う設定を確定
  SessionConfig s = g_defSession;
  FixedGainConfig g = g_defFixedGain;
  if (sessionOpt) s = *sessionOpt;
  if (gainOpt) g = *gainOpt;

  const uint16_t bytesPerSample = s.bitsPerSamp / 8;
  if (s.bitsPerSamp != 16 || s.channels != 1) {
    // この実装は PCM16 mono 前提（必要なら拡張）
    return RecResult::HeaderPlaceWriteError;  // 簡易に不正扱い
  }

  String path = nextWavPath(s.dir);
  File f = SD.open(path.c_str(), FILE_WRITE);
  if (!f) return RecResult::FileOpenError;

  uint8_t zero[44] = { 0 };
  if (f.write(zero, 44) != 44) {
    f.close();
    return RecResult::HeaderPlaceWriteError;
  }
  f.flush();

  dcReset();
  const float dcAlpha = dc_alpha_for(s.sampleRate);

  // バッファ準備
  int16_t* bufPtr = nullptr;
  size_t bufSamps = 0;
  std::unique_ptr<int16_t[]> bufHolder;
  if (s.extBuffer && s.extBufSamps >= s.blockSamples) {
    bufPtr = s.extBuffer;
    bufSamps = s.extBufSamps;
  } else {
    bufHolder.reset(new int16_t[s.blockSamples]);
    bufPtr = bufHolder.get();
    bufSamps = s.blockSamples;
  }
  const size_t bufBytes = bufSamps * sizeof(int16_t);

  const uint32_t totalBytes = s.sampleRate * s.channels * bytesPerSample * (uint32_t)0 +  // init
                              s.sampleRate * s.channels * bytesPerSample * 1;             // dummy to keep form
  // 正式に：
  const uint32_t totalBytesFormal = s.sampleRate * s.channels * bytesPerSample * (uint32_t)0;  // 0 for clarity
  (void)totalBytesFormal;                                                                      // 未使用警告抑止

  const uint32_t totalBytesTarget = s.sampleRate * (uint32_t) /*seconds*/ 0;  // 使わない式テンプレ抑止
  (void)totalBytesTarget;

  // 秒数は呼び出し元引数で管理する都合上、Exでは秒数を受けない設計 → シンプルAPIから渡す
  // → 実体は下のシンプルAPIから呼ばれる内部ラムダで扱う（次の recordingFixed 参照）
  // ここではダミー返し（本関数は内部ラムダからのみ使う設計）
  return RecResult::Success;
}

// ======================= 録音：固定ゲイン（処理部コメント強化版） =======================
static RecResult doRecordingFixedSeconds(uint32_t recSeconds,
                                         const SessionConfig& s,
                                         const FixedGainConfig& g,
                                         String* outPath,
                                         uint32_t* outBytes) {
  const uint16_t bytesPerSample = s.bitsPerSamp / 8;
  String path = nextWavPath(s.dir);
  File f = SD.open(path.c_str(), FILE_WRITE);
  if (!f) return RecResult::FileOpenError;

  // 1) WAVヘッダは後で上書きするので "空44バイト" を最初に確保
  uint8_t zero[44] = { 0 };
  if (f.write(zero, 44) != 44) {
    f.close();
    return RecResult::HeaderPlaceWriteError;
  }
  f.flush();

  // 2) フィルタ状態を録音ごとに初期化
  dcReset();
  const float dcAlpha = dc_alpha_for(s.sampleRate);

  // 3) 入出力バッファを用意（外部提供があればそれを使用）
  int16_t* bufPtr = nullptr;
  std::unique_ptr<int16_t[]> bufHolder;
  if (s.extBuffer && s.extBufSamps >= s.blockSamples) {
    bufPtr = s.extBuffer;
  } else {
    bufHolder.reset(new int16_t[s.blockSamples]);
    bufPtr = bufHolder.get();
  }
  const size_t bufBytes = s.blockSamples * sizeof(int16_t);

  // 4) 総書き込みバイトと“頭出しドロップ”（立ち上がりノイズ/クリック対策）
  const uint32_t totalBytes = s.sampleRate * recSeconds * s.channels * bytesPerSample;
  uint32_t written = 0;
  uint32_t dropBytes = (s.dropHeadMs * s.sampleRate / 1000) * s.channels * bytesPerSample;

  // 5) 読み→処理→書き込み をブロック単位で繰り返す
  while (written < totalBytes) {
    size_t br = 0;
    esp_err_t err = i2s_channel_read(rx_handle, bufPtr, bufBytes, &br, 200);
    if (err != ESP_OK) {
      f.close();
      return RecResult::I2sReadError;
    }
    if (br == 0) continue;  // タイムアウト等はスキップ

    const size_t samples = br / sizeof(int16_t);

    // (A) DCブロック：直流成分/オフセットを取り除く（クリップ・ポンピング防止の下地作り）
    dcBlocker(bufPtr, samples, dcAlpha);

    // (B) 固定ゲイン適用＋セーフティリミッタ：+40 dB ≈ 100倍等の"振幅倍率"で持ち上げ
    //     ただし、事前ピークで安全側に縮めてから適用 → 16bit範囲を超えないようにする
    applyFixedGain(bufPtr, samples, db2lin(g.gainDb));

    // (C) 録音開始直後の "ゴミ" を数百msだけ捨てる（クリック/起動ノイズ対策）
    size_t advance = 0;
    if (dropBytes > 0) {
      advance = (br <= dropBytes) ? br : dropBytes;
      dropBytes -= advance;
    }
    uint8_t* p = reinterpret_cast<uint8_t*>(bufPtr) + advance;
    size_t avail = br - advance;

    // (D) 末尾ちょうどで切る（総バイト数をオーバーしない）
    const uint32_t remain = totalBytes - written;
    const size_t to_write = (avail > remain) ? remain : avail;
    if (to_write > 0) {
      size_t w = f.write(p, to_write);
      if (w != to_write) {
        f.close();
        return RecResult::SdWriteError;
      }
      written += w;
    }
  }

  // 6) WAVヘッダを正しいサイズで上書きして完了
  writeWavHeader(f, s.sampleRate, s.bitsPerSamp, s.channels);
  f.close();
  if (outPath) *outPath = path;
  if (outBytes) *outBytes = written;
  return RecResult::Success;
}


RecResult recordingFixedEx(uint32_t recSeconds,
                           const SessionConfig* sessionOpt,
                           const FixedGainConfig* gainOpt,
                           String* outPath,
                           uint32_t* outBytes) {
  SessionConfig s = g_defSession;
  FixedGainConfig g = g_defFixedGain;
  if (sessionOpt) s = *sessionOpt;
  if (gainOpt) g = *gainOpt;

  return doRecordingFixedSeconds(recSeconds, s, g, outPath, outBytes);
}

// ======================= 録音：オートゲイン（処理部コメント強化版） =======================
static RecResult doRecordingAutoSeconds(uint32_t recSeconds,
                                        const SessionConfig& s,
                                        const AgcConfig& a,
                                        String* outPath,
                                        uint32_t* outBytes) {
  const uint16_t bytesPerSample = s.bitsPerSamp / 8;
  String path = nextWavPath(s.dir);
  File f = SD.open(path.c_str(), FILE_WRITE);
  if (!f) return RecResult::FileOpenError;

  // 1) WAVヘッダの予約（後で上書き）
  uint8_t zero[44] = { 0 };
  if (f.write(zero, 44) != 44) {
    f.close();
    return RecResult::HeaderPlaceWriteError;
  }
  f.flush();

  // 2) 状態初期化（DCフィルタ・AGCゲイン）
  dcReset();
  float agc_lin_gain = 1.0f;  // 初期ゲイン=等倍（0 dB）
  const float dcAlpha = dc_alpha_for(s.sampleRate);

  // 3) バッファ確保
  int16_t* bufPtr = nullptr;
  std::unique_ptr<int16_t[]> bufHolder;
  if (s.extBuffer && s.extBufSamps >= s.blockSamples) {
    bufPtr = s.extBuffer;
  } else {
    bufHolder.reset(new int16_t[s.blockSamples]);
    bufPtr = bufHolder.get();
  }
  const size_t bufBytes = s.blockSamples * sizeof(int16_t);

  // 4) 総バイト＆頭出しドロップ
  const uint32_t totalBytes = s.sampleRate * recSeconds * s.channels * bytesPerSample;
  uint32_t written = 0;
  uint32_t dropBytes = (s.dropHeadMs * s.sampleRate / 1000) * s.channels * bytesPerSample;

  while (written < totalBytes) {
    size_t br = 0;
    esp_err_t err = i2s_channel_read(rx_handle, bufPtr, bufBytes, &br, 200);
    if (err != ESP_OK) {
      f.close();
      return RecResult::I2sReadError;
    }
    if (br == 0) continue;

    const size_t samples = br / sizeof(int16_t);

    // (A) DCブロック
    dcBlocker(bufPtr, samples, dcAlpha);

    // (B) ブロックRMSから“今必要な倍率”を見積り → アタック/リリース/ゲートで滑らかに更新
    //     例: targetPeakDbFS=-3dBFS ≈ 0.707FS、maxGainDb=+36dB ≈ 63x
    //     無音（RMSが -60 dBFS 未満）の時は暴走を防ぐため追従を鈍らせます。
    const float rms = block_rms(bufPtr, samples);
    agc_lin_gain = agc_update_gain(agc_lin_gain, rms, a, samples, s.sampleRate);

    // (C) セーフティリミッタ込みでゲイン適用
    //     AGCで上げても、事前ピークで安全側に縮めた後に適用するため、クリップはしにくい設計。
    applyFixedGain(bufPtr, samples, agc_lin_gain);

    // (D) 立ち上がりの不要部分をドロップ
    size_t advance = 0;
    if (dropBytes > 0) {
      advance = (br <= dropBytes) ? br : dropBytes;
      dropBytes -= advance;
    }
    uint8_t* p = reinterpret_cast<uint8_t*>(bufPtr) + advance;
    size_t avail = br - advance;

    // (E) 末尾でちょうど切る
    const uint32_t remain = totalBytes - written;
    const size_t to_write = (avail > remain) ? remain : avail;
    if (to_write > 0) {
      size_t w = f.write(p, to_write);
      if (w != to_write) {
        f.close();
        return RecResult::SdWriteError;
      }
      written += w;
    }
  }

  // 5) ヘッダ上書きで完了
  writeWavHeader(f, s.sampleRate, s.bitsPerSamp, s.channels);
  f.close();
  if (outPath) *outPath = path;
  if (outBytes) *outBytes = written;
  return RecResult::Success;
}


RecResult recordingAutoEx(uint32_t recSeconds,
                          const SessionConfig* sessionOpt,
                          const AgcConfig* agcOpt,
                          String* outPath,
                          uint32_t* outBytes) {
  SessionConfig s = g_defSession;
  AgcConfig a = g_defAgc;
  if (sessionOpt) s = *sessionOpt;
  if (agcOpt) a = *agcOpt;

  return doRecordingAutoSeconds(recSeconds, s, a, outPath, outBytes);
}

// シンプルAPI（秒数つき）
RecResult recordingAuto(uint32_t recSeconds,
                        String* outPath,
                        uint32_t* outBytes) {
  SessionConfig s = g_defSession;
  AgcConfig a = g_defAgc;
  return doRecordingAutoSeconds(recSeconds, s, a, outPath, outBytes);
}

RecResult recording(uint32_t recSeconds, String* outPath, uint32_t* outBytes) {
  return recordingAuto(recSeconds, outPath, outBytes);
}
RecResult recording(uint32_t recSeconds) {
  return recording(recSeconds, nullptr, nullptr);
}
