#ifndef _MIC_H_
#define _MIC_H_ 1

//CAM S3 UnitもCAM S3 Unit-5MPもどちらも同じで動く
#define CAMERA_MODEL_M5STACK_CAMS3_UNIT_5MP
#define CAMERA_MODEL_M5STACK_CAMS3_UNIT
#include "mic_pins.h"
#include "sdcard_pins.h"

// 録音結果の列挙型
enum class RecResult {
  Success,
  FileOpenError,
  HeaderPlaceWriteError,
  I2sReadError,
  SdWriteError,
};

// ---- セッション設定（録音ごと/既定値ベース）----
struct SessionConfig {
  uint32_t sampleRate = 16000;   // Hz
  uint16_t bitsPerSamp = 16;     // 16のみ想定
  uint8_t channels = 1;          // mono
  uint32_t dropHeadMs = 700;     // ms
  uint16_t blockSamples = 1024;  // I/Oブロック長（samples）
  const char* dir = "/audio";    // 保存ディレクトリ
  int16_t* extBuffer = nullptr;  // 外部バッファ（任意）
  size_t extBufSamps = 0;        // 外部バッファ長（samples）
};

// ---- 固定ゲイン設定 ----
struct FixedGainConfig {
  // gainDb は「振幅（電圧）に対する dB」です。適用ゲイン(線形) = 10^(dB/20)。
  //
  // 目安（振幅倍率・クリップのしやすさの感覚に直結）:
  //   +40 dB ≈ 100.0x  ← 今の既定値。小さな入力も一気に持ち上がる。リミッタ無しだと即クリップしやすい
  //   +20 dB ≈ 10.0x
  //    +6 dB ≈ 2.0x     ← 「振幅が約2倍」
  //    +3 dB ≈ 1.41x
  //     0 dB = 1.0x     ← 変化なし
  //    -6 dB ≈ 0.5x     ← 「振幅が約半分」
  //   -12 dB ≈ 0.25x
  //
  // ※注意※
  //  - 上の倍率は「振幅」の話です。人の「聴感上の大きさ」はおおむね +10 dB で約2倍に感じることが多いです。
  //  - 本実装はセーフティ・リミッタがあるため、ある程度の過大ゲインでもクリップは回避されますが、
  //    ノイズも同時に持ち上げる点は変わりません。必要に応じて適切な値に下げてください。
  float gainDb = 40.0f;  // dB（振幅dB）
};

// ---- オートゲイン（AGC）設定 ----
struct AgcConfig {
  // targetPeakDbFS: 目標ピーク（dBFS）。0 dBFS がフルスケール（±32767）。
  //   -3 dBFS ≈ 0.707 FS（約70%の振幅）← 既定
  //   -6 dBFS ≈ 0.50  FS（約半分の振幅）
  // クリップ余裕を確保したいほど、負の値を大きく（例: -6 ～ -9 dBFS）。
  float targetPeakDbFS = -3.0f;

  // maxGainDb / minGainDb: AGC が適用できるゲインの上限・下限（振幅dB）。
  //   例）maxGainDb = +36 dB → 最大で約 63.1 倍まで持ち上げ
  //       minGainDb =   0 dB → 最低でも等倍（下げない）
  //   下げる動作も許容したいなら minGainDb を負に（例: -12 dB ≈ 0.25x）。
  float maxGainDb = 36.0f;  // 上限ゲイン（+6 dB ≈ 2x, +20 dB ≈ 10x, +36 dB ≈ 63x）
  float minGainDb = 0.0f;   // 下限ゲイン（0 dB=等倍。下げたいなら負値に）

  // attackMs / releaseMs: ゲインの追従速度（ブロック単位で一次フィルタ）。
  //   attackMs  … 音が急に大きくなった時に「下げる速さ」（小さいほど素早く抑える）
  //   releaseMs … 音が小さくなった時に「上げる速さ」（大きいほどゆっくり戻す）
  //   目安：attack 10–100 ms / release 200–1000 ms。
  //   速すぎるとポンピング感、遅すぎるとレベルが安定しない場合があります。
  float attackMs = 50.0f;    // 追従（Down方向：速い）
  float releaseMs = 500.0f;  // 追従（Up方向：遅い）

  // noiseGateDbFS: 無音に近い時のゲイン暴走を抑える閾値（RMS, dBFS）。
  //   -60 dBFS ≈ 0.1% FS（振幅比 ≈ 0.001）。これ未満のブロックではゲイン更新を鈍くします。
  // gateReleaseMs: ゲート解除までの時定数（大きいほど静音時はゆっくり動く）。
  float noiseGateDbFS = -60.0f;   // 無音ゲート閾値（RMS）
  float gateReleaseMs = 1000.0f;  // ゲート解除の時定数

  // 参考（dB↔倍率の感覚）:
  //   振幅 +6 dB ≈ 2x、+20 dB ≈ 10x、+40 dB ≈ 100x
  //   振幅 -6 dB ≈ 0.5x、-20 dB ≈ 0.1x
  //   dBFS の -3 dB ≈ 0.707 FS、-6 dB ≈ 0.5 FS
  //   線形換算式: 振幅倍率 = 10^(dB/20)
};


// ---- 初期化 ----
bool micInit();  // PDM自動検出（既定セッションの sampleRate を使用）

// ---- シンプルAPI（設定いらず）----
RecResult recordingFixed(uint32_t recSeconds,
                         String* outPath = nullptr,
                         uint32_t* outBytes = nullptr);
RecResult recordingAuto(uint32_t recSeconds,
                        String* outPath = nullptr,
                        uint32_t* outBytes = nullptr);

// ★ ここを修正：Ex にも秒数を追加
RecResult recordingFixedEx(uint32_t recSeconds,
                           const SessionConfig* sessionOpt,
                           const FixedGainConfig* gainOpt,
                           String* outPath,
                           uint32_t* outBytes);

RecResult recordingAutoEx(uint32_t recSeconds,
                          const SessionConfig* sessionOpt,
                          const AgcConfig* agcOpt,
                          String* outPath,
                          uint32_t* outBytes);

// ---- 既定値の取得/設定（Get → 比較 → 必要な差分だけ Set）----
SessionConfig getDefaultSession();
FixedGainConfig getDefaultFixedGain();
AgcConfig getDefaultAgc();

void setDefaultSession(const SessionConfig& s);
void setDefaultFixedGain(const FixedGainConfig& g);
void setDefaultAgc(const AgcConfig& a);

// ---- 後方互換：既存API ----
RecResult recording(uint32_t recSeconds);
RecResult recording(uint32_t recSeconds, String* outPath, uint32_t* outBytes);

#endif  // _MIC_H_