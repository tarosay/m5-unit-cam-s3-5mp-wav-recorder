#include <Arduino.h>
#include <esp_camera.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "mic.h"

#define GPIO_0_NUM 0
#define BAURATE 115200

void setup() {
  Serial.begin(BAURATE);

  // IOG0を入力に設定し、プルアップ抵抗を有効にする
  pinMode(GPIO_0_NUM, INPUT_PULLUP);
  //青色LEDをOUTPUTに設定
  pinMode(LED_GPIO_NUM, OUTPUT);

  //LED消去
  digitalWrite(LED_GPIO_NUM, HIGH);

  // WiFiの無効化
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  Serial.println("WiFiを無効化しました");

  // SPIピンの設定
  SPI.begin(SD_CLK_GPIO_NUM, SD_MISO_GPIO_NUM, SD_MOSI_GPIO_NUM, SD_CS_GPIO_NUM);
  // SDカードの初期化
  if (!SD.begin(SD_CS_GPIO_NUM)) {
    Serial.println("SDカードのマウントに失敗しました");
    while (1) {
      digitalWrite(LED_GPIO_NUM, LOW);
      delay(150);
      digitalWrite(LED_GPIO_NUM, HIGH);
      delay(150);
    }
  }
  Serial.println("SDカードがマウントされました");

  // マイク初期化
  if (!micInit()) {
    Serial.println("マイクの初期化に失敗しました");
    while (1) {
      digitalWrite(LED_GPIO_NUM, LOW);
      delay(150);
      digitalWrite(LED_GPIO_NUM, HIGH);
      delay(150);
    }
  }
  Serial.println("マイクを初期化しました");

  // 30秒録音する
  static constexpr uint32_t REC_SECONDS = 10;
  
  // ───────────────────────────────────────────────────────────────
  // 今のテストで何を見るか（AGC版）
  // ───────────────────────────────────────────────────────────────
  // T1: 既定AGCの総合確認（長め）。大小の音で“ちょうど良い”レベルに収束するか/歪まないか。
  // T2: 既定AGCの短時間確認（API直呼び）→ T1と等価挙動のクイック版。
  // T3: 固定ゲイン比較（20/30/36/40/44 dB）→ AGC無しの基準を作って相対評価。
  // T4: AGCの目標ピークだけ -6 dBFS に **一時**変更 → クリップ余裕が増える効果を確認。
  // T5: 先頭ドロップを 200ms に変更 → 起動ノイズの短縮効果。
  // T7: **段階テスト（推し）**：無音→ささやき→普通→大きめ→無音（各区間8秒）を
  //     5ファイルで順に録る。AGCの追従と安定性を段階的に観察しやすい。
  //
  // 聴きどころ：
  //  - 大声で急に割れない（すぐ下げる）、小声で埋もれない（ゆっくり上げる）
  //  - 無音時にノイズが“ぐわっ”と持ち上がらない（ゲートが効く）
  //  - ポンピングが過度に目立たない（必要に応じ attack/release 調整）

  String saved;
  uint32_t bytes = 0;
  auto print = [&](const char* tag, RecResult r) {
    switch (r) {
      case RecResult::Success: Serial.printf("%s 成功: %s (%lu bytes)\n", tag, saved.c_str(), (unsigned long)bytes); break;
      case RecResult::FileOpenError: Serial.printf("%s 失敗: ファイルオープン失敗\n", tag); break;
      case RecResult::HeaderPlaceWriteError: Serial.printf("%s 失敗: ヘッダ仮書き込み失敗\n", tag); break;
      case RecResult::I2sReadError: Serial.printf("%s 失敗: I2S読み取り失敗\n", tag); break;
      case RecResult::SdWriteError: Serial.printf("%s 失敗: SD書き込み失敗\n", tag); break;
    }
  };

  Serial.println("テスト開始");

  // 短/長の基準秒数（必要に応じて調整）
  const uint32_t AGC_LONG_SEC = 20;  // ← AGCは“動き”を見るので長め
  const uint32_t AGC_SHORT_SEC = 8;  // 段階テストの各区間
  const uint32_t FIX_SHORT_SEC = 5;  // 固定ゲインは短めでもOK

  // ───────────────────────────
  // [T1] 既定オーバーロード（中身はAGC）— 長め
  // 目的: デフォルトAGCの総合確認（歪み/安定/収束）。
  // ───────────────────────────
  saved = "";
  bytes = 0;
  Serial.println("[T1] recording(20s) でオート録音（既定, 長め）");
  RecResult r1 = recording(AGC_LONG_SEC, &saved, &bytes);
  print("[T1]", r1);

  // ───────────────────────────
  // [T2] AGC直呼び（クイック確認）
  // 目的: T1と等価挙動のクイック版。
  // ───────────────────────────
  saved = "";
  bytes = 0;
  Serial.println("[T2] recordingAuto(8s) でオート録音（既定, 短め）");
  RecResult r2 = recordingAuto(AGC_SHORT_SEC, &saved, &bytes);
  print("[T2]", r2);

  // ───────────────────────────
  // [T3] 固定ゲイン比較（20/30/36/40/44 dB）
  // 目的: AGCなしの基準→AGCの良さを相対評価。
  // ───────────────────────────
  auto doFixedGain = [&](float gainDb, const char* tag) {
    FixedGainConfig fg = getDefaultFixedGain();
    fg.gainDb = gainDb;
    saved = "";
    bytes = 0;
    RecResult rr = recordingFixedEx(FIX_SHORT_SEC, nullptr, &fg, &saved, &bytes);
    print(tag, rr);
  };
  Serial.println("[T3] recordingFixedEx(5s) ゲイン比較");
  doFixedGain(20.0f, "[T3-20dB]");  // ≈10x
  doFixedGain(30.0f, "[T3-30dB]");  // ≈31.6x
  doFixedGain(36.0f, "[T3-36dB]");  // ≈63.1x
  doFixedGain(40.0f, "[T3-40dB]");  // ≈100x
  doFixedGain(44.0f, "[T3-44dB]");  // ≈158x

  // ───────────────────────────
  // [T4] AGC目標を -6 dBFS に“一時”変更（効果確認）
  // 目的: クリップ余裕↑ / 全体レベル↓ のバランスを見る。
  // ───────────────────────────
  {
    AgcConfig agc1 = getDefaultAgc();
    agc1.targetPeakDbFS = -6.0f;
    saved = "";
    bytes = 0;
    Serial.println("[T4] recordingAutoEx(8s, target=-6 dBFS)");
    RecResult r4 = recordingAutoEx(AGC_SHORT_SEC, nullptr, &agc1, &saved, &bytes);
    print("[T4]", r4);
  }

  // ───────────────────────────
  // [T5] 先頭ドロップ 200ms（起動ノイズ対策）
  // 目的: クリック/ゴミの減少効果。
  // ───────────────────────────
  {
    SessionConfig sess = getDefaultSession();
    sess.dropHeadMs = 200;
    saved = "";
    bytes = 0;
    Serial.println("[T5] recordingAutoEx(8s, dropHeadMs=200)");
    RecResult r5 = recordingAutoEx(AGC_SHORT_SEC, &sess, nullptr, &saved, &bytes);
    print("[T5]", r5);
  }

  // ───────────────────────────
  // [T7] 段階テスト（推奨）— 無音→ささやき→普通→大きめ→無音（各8s）
  // 目的: AGCの“追従と安定”を段階的に確認。
  // 実施方法:
  //   1) [SILENCE]   8秒 無音（できるだけ静かに）
  //   2) [WHISPER]   8秒 ささやき声
  //   3) [NORMAL]    8秒 普通の声
  //   4) [LOUD/CLAP] 8秒 大きめの声 or 手拍子
  //   5) [SILENCE]   8秒 再び無音
  // 観察ポイント:
  //   - 区間ごとに音量が“過不足なく”揃う／移り変わりで不自然なポンピングが強くない。
  //   - 大きい区間に入ると素早く抑え、静かな区間では緩やかに上げ、無音で暴走しない。
  // 備考:
  //   - 区間ごとに別ファイルで保存するので、後で波形比較しやすい。
  // ───────────────────────────
  auto step = [&](const char* tag, const char* guide) {
    Serial.printf("[T7-%s] %s\n", tag, guide);
    delay(1500);  // 軽い準備時間
    saved = "";
    bytes = 0;
    RecResult r = recordingAuto(AGC_SHORT_SEC, &saved, &bytes);
    print(tag, r);
  };
  step("SILENCE1", "8秒 無音で待機");
  step("WHISPER", "8秒 ささやき声で話す");
  step("NORMAL", "8秒 普通の声で話す");
  step("LOUD", "8秒 大きめの声/手拍子");
  step("SILENCE2", "8秒 無音で待機");

  Serial.println("テスト完了");
}

void loop() {
  digitalWrite(LED_GPIO_NUM, LOW);
  delay(350);
  digitalWrite(LED_GPIO_NUM, HIGH);
  delay(350);
}
