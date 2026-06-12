# グリッパー指定範囲開閉プログラム (test_gripper_range)

## 概要

`test_dex1_1_gripper_range` は、ターミナルからコマンド入力でグリッパーを  
指定角度（開き/閉じ）に動かすプログラムです。  
`test_gripper2.cpp` の入力スレッド構造をベースに作成しました。

- ソース: `test/test_gripper_range.cpp`
- 実行ファイル: `bin/test_dex1_1_gripper_range`

---

## 実測値（右グリッパー）

| 状態 | 角度 [rad] | 備考 |
|------|-----------|------|
| 開き機械的限界 | 5.4 rad | これ以上送ると過電流フォルト |
| 開き設定値 (Q_OPEN) | **5.2 rad** | 限界より 0.2 rad 手前（安全マージン） |
| 閉じ設定値 (Q_CLOSE) | **4.4 rad** | 実測での目標到達時間 73 ms |
| 移動幅 | 0.8 rad | Q_OPEN − Q_CLOSE |

---

## 操作コマンド

プログラム起動後、ターミナルに文字を入力して Enter を押します。

| 入力 | 動作 |
|-----|------|
| `o` | グリッパーを開く（Q_OPEN = 5.2 rad へ移動） |
| `q` | グリッパーを閉じる（Q_CLOSE = 4.4 rad へ移動） |
| `x` | プログラム終了 |

---

## 画面表示

```
Right  actual: 5.200 rad        ← 行1: 現在の実角度
       target: 5.200 rad        ← 行2: 現在の目標角度
Time to target (ms): 73         ← 行3: 前回コマンドの到達時間
command [o=open(5.2), q=close(4.4), x=quit]:   ← 行4: 入力プロンプト
```

---

## パラメータ

`test_gripper_range.cpp` 冒頭の `constexpr` で変更します。

```cpp
static constexpr float Q_OPEN        = 5.2f;  // 開き位置 [rad]
static constexpr float Q_CLOSE       = 4.4f;  // 閉じ位置 [rad]
static constexpr float APPROACH_TIME = 2.0f;  // 起動時の初期移動時間 [sec]
```

---

## プログラム構造

### スレッド構成

```
main スレッド         入力スレッド (input_thread)
    │                       │
    │  wait_for_connection   │
    │  ─────────────────>   │
    │                       │  getline(stdin) → 'o' or 'q' or 'x'
    │  cmd_q (atomic<float>) │
    │  <─────────────────   │  cmd_q = Q_OPEN or Q_CLOSE
    │                       │  measuring = true
    │  200Hz 制御ループ      │
    │  pub.q() = cmd_q      │
    │  unlockAndPublish()    │
```

- `cmd_q`: 目標角度をスレッド間で共有する `std::atomic<float>`
- `measuring`: 到達時間の計測中フラグ (`std::atomic<bool>`)
- `io_mutex`: 画面出力の排他制御

### 起動時の安全移動 (Approach フェーズ)

起動直後はグリッパーが任意の位置にある可能性があるため、  
`APPROACH_TIME` 秒かけて現在位置から Q_OPEN へ**線形補間**で移動します。

```
q(t) = q_init + (Q_OPEN - q_init) × clamp(t / APPROACH_TIME, 0, 1)
```

急激な動作を防ぐための安全処理です。

### 目標角度の更新（入力スレッド）

```
'o' 入力  →  cmd_q = Q_OPEN  (5.2 rad)
'q' 入力  →  cmd_q = Q_CLOSE (4.4 rad)
```

更新と同時に計測タイマーをリセット (`measure_start = now()`)。

### 制御ループ（メインスレッド, 5ms = 200Hz）

```cpp
pub.cmds()[0].q() = cmd_q.load();   // 毎サイクル目標角度を送信
pub.unlockAndPublish();
```

モーターは PD 制御で目標角度に追従します（kp=5.0, kd=0.05）。

### 到達判定

```
|q_actual - target| < 0.05 rad  かつ  stable_count == 0
  → elapsed_ms = (now - measure_start) [ms]
  → measuring = false
```

---

## 移動速度の目安

| 方向 | 距離 | 実測到達時間 |
|-----|------|------------|
| 閉 → 開 (4.4 → 5.2 rad) | 0.8 rad | ～73 ms |
| 開 → 閉 (5.2 → 4.4 rad) | 0.8 rad | ～73 ms |

速度は kp/kd と負荷によって変わります。`Time to target (ms)` で確認してください。

---

## モーター制御パラメータ

```cpp
cmd.mode() = 1;      // FOCモード（位置制御）
cmd.kp()   = 5.0f;  // 位置ゲイン [Nm/rad]
cmd.kd()   = 0.05f; // 速度ゲイン（ダンパー）[Nm/(rad/s)]
cmd.q()    = target; // 目標角度 [rad]
```

制御周期: **5 ms（200 Hz）**

---

## ビルド方法

```bash
cd /home/rad907/workspace/ishimaru/dex1_1_service/build
cmake .. && make -j4
```

CMakeLists.txt への追加行（追加済み）:

```cmake
add_executable(test_dex1_1_gripper_range  test/test_gripper_range.cpp)
```

---

## 実行方法

```bash
# サーバーを起動（別ターミナル）
./bin/dex1_1_gripper_server

# 右グリッパー
./bin/test_dex1_1_gripper_range --right

# 左グリッパー
./bin/test_dex1_1_gripper_range --left
```

---

## 注意事項

- **Q_OPEN を 5.4 rad（機械的限界）に設定しない。** モーターが限界に当たると過電流保護でフォルト状態になります。回復はサーバー再起動または電源再投入が必要です。
- 起動直後に Approach フェーズ（2秒）があります。その間は入力を受け付けません。
