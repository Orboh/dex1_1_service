# test_gripper_range.cpp 解説

## このプログラムで何ができるか

ターミナルから `o` または `q` を入力するだけで、グリッパーを開いたり閉じたりできるプログラムです。

```
o を入力 → グリッパーが 5.2 rad（開き）に移動
q を入力 → グリッパーが 4.4 rad（閉じ）に移動
x を入力 → プログラム終了
```

---

## 全体の流れ

```
起動
 │
 ├─ 1. 初期化（DDS接続・モーター設定）
 │
 ├─ 2. Approachフェーズ（2秒かけて安全に開き位置へ移動）
 │
 ├─ 3. 入力スレッド 開始  ←── o/q/x の入力を待つ
 │
 └─ 4. 制御ループ（5ms ごとにモーターへ命令を送り続ける）
         └── x を入力すると終了
```

---

## ① パラメータ定義（23〜25行目）

```cpp
static constexpr float Q_OPEN        = 5.2f;  // 開き位置 [rad]
static constexpr float Q_CLOSE       = 4.4f;  // 閉じ位置 [rad]
static constexpr float APPROACH_TIME = 2.0f;  // 起動時の初期移動時間 [sec]
```

`constexpr` は「コンパイル時に確定する定数」です。  
値を変えたい場合はここだけ修正してビルドしなおします。

> **Q_OPEN を 5.4（機械的限界）にしてはいけない理由**  
> モーターが壁に当たり続けると過電流保護が働き、フォルト状態になります。  
> 限界より 0.2 rad 手前の 5.2 rad を上限にしています。

---

## ② グローバル変数（28〜36行目）

```cpp
std::atomic<float> cmd_q(Q_OPEN);   // 現在の目標角度
std::atomic<bool>  running(true);   // false になるとプログラム終了

std::atomic<bool>  measuring(false); // 到達時間の計測中かどうか
std::chrono::steady_clock::time_point measure_start; // 計測開始時刻
std::atomic<float> elapsed_ms(-1.0f); // 到達までにかかった時間 [ms]
std::atomic<int>   stable_count(0);   // 到達安定カウント

std::mutex io_mutex; // 画面出力の排他ロック
```

### `std::atomic` とは？

2つのスレッド（入力スレッドと制御ループ）が同じ変数を同時に読み書きすると  
データが壊れる可能性があります。`atomic` を使うと**読み書きが安全に行われる**ことが保証されます。

```
入力スレッド       制御ループ
    │                  │
    │  cmd_q = 5.2    │
    │ ──────────────> │  cmd_q.load() → 5.2  ← 安全に読める
```

### `std::mutex` とは？

画面への出力は2つのスレッドから行われます。同時に書き込むと表示が乱れるため、  
`io_mutex` でロックを取り、一方が終わるまでもう一方を待たせます。

---

## ③ 入力スレッド `input_thread()`（41〜84行目）

```cpp
void input_thread()
{
    while (running) {
        // プロンプトを表示
        std::cout << "\033[4;1H";  // ← カーソルを4行目に移動
        std::cout << "command [o=open(5.2), q=close(4.4), x=quit]: ";

        std::string line;
        std::getline(std::cin, line);  // Enter まで入力を待つ

        char cmd = line[0];  // 最初の1文字だけ見る

        if      (cmd == 'o') target = Q_OPEN;   // 開く
        else if (cmd == 'q') target = Q_CLOSE;  // 閉じる
        else if (cmd == 'x') { running = false; break; }  // 終了

        cmd_q         = target;        // 目標角度を更新（atomic）
        measuring     = true;          // 計測開始フラグON
        elapsed_ms    = -1.0f;         // 前回の計測結果をリセット
        stable_count  = 0;
        measure_start = now();         // 計測開始時刻を記録
    }
}
```

### `\033[4;1H` とは？

**ANSIエスケープコード**という画面制御コードです。

| コード | 意味 |
|--------|------|
| `\033[2J` | 画面全体をクリア |
| `\033[1;1H` | カーソルを1行1列に移動 |
| `\033[4;1H` | カーソルを4行1列に移動 |
| `\033[s` | 現在のカーソル位置を保存 |
| `\033[u` | 保存したカーソル位置に戻る |
| `\033[K` | カーソル位置から行末まで消去 |

これらを使うことで、画面全体を再描画せずに特定の行だけ更新できます。

---

## ④ 状態表示 `print_status()`（86〜112行目）

```cpp
void print_status(bool use_left, float q_actual, float q_target, float elapsed)
{
    std::lock_guard<std::mutex> lk(io_mutex);  // ロック取得（関数終了で自動解放）
    std::cout << "\033[s";  // 現在のカーソル位置を保存

    // 1行目: 実際の角度
    std::cout << "\033[1;1H";
    std::snprintf(buf, sizeof(buf), "%.3f", q_actual);  // 小数点3桁でフォーマット
    std::cout << "Right  actual: " << buf << " rad";

    // 2行目: 目標角度
    std::cout << "\033[2;1H";
    std::cout << "       target: " << buf << " rad";

    // 3行目: 到達時間
    std::cout << "\033[3;1H";
    std::cout << "Time to target (ms): " << static_cast<int>(elapsed);

    std::cout << "\033[u";  // 保存したカーソル位置に戻す（プロンプトの位置）
}
```

カーソルを保存→1〜3行目を書き換え→元の位置に戻すことで、  
**入力プロンプトを邪魔せず**に表示を更新します。

```
1行目: Right  actual: 5.200 rad   ← 常に最新の実角度
2行目:        target: 5.200 rad   ← 現在の目標
3行目: Time to target (ms): 73    ← 前回の到達時間
4行目: command [o=open...]:       ← 入力プロンプト（カーソルはここ）
```

---

## ⑤ main() の初期化（114〜144行目）

```cpp
// コマンドライン引数の解析（--left または --right）
auto vm = param::helper_test(argc, argv);

// DDS通信の初期化（ネットワーク経由でモーターとやりとり）
unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

// Subscriber（状態を受け取る） と Publisher（命令を送る）を作成
sub = make_shared<SubscriptionBase>("rt/dex1/right/state");
pub = make_shared<RealTimePublisher>("rt/dex1/right/cmd");
sub->wait_for_connection();  // サーバーと繋がるまで待機

// モーター制御モードの設定
pub->msg_.cmds()[0].mode() = 1;      // FOC位置制御モード
pub->msg_.cmds()[0].kp()   = 5.0f;  // 位置ゲイン
pub->msg_.cmds()[0].kd()   = 0.05f; // 速度ゲイン（ダンパー）
```

### DDSトピックとは？

モーターとプログラムは**DDS（Data Distribution Service）**というネットワークプロトコルで通信します。

```
このプログラム                         dex1_1_gripper_server（サーバー）
    │                                           │
    │── "rt/dex1/right/cmd" ──────────────────>│  命令を送る
    │                                           │
    │<── "rt/dex1/right/state" ────────────────│  状態を受け取る
```

### kp・kd とは？

モーターは **PD制御**（比例・微分制御）で目標角度に追従します。

```
トルク = kp × (目標角度 − 現在角度)   ← 位置誤差を打ち消す力
       + kd × (0 − 現在速度)          ← 速く動きすぎないようブレーキをかける
```

- `kp = 5.0`：位置ゲイン。大きいほど強い力で目標に引き寄せる。
- `kd = 0.05`：速度ゲイン。大きいほど振動が抑えられる。

---

## ⑥ Approachフェーズ（149〜162行目）

```cpp
// 現在位置を読む
float q_init = sub->msg_.states()[0].q();

auto t_start = now();
while (true) {
    float elapsed = (now() - t_start).count();             // 経過時間
    float ratio   = clamp(elapsed / APPROACH_TIME, 0, 1); // 0.0 → 1.0 に増加
    float q       = q_init + (Q_OPEN - q_init) * ratio;   // 線形補間
    pub->cmds()[0].q() = q;
    pub->unlockAndPublish();
    if (elapsed >= APPROACH_TIME) break;
    sleep(5ms);
}
```

### 線形補間とは？

「AからBへ一定速度で移動する」計算です。

```
ratio = 0.0 のとき → q = q_init（現在位置）
ratio = 0.5 のとき → q = 中間
ratio = 1.0 のとき → q = Q_OPEN（目標位置）
```

例: `q_init = 3.0`、`Q_OPEN = 5.2`、`APPROACH_TIME = 2.0 sec`

```
時刻 0.0s → ratio=0.00 → q = 3.0 + (5.2-3.0)×0.00 = 3.000 rad
時刻 0.5s → ratio=0.25 → q = 3.0 + (5.2-3.0)×0.25 = 3.550 rad
時刻 1.0s → ratio=0.50 → q = 3.0 + (5.2-3.0)×0.50 = 4.100 rad
時刻 2.0s → ratio=1.00 → q = 3.0 + (5.2-3.0)×1.00 = 5.200 rad
```

急に大きな角度を送ると衝撃が起きるため、この処理で**滑らかに**開き位置まで移動します。

---

## ⑦ メイン制御ループ（168〜190行目）

```cpp
while (running) {
    float target   = cmd_q.load();              // 入力スレッドが設定した目標角度を読む
    float q_actual = sub->msg_.states()[0].q(); // モーターの現在角度を読む

    pub->msg_.cmds()[0].q() = target;  // 目標角度をモーターに送る
    pub->unlockAndPublish();

    // 到達判定: 実角度が目標の ±0.05 rad 以内に入ったか
    bool within = fabs(q_actual - target) < 0.05f;
    if (measuring && within && stable_count == 0) {
        elapsed_ms   = (now() - measure_start) [ms]; // 到達時間を記録
        measuring    = false;
        stable_count = 1;
    }
    if (!within) stable_count = 0; // 範囲を外れたらリセット

    print_status(...);
    sleep(5ms);  // 5ms待機 → 200Hz制御
}
```

### なぜ毎回 `unlockAndPublish()` を呼ぶのか？

Unitree SDK の `RealTimePublisher` は**タイムアウト検知**を持っています。  
コマンドが一定時間届かないとサーバーがブレーキモードに切り替わります。  
毎回 publish することで「プログラムが生きている」ことをサーバーに伝え続けます。

### 到達時間の計測の流れ

```
入力スレッド             制御ループ
    │                       │
    │  measuring = true     │
    │  measure_start = now  │
    │─────────────────────> │
    │                       │  毎5ms: |actual - target| < 0.05 ?
    │                       │      YES → elapsed_ms = now - measure_start
    │                       │           measuring = false
    │                       │
    │              画面に "Time to target: 73ms" を表示
```

---

## 2スレッドの役割まとめ

```
┌─────────────────────────────────┐   ┌──────────────────────────────────┐
│        入力スレッド              │   │          制御ループ (main)        │
│                                 │   │                                  │
│  getline() でキー入力を待つ      │   │  5ms ごとにループ                │
│                                 │   │                                  │
│  'o' → cmd_q = 5.2             │   │  pub.q() = cmd_q.load()          │
│  'q' → cmd_q = 4.4             │──>│  pub.unlockAndPublish()          │
│  'x' → running = false         │   │                                  │
│                                 │   │  到達判定 → elapsed_ms 更新      │
│  measuring = true               │   │                                  │
│  measure_start = now()          │   │  print_status()                  │
└─────────────────────────────────┘   └──────────────────────────────────┘
           共有変数: cmd_q, running, measuring, elapsed_ms（すべて atomic）
```

---

## ビルドと実行

```bash
cd /home/rad907/workspace/ishimaru/dex1_1_service/build
cmake .. && make -j4
```

```bash
# ターミナル1: サーバー起動
./bin/dex1_1_gripper_server

# ターミナル2: テストプログラム起動
./bin/test_dex1_1_gripper_range --right   # 右グリッパー
./bin/test_dex1_1_gripper_range --left    # 左グリッパー
```
