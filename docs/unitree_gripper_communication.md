# Unitree Dex1-1 グリッパー 通信・接続ガイド

## 概要

Unitree Dex1-1 グリッパーは、**シリアル通信（USB）** でモーターと通信し、**CycloneDDS** を介してロボットシステム全体にその状態を公開・受信する構成をとっています。`dex1_1_service` はこの **シリアル ↔ DDS ブリッジ** として動作します。

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ロボットシステム / PC                         │
│                                                                     │
│   ┌──────────────────┐       DDS (CycloneDDS)      ┌────────────┐  │
│   │  遠隔操作 / 制御  │ ◄──── rt/dex1/right/cmd ────►│            │  │
│   │  アプリケーション │       rt/dex1/left/cmd       │            │  │
│   │                  │ ◄──── rt/dex1/right/state ───│ dex1_1_    │  │
│   └──────────────────┘       rt/dex1/left/state     │ service    │  │
│                                                     │ (ブリッジ) │  │
│                                                     └─────┬──────┘  │
└───────────────────────────────────────────────────────────┼─────────┘
                                                            │ USB Serial
                                                            │ 6.0 Mbps
                                                   ┌────────▼────────┐
                                                   │  Unitree M4010  │
                                                   │  モーター ×2     │
                                                   │  ID 0: 右グリッパ │
                                                   │  ID 1: 左グリッパ │
                                                   └─────────────────┘
```

---

## 1. DDS 通信レイヤー

### 1.1 DDS とは

[CycloneDDS-and-UnitreeSDK-(en).md](../xr_teleoperate.wiki/CycloneDDS-and-UnitreeSDK-(en).md) に詳細が記載されているとおり、DDS（Data Distribution Service）は分散システム向けのミドルウェア標準です。Unitree SDK2 は Eclipse Cyclone DDS（v0.10.2）を採用しています。

| DDS の概念 | グリッパーでの役割 |
|---|---|
| **Domain ID** | `0`（実機）、`1`（シミュレーション） |
| **Topic 名** | `rt/dex1/right/cmd` など |
| **IDL 型** | `unitree_go::msg::dds_::MotorCmds_` など |
| **QoS** | BestEffort（状態）/ Reliable（コマンド） |

### 1.2 DDS トピック一覧

| 方向 | トピック名 | メッセージ型 | 対象 |
|---|---|---|---|
| コマンド → グリッパー | `rt/dex1/right/cmd` | `MotorCmds_` | 右グリッパー（Motor ID 0） |
| コマンド → グリッパー | `rt/dex1/left/cmd` | `MotorCmds_` | 左グリッパー（Motor ID 1） |
| グリッパー → 状態 | `rt/dex1/right/state` | `MotorStates_` | 右グリッパー状態 |
| グリッパー → 状態 | `rt/dex1/left/state` | `MotorStates_` | 左グリッパー状態 |

### 1.3 メッセージ型の構造

**コマンドメッセージ：`unitree_go::msg::dds_::MotorCmds_`**

```
MotorCmds_
└── cmds[0]  (MotorCmd_)
    ├── mode   : uint8   // 1 = FOC（位置制御）モード
    ├── q      : float32 // 目標位置 [rad]
    ├── dq     : float32 // 目標速度 [rad/s]
    ├── tau    : float32 // 目標トルク [N·m]
    ├── kp     : float32 // 位置比例ゲイン [N·m/rad]
    └── kd     : float32 // 速度微分ゲイン [N·m/(rad/s)]
```

**状態メッセージ：`unitree_go::msg::dds_::MotorStates_`**

```
MotorStates_
└── states[0]  (MotorState_)
    ├── q        : float32 // 実際の位置 [rad]
    ├── dq       : float32 // 実際の速度 [rad/s]
    └── tau_est  : float32 // 推定トルク [N·m]
```

IDL ヘッダーは `unitree_sdk2` が提供します：
- `unitree/idl/go2/MotorCmds_.hpp`
- `unitree/idl/go2/MotorStates_.hpp`

### 1.4 ネットワーク設定

```
ロボット PC1  : 192.168.123.161/24
ロボット PC2  : 192.168.123.164/24
開発 PC       : 192.168.123.x/24（推奨: 192.168.123.99 または .222）
```

DDS 起動コマンド（`-n` でネットワークインターフェースを指定）：

```bash
sudo ./dex1_1_gripper_server --network eth0
```

DDS 通信確認：

```bash
# トピックが見えるか確認
cyclonedds ps
# 詳細な QoS や型情報を確認
cyclonedds ls
# トピックの中身をリアルタイム確認
cyclonedds subscribe rt/dex1/right/state
```

---

## 2. シリアル通信レイヤー

### 2.1 モーターとの直接接続

グリッパーモーター（Unitree M4010）とは **USB シリアル** で通信します。

| 項目 | 値 |
|---|---|
| ボーレート | **6.0 Mbps** |
| デバイスパス | `/dev/ttyUSB*`、`/dev/ttyCH343USB*` |
| 制御ループ | **2000 Hz**（500 µs 周期） |
| モーター型 | Unitree M4010 |

対応デバイスの例：
- `/dev/ttyUSB0`、`/dev/ttyUSB1`（一般的な USB シリアル）
- `/dev/ttyCH343USB0`（新型シリアルハブ）

### 2.2 ギア比変換

DDS で受け取ったコマンドはギア比を使ってモーター制御値に変換されます：

```
ギア比 = queryGearRatio(MotorType::M4010)

// DDS コマンド → モーター制御値（コマンド方向）
motor_kp  = dds_kp  / (gear_ratio²)
motor_kd  = dds_kd  / (gear_ratio²)
motor_q   = dds_q   × gear_ratio
motor_dq  = dds_dq  × gear_ratio
motor_tau = dds_tau / gear_ratio

// モーター状態 → DDS 状態（状態方向）
dds_q       = motor_q   / gear_ratio
dds_dq      = motor_dq  / gear_ratio
dds_tau_est = motor_tau × gear_ratio
```

---

## 3. グリッパーの位置範囲

右グリッパーの実測値（[dex1_1_gripper_range.md](dex1_1_gripper_range.md) より）：

| 状態 | 位置 [rad] | 備考 |
|---|---|---|
| 閉じた位置 | 4.4 rad | `Q_CLOSE` |
| 安全な開き位置 | 5.2 rad | `Q_OPEN`（安全マージン 0.2 rad） |
| 機械的限界（開） | 5.4 rad | これ以上は過電流故障 |
| キャリブレーション範囲 | 5.62 rad（322°） | モーター限界 |

移動範囲：`Q_OPEN - Q_CLOSE = 0.8 rad`  
典型的な移動時間：約 73 ms

---

## 4. 標準制御パラメータ

テストプログラムで使用されている標準値：

```cpp
mode = 1      // FOC（Field-Oriented Control）モード
kp   = 5.0f   // 位置比例ゲイン [N·m/rad]
kd   = 0.05f  // 速度微分ゲイン [N·m/(rad/s)]
```

制御ループ周波数：**200 Hz**（5 ms 周期）

---

## 5. システム起動シーケンス

```
1. シリアルポートスキャン
   └── /dev/ttyUSB* を順にスキャン
   └── Motor ID 0（右）と ID 1（左）を探索

2. シリアル接続確立
   └── ボーレート 6.0 Mbps で M4010 に接続

3. DDS 初期化
   └── ChannelFactory::Instance()->Init(domain_id, network_if)
   └── 各モーターの Publisher / Subscriber を作成

4. 制御ループ開始
   └── RecurrentThread で 2 kHz の制御スレッドを起動
   └── タイムアウト監視（コマンドが来なければ BRAKE モード）

5. 運用
   └── コマンド受信 → ギア比変換 → シリアル送信
   └── シリアル受信 → ギア比変換 → DDS パブリッシュ
```

キャリブレーションモード（起動時）：

```bash
sudo ./dex1_1_gripper_server --network eth0 -c
```

---

## 6. Python での DDS 購読例

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_

def state_callback(msg: MotorStates_):
    q   = msg.states()[0].q()
    dq  = msg.states()[0].dq()
    tau = msg.states()[0].tau_est()
    print(f"q={q:.4f} rad, dq={dq:.4f} rad/s, tau={tau:.4f} N·m")

# Domain ID 0（実機）、eth0 を使用
ChannelFactoryInitialize(0, "eth0")

sub = ChannelSubscriber("rt/dex1/right/state", MotorStates_)
sub.Init(state_callback, 10)
```

---

## 7. SDK との関係

### 7.1 使用する SDK の全体像

```
dex1_1_service が依存する SDK / ライブラリ

┌──────────────────────────────────────────────────────────────┐
│                     dex1_1_service                           │
│                                                              │
│  ┌─────────────────────────┐  ┌──────────────────────────┐  │
│  │    unitree_sdk2 (C++)    │  │  UnitreeMotorSDK (.so)   │  │
│  │  ─────────────────────  │  │  ────────────────────    │  │
│  │  ChannelFactory::Init() │  │  シリアル通信プロトコル   │  │
│  │  ChannelPublisher<T>    │  │  M4010 コマンド生成       │  │
│  │  ChannelSubscriber<T>   │  │  queryGearRatio()         │  │
│  │  RecurrentThread        │  │  MotorCmd / MotorData     │  │
│  │  IDL 型定義              │  │  libUnitreeMotorSDK*.so  │  │
│  └────────────┬────────────┘  └──────────┬───────────────┘  │
│               │ CycloneDDS                │ libserialport     │
│  ┌────────────▼────────────┐  ┌──────────▼───────────────┐  │
│  │  ddsc / ddscxx (DDS)    │  │  /dev/ttyUSB*             │  │
│  │  libddsc.so             │  │  USB シリアルポート        │  │
│  │  libddscxx.so           │  └──────────────────────────┘  │
│  └─────────────────────────┘                                 │
└──────────────────────────────────────────────────────────────┘
```

### 7.2 unitree_sdk2（C++ SDK）

DDS 通信の**すべて**を unitree_sdk2 に依存しています。

**初期化：**
```cpp
// domain_id=0（実機）、ネットワークIF を指定
unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
```

**Publisher（コマンド送信）：**
```cpp
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/MotorCmds_.hpp>

// RealTimePublisher はロックフリーで 2kHz 制御ループからも安全に使える
std::shared_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>> pub_;
pub_ = std::make_shared<...>("rt/dex1/right/cmd");
pub_->InitChannel();
pub_->Write(msg);
```

**Subscriber（状態受信）：**
```cpp
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>

std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>> sub_;
sub_ = std::make_shared<...>("rt/dex1/right/state", callback);
sub_->InitChannel();
```

**制御スレッド：**
```cpp
#include <unitree/common/thread/thread.hpp>

// 2000 Hz（500 µs）の周期スレッド
std::unique_ptr<unitree::common::RecurrentThread> ctrl_thread_;
ctrl_thread_ = unitree::common::CreateRecurrentThreadEx(
    "motor_ctrl", RT_SCHED_FIFO, 99, 500'000 /*ns*/, &MotorUnit::Run, this);
```

**IDL 型（メッセージ定義）：**
```
unitree_sdk2/include/unitree/idl/go2/
├── MotorCmds_.hpp    ← コマンドメッセージ型
└── MotorStates_.hpp  ← 状態メッセージ型
```

### 7.3 UnitreeMotorSDK（モーター制御ライブラリ）

USB シリアル経由のモーター制御は **UnitreeMotorSDK** が担当します。  
バイナリライブラリとして提供されています。

| ファイル | アーキテクチャ |
|---|---|
| `lib/libUnitreeMotorSDK_Arm64.so` | ARM64（ロボット本体） |
| `lib/libUnitreeMotorSDK_Linux64.so` | x86_64（開発 PC） |

提供する主な機能：

```cpp
#include "unitreeMotor/unitreeMotor.h"

// ギア比取得（DDS 値 ↔ モーター値の変換に使用）
double ratio = queryGearRatio(MotorType::M4010);

// コマンド構造体
MotorCmd cmd;
cmd.id        = 0;           // Motor ID
cmd.motorType = MotorType::M4010;
cmd.mode      = 1;           // FOC モード
cmd.kp        = kp_motor;
cmd.kd        = kd_motor;
cmd.q         = q_motor;     // ギア比変換済み

// 状態構造体
MotorData data;
data.q   // 実際の位置（ギア比変換前）
data.dq  // 実際の速度
data.tau // 推定トルク
```

### 7.4 カスタム DDS ラッパー（プロジェクト内）

`dex1_1_service` は unitree_sdk2 の上に薄いラッパーを追加しています：

| ファイル | クラス | 役割 |
|---|---|---|
| [include/dds/Publisher.h](../dex1_1_service/include/dds/Publisher.h) | `PublisherBase<T>`, `RealTimePublisher<T>` | ロックフリーパブリッシュ |
| [include/dds/Subscription.h](../dex1_1_service/include/dds/Subscription.h) | `SubscriptionBase<T>` | タイムアウト検知付きサブスクリプション |

### 7.5 CMake 依存関係まとめ

```cmake
# CMakeLists.txt より
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

include_directories(/usr/local/include/ddscxx)   # CycloneDDS C++ ヘッダ

target_link_libraries(... 
    unitree_sdk2          # Unitree C++ SDK
    ddsc                  # CycloneDDS C ランタイム
    ddscxx                # CycloneDDS C++ バインディング
    UnitreeMotorSDK_*     # モーター制御バイナリ
    boost_program_options
    yaml-cpp
    serialport            # USB シリアル
)
```

---

## 9. 関連ファイル

| ファイル | 説明 |
|---|---|
| [dex1_1_service/main.cpp](../dex1_1_service/main.cpp) | DDS ↔ シリアルブリッジのメイン実装 |
| [dex1_1_service/test/test_gripper_range.cpp](../dex1_1_service/test/test_gripper_range.cpp) | グリッパー範囲テスト（open/close コマンド） |
| [dex1_1_service/test/test_gripper.cpp](../dex1_1_service/test/test_gripper.cpp) | 正弦波モーションテスト |
| [dex1_1_service/test/test_gripper2.cpp](../dex1_1_service/test/test_gripper2.cpp) | 任意角度入力テスト |
| [dex1_1_service/test/test_gripper_sub.py](../dex1_1_service/test/test_gripper_sub.py) | Python DDS 購読サンプル |
| [dex1_1_service/README.md](../dex1_1_service/README.md) | セットアップ・使い方（英語） |
| [docs/dex1_1_gripper_range.md](dex1_1_gripper_range.md) | グリッパー範囲の技術メモ（日本語） |
| [xr_teleoperate.wiki/CycloneDDS-and-UnitreeSDK-(en).md](../xr_teleoperate.wiki/CycloneDDS-and-UnitreeSDK-(en).md) | CycloneDDS と Unitree SDK の詳細解説 |

---

## 8. よくあるトラブルシューティング

| 症状 | 原因 | 対処法 |
|---|---|---|
| DDS トピックが見えない | ネットワーク設定の問題 | IP を `192.168.123.x/24` に設定、`ping 192.168.123.161` で確認 |
| 同じトピックで干渉 | Domain ID の衝突 | シミュレーション側を Domain ID `1` に変更 |
| グリッパーが動かない | タイムアウト（BRAKE モード） | コマンドを定期的に送信する（200 Hz 推奨） |
| シリアル接続失敗 | デバイスが見つからない | `ls /dev/ttyUSB*` でデバイス確認、権限付与 `sudo chmod 666 /dev/ttyUSB0` |
| 位置がずれている | キャリブレーション未実施 | `-c` フラグで起動してキャリブレーション実施 |
