# Dex1-1 グリッパー 環境構築手順

> 対象環境: Ubuntu 24.04 LTS / x86_64 (amd64)  
> 作成日: 2026-06-12

---

## 概要

Dex1-1 はUnitree製の平行2指グリッパーです。  
このドキュメントでは `dex1_1_service`（serial2dds サービス）のビルドから起動までの手順を説明します。

### 通信トピック構成

```
ユーザー -- rt/dex1/right/cmd --> Dex1-1 (motor_id=0, 右)
Dex1-1   -- rt/dex1/right/state --> ユーザー

ユーザー -- rt/dex1/left/cmd  --> Dex1-1 (motor_id=1, 左)
Dex1-1   -- rt/dex1/left/state  --> ユーザー
```

---

## 1. 依存パッケージのインストール

### 1-1. ビルドツール

```bash
sudo apt update
sudo apt install -y cmake build-essential
```

### 1-2. 依存ライブラリ

以下のコマンドで一括インストールしてください。**cmake も含めて事前にすべて入れておくことが重要です。**  
途中で不足ライブラリが出ると `cmake ..` のやり直しが必要になります。

```bash
sudo apt install -y \
    libserialport-dev \
    libspdlog-dev \
    libboost-all-dev \
    libyaml-cpp-dev \
    libfmt-dev \
    libeigen3-dev
```

> **よくある落とし穴**: `libspdlog-dev` が入っていない状態で `make` を実行すると  
> `spdlog/spdlog.h: No such file or directory` エラーになります。  
> 新たにライブラリをインストールした後は必ず `cmake ..` を再実行してください。
> ```bash
> # ライブラリ追加後の再ビルド手順
> cd /home/rad907/workspace/ishimaru/dex1_1_service/build
> cmake ..   # ← 必ず再実行してキャッシュを更新する
> make -j6
> ```

> **オフライン環境 (arm64) の場合**  
> `libserialport` は `dex1_1_service/lib/` にオフライン用 `.deb` が同梱されています。  
> インターネットが使えない場合は以下の順で手動インストールしてください（順序厳守）。
> ```bash
> sudo dpkg -i lib/libserialport0_0.1.1-3_arm64.deb
> sudo dpkg -i lib/libserialport-dev_0.1.1-3_arm64.deb
> ```
> ※ このマシン (amd64) では apt を使用してください。

---

## 2. unitree_sdk2 のビルド＆インストール

`dex1_1_service` は Unitree SDK2 のヘッダファイルに依存しています。  
先にビルドしてシステムにインストールする必要があります。

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2
cd unitree_sdk2
mkdir -p build && cd build
cmake .. -DBUILD_EXAMPLES=OFF
sudo make install
```

> **注意**: `-DBUILD_EXAMPLES=OFF` を付けないとサンプルコードのビルドも試みられ、  
> 追加の依存ライブラリ不足でエラーになることがあります。  
> ヘッダのインストールだけが目的なので必ずこのオプションを付けてください。

---

## 3. dex1_1_service のビルド

```bash
cd /home/rad907/workspace/ishimaru/dex1_1_service
mkdir -p build && cd build
cmake ..
make -j6
```

ビルド成功すると `build/` ディレクトリに以下の実行ファイルが生成されます。

| ファイル | 役割 |
|---|---|
| `dex1_1_gripper_server` | グリッパーサーバー本体 |
| `test_dex1_1_gripper_server` | 動作テスト用ツール |

---

## 4. 起動

### 4-1. サーバー起動

```bash
cd /home/rad907/workspace/ishimaru/dex1_1_service/build

# デフォルト (ネットワークインターフェース: eth0)
sudo ./dex1_1_gripper_server

# ネットワークインターフェースを明示する場合
sudo ./dex1_1_gripper_server --network eth0
```

> **ネットワークインターフェースの確認方法**  
> USBハブ経由でEthernetを接続している場合、`eth0` ではなく `eth1` などになることがあります。  
> `ip addr` または `ifconfig` で `192.168.123.*` のIPを持つインターフェース名を確認してください。
> ```bash
> ip addr | grep "192.168.123"
> ```

### 4-2. 動作テスト

```bash
# 左右両方テスト
sudo ./test_dex1_1_gripper_server -l -r

# 左のみ
sudo ./test_dex1_1_gripper_server --network eth0 -l

# 右のみ
sudo ./test_dex1_1_gripper_server -r
```

正常時の出力例：
```
[info] Right gripper init at q = 0.001
[info] Left gripper init at q = 0.000
R= 0.508 L= 0.502
```

---

## 5. キャリブレーション

> **motor_id の対応**: ID=0 → 右グリッパー / ID=1 → 左グリッパー

グリッパーを**手動で閉じた状態**にしてから以下を実行します。

```bash
sudo ./dex1_1_gripper_server -c
```

画面の指示に従い、キャリブレーションしたいモーターに対して `s` + `Enter` を押します。

```
> Please manually close the gripper tightly.
  Then press 's' + Enter to calibrate, or any other key to skip.
> s
[info] Calibrating motor 0...
Calibration successful!
```

---

## 6. 自動起動の設定（任意）

テストが完了した後、起動時に自動でサービスを開始したい場合は以下を実行します。

```bash
cd /home/rad907/workspace/ishimaru/dex1_1_service
bash setup_autostart.sh
```

> **注意**: `setup_autostart.sh` はデフォルトで `eth0` を使用します。  
> `eth0` が正しいインターフェースでない場合は、スクリプト内の `ExecStart` 行を編集してください。
> ```
> ExecStart=$SCRIPT_BIN/dex1_1_gripper_server -n eth1
> ```

---

## トラブルシューティング

### ビルドエラー: `unitree/idl/go2/MotorCmds_.hpp: No such file or directory`

unitree_sdk2 がインストールされていません。[手順2](#2-unitree_sdk2-のビルドインストール) を先に実行してください。

### ビルドエラー: `Could NOT find Boost`

```bash
sudo apt install -y libboost-all-dev
cd /home/rad907/workspace/ishimaru/dex1_1_service/build
cmake ..
make -j6
```

### ビルドエラー: `spdlog/spdlog.h: No such file or directory`

`libspdlog-dev` が未インストールです。インストール後に `cmake ..` の再実行が必要です。

```bash
sudo apt install -y libspdlog-dev
cd /home/rad907/workspace/ishimaru/dex1_1_service/build
cmake ..
make -j6
```

### 起動エラー: `Motors not found after multiple attempts`

グリッパーの電源が入っていません。電源接続を確認してください。

### 起動エラー: `No ttyUSB serial ports found`

グリッパーのシリアルボード（G1のUSBポートに接続するボード）が接続されていません。

### cmake が見つからない

```bash
sudo apt install -y cmake
```
