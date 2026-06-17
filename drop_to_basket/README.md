# drop_to_basket

G1の右手で収穫したオクラを、左手のかごに入れるプログラム。
まずMuJoCoシミュレーション上で動作確認し、その後に実機（DDS）で動かす構成。

## セットアップ

### 1. Python依存パッケージ

```bash
pip install mujoco numpy
```

### 2. MuJoCoモデルの準備

G1のMuJoCoモデルはUnitree公式リポジトリから取得する。

```bash
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```

## 実行

```bash
cd drop_to_basket

# ビューワー付きで実行（3D表示で動作確認）
python3 drop_to_basket_mujoco.py \
  --scene /path/to/unitree_mujoco/unitree_robots/g1/scene_29dof_with_hand.xml

# ビューワーなしで実行（動作ログのみ）
python3 drop_to_basket_mujoco.py \
  --scene /path/to/unitree_mujoco/unitree_robots/g1/scene_29dof_with_hand.xml \
  --no-viewer

# ランダムシードを指定（再現性のある開始姿勢）
python3 drop_to_basket_mujoco.py \
  --scene /path/to/unitree_mujoco/unitree_robots/g1/scene_29dof_with_hand.xml \
  --seed 42
```

### オプション

| オプション | 説明 |
|---|---|
| `--scene` | MuJoCoシーンXMLのパス（必須） |
| `--seed` | ランダムシード（右腕の開始姿勢の再現用） |
| `--no-viewer` | ビューワーを開かずに実行 |
| `--free-base` | ベースを固定しない（バランス制御なしだと倒れる） |
| `--enable-gravity` | 重力を有効化（デフォルトは無効） |
| `--dynamic` | PD制御でシミュレーション（デフォルトはキネマティック再生） |

## 投入姿勢の書き出し（MuJoCo → 実機の橋渡し）

MuJoCoのIKで求めた投入姿勢を、実機スクリプトが読むJSONに書き出す。

```bash
python3 export_mujoco_drop_pose.py \
  --scene /path/to/unitree_mujoco/unitree_robots/g1/scene_29dof_with_hand.xml
# → drop_to_basket/data/mujoco_right_arm_drop_pose.json が生成される
```

> このJSONは生成物（`.gitignore` 済み）。実機スクリプトを動かす前に一度実行して生成する。
> 投入姿勢の値は現状MuJoCoのIK仮値。最終的には実機計測値へ差し替える。

## 実機（DDS）での実行

実機スクリプトは `unitree_sdk2py` を使う。SDKのパスは環境変数 `UNITREE_SDK2_PYTHON_PATH` で指定する（未指定時は `/home/techshare/ILkit/unitree_sdk2_python`）。

```bash
export UNITREE_SDK2_PYTHON_PATH=/path/to/unitree_sdk2_python

# 投入姿勢JSONへ右腕を移動
python3 g1_move_right_arm_toward_pose.py
```

> ⚠️ 実機スクリプトは実機で動作させるもの。安全を確認してから実行すること。

## ファイル構成

### MuJoCoシミュレーション

| ファイル | 説明 |
|---|---|
| `drop_to_basket_mujoco.py` | MuJoCoシミュレーションのメインスクリプト |
| `arm_interpolator.py` | 関節角度の線形補間ユーティリティ（MuJoCo/実機共用） |
| `export_mujoco_drop_pose.py` | MuJoCoの投入姿勢をJSONへ書き出す |

### 実機（DDS）

| ファイル | 説明 |
|---|---|
| `g1_read_joints.py` | 関節角度の読み取り |
| `g1_hold_right_arm_current.py` | 右腕の現在姿勢保持 |
| `g1_move_one_right_arm_joint.py` | 右腕の1関節を動かす（動作確認用） |
| `g1_move_right_arm_small_pose.py` | 右腕を小さく動かす（動作確認用） |
| `g1_move_right_arm_toward_pose.py` | 右腕を投入姿勢JSONへ動かす |

## 動作概要（MuJoCoデモ）

1. 左手を「かごを持つ姿勢」にIKで配置
2. 右手をランダムな収穫位置に配置（毎回異なる位置を模擬）
3. 右手を握った状態で待機
4. 右手をかご上空へ補間移動（速度制限付き）
5. 到達後、右手を開いてオクラをリリース
