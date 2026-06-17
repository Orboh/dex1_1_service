# drop_to_basket

G1の右手で収穫したオクラを、左手のかごに入れるプログラム。
MuJoCoシミュレーション上で動作確認できる。

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

## ファイル構成

| ファイル | 説明 |
|---|---|
| `drop_to_basket_mujoco.py` | MuJoCoシミュレーションのメインスクリプト |
| `arm_interpolator.py` | 関節角度の線形補間ユーティリティ |
| `g1_read_joints.py` | 関節角度の読み取り |
| `g1_hold_right_arm_current.py` | 右腕の現在姿勢保持 |

## 動作概要

1. 左手を「かごを持つ姿勢」にIKで配置
2. 右手をランダムな収穫位置に配置（毎回異なる位置を模擬）
3. 右手を握った状態で待機
4. 右手をかご上空へ補間移動（速度制限付き）
5. 到達後、右手を開いてオクラをリリース
