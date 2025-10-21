# kachaka_ROS2_package

 [スマートファニチャー・プラットホーム「カチャカ」](https://kachaka.life/)ROS2用パッケージ群
 
![Image](https://github.com/user-attachments/assets/ac880648-8ff6-4046-a70e-bb9f08267a01)

## 目次
<!-- TOC -->

- [概要](#概要)
- [はじめに](#はじめに)
- [開発環境](#開発環境)
- [ハードウェア](#ハードウェア)
- [インストール方法](#インストール方法)
- [使用方法](#使用方法)

<!-- /TOC -->

## 概要
業務向け自律搬送ロボットの [カチャカ](https://kachaka.life/)を用いたNavigation2(Nav2)での自律移動やコントローラでの操作などのをROS2で制御する為のリポジトリを提供します。

### パッケージ構成
|パッケージ名|説明|
|---|---|
|kachaka_description|カチャカの各部分のリンクやジョイント、又はセンサの情報を表示するパッケージ。[kachaka_description](https://github.com/pf-robotics/kachaka-api/tree/v3.8.5/ros2/kachaka_description)に基づいています|
|kachaka_interfaces|カチャカのアクション、メッセージファイルを管理するパッケージ。[kachaka_interfaces](https://github.com/pf-robotics/kachaka-api/tree/v3.8.5/ros2/kachaka_interfaces)に基づいています|
|kachaka_nav2_bringup|Navigation2(Nav2)における自律走行を行うためのパッケージ。[kachaka_nav2_bringup](https://github.com/pf-robotics/kachaka-api/tree/v3.8.5/ros2/demos/kachaka_nav2_bringup)に基づいています|
|kachaka_gazebo|Gazebo Ignitionによるカチャカのシミュレーション環境を提供するパッケージ|
|joy_controller|お持ちのコントローラーでカチャカを操作する為のパッケージ|

こちらのリポジトリ(kachaka_ros2)では、カチャカの実機やシミュレーション開発を用いてナビゲーション等を実行する機能を提供します。
シミュレーション環境（kachaka_gazeboパッケージ）を使ってナビゲーションを実行した例を下図に示します。


## はじめに

先ずは、ROS2でカチャカAPIを利用可能にする必要があります

### カチャカAPIの有効化
> [!POINT!]
> スマートフォンアプリ「Kachaka」を使ってカチャカAPIを有効にする必要があります。まだスマートフォンアプリ「Kachaka」をインストールしていない方はインストールしてください

* カチャカに接続し、[⚙設定]のタブから接続するロボットを選択、[カチャカAPI]ページを開いて「カチャカAPIを有効化する」をONにします。
* ダイアログが表示されるので、「利用規約」を確認の上、「カチャカAPI利用規約に同意する」をチェックして「設定する」を押して下さい。

### カチャカのIPアドレスの確認
* またいずれの場合にも、カチャカのIPアドレスが必要になります。
* [⚙設定] > [アプリ情報] から確認することができます。(以下のキャプチャは白塗りしてあります)
* また、mDNSによる名前解決に対応しており、同画面の「シリアル番号」からなる
    * `kachaka-<シリアル番号>.local`というホスト名でもアクセス可能です。

### kachaka-apiのクローン作成
```bash
cd
git clone https://github.com/pf-robotics/kachaka-api.git
```

### kachaka-apiによる接続

ここではDockerを用います

#### Dockerのインストール
* 以下を参考に、Dockerの設定を行って下さい。
    * https://docs.docker.com/engine/install/ubuntu/

#### ros2_bridgeの起動
* 以下のスクリプトを実行すると、ブリッジが実行されます。
* 初回実行時に、Dockerイメージがダウンロードされます。
    * ※ イメージの提供は予告なく停止される場合があります。

```bash
cd ~/kachaka-api/tools/ros2_bridge
./start_bridge.sh <カチャカのIPアドレス>
```

#### ⚠️カチャカのIPアドレスの確認
* [⚙設定] > [アプリ情報] から確認することができます。(以下のキャプチャは白塗りしてあります)
* また、mDNSによる名前解決に対応しており、同画面の「シリアル番号」からなる
    * `kachaka-<シリアル番号>.local`というホスト名でもアクセス可能です。
  
#### 動作を確認する

* トピックからメッセージを取得できるかどうか確認してみましょう。
* ブリッジのコンテナが立っているので、これを使ってトピックの確認などを行うことができます。

* トピック一覧の取得
```bash
docker exec -it ros2_bridge_ros2_bridge_1 /opt/kachaka/env.sh ros2 topic list
```

* 目的地一覧の取得

```bash
docker exec -it ros2_bridge_ros2_bridge_1 /opt/kachaka/env.sh ros2 topic echo /kachaka/layout/locations/list
```

* 以下のようなレスポンスが返ってきたら成功です。

```yaml
locations:
- id: L01
  name: ダイニング
  type: 0
  pose:
    x: 1.33572
    y: 2.328592
    theta: 0.0
```

#### ⚠️カチャカのアップデートの際の注意点
※kachakaをアップデートした際はカチャカのIPアドレスが変更されるために、新しいDocker環境が必要になるので、以下のコマンドを実行して既存のDocker環境を削除しなければならない
```bash
cd ~/kachaka-api/tools/ros2_bridge

# 停止中コンテナの削除（必要に応じて）
docker container prune -f

# 既存のros2_bridgeサービスの削除
docker-compose down --volumes --remove-orphans

# 古いイメージの削除（ピンポイントで削除するのが安全）
docker rmi asia-northeast1-docker.pkg.dev/kachaka-api/docker/kachaka-grpc-ros2-bridge:20250213
```
そして再びkachaka-apiでの接続を行う
```bash
cd ~/kachaka-api/tools/ros2_bridge
./start_bridge.sh <新しいkachakaのIPアドレス>
```


## インストール方法

リポジトリのclone
```bash
mkdir -p ~/your_ws/src
cd ~/your_ws/src
git clone https://github.com/ndlab-ros2/kachaka_ros2.git
cd ..
rosdep install -y -i --from-paths src
colcon build --packages-skip wiimote
source install/setup.bash
```
