# docker_environments
各種docke環境保存用リポジトリ
## docker環境リスト
- ros_noetic
## Build

```bash
. build.sh
```

## run bush

```bash
docker-compose run bash
```
## トラブルシューティング
### rvizやrqtなどのGUIアプリが起動しないとき
以下のコマンドを実行してから起動する

```
xhost +
```