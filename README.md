# RealSense

## Build

```
sudo ./build.sh
```

## Run

```
 sudo ./run.sh
```
gpuあり
```
 sudo ./run.sh -g
```
### realsense-viewer

```
realsense-viewer
```
### ros

```
roslaunch realsense2_camera rs_camera.launch
```

pointcloud2出力
```
roslaunch realsense2_camera rs_rgbd.launch align_depth:=true
```
# docker-compose

## build
```
docker-compose build
```
## run
```
docker-compose run bash
```

```
docker-compose run master
```

```
docker-compose run rviz
```

```
docker-compose run rs-rgbd
```

```
docker-compose run realsense-viewer
```
