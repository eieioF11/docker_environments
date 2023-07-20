# RealSense

## Build

```
sudo ./build.sh
```

## Run

```
 sudo ./run.sh
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
