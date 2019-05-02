# tracker_ros

## Description
OpenCVの物体追跡のモジュールをROSで使いやすくしました。(opencv3.4以上推奨)

- [KCF(Kernelized Correlation Filters)](https://www.slideshare.net/hitoshinishimura75/kcf-75287109)
   - GOTURNに比べて圧倒的に速い（リアルタイムで追跡可能）
   - スケール変化に弱い

- [GOTURN(Generic Object Tracking Using Regression Networks)](https://qiita.com/nonbiri15/items/ca3d43acea0ccd43d412#goturn-tracker)  
  - ロバスト性は高い
  - 処理に時間がかかる（リアルタイムトラッキングは難しい）

## Demo  
![](gif/demo.gif)

## Setup
- **clone this repository & catkin_make**
  ```
  cd ~/catkin_ws/src

  git clone https://gitlab.com/TeamSOBITS/tracker_ros.git

  cd ..

  catkin_make
  ```

- **download caffe model**  
  Download [here](https://github.com/Mogball/goturn-files)
  and then...  
  ```
  sudo cp /.../PATH/TO/DOWNLOWDs/goturn.prototxt ~/.ros/

  sudo cp /.../PATH/TO/DOWNLOWDs/goturn.caffemodel ~/.ros/
  ```

## Run
実行すると、"DrawBoundinbBox"という名前のwindowが出ます。  
追跡したい範囲をドラッグで囲んでください。

```
roslaunch tracker_ros kcf_2d.launch #rgbカメラのみ
```
```
roslaunch tracker_ros kcf_3d.launch #対象物のtfが出る
```
```
roslaunch tracker_ros kcf_rviz.launch #debug用
```
```
roslaunch tracker_ros goturn_2d.launch #rgbカメラのみ
```
```
roslaunch tracker_ros goturn_3d.launch #対象物のtfが出る
```
```
roslaunch tracker_ros goturn_rviz.launch #debug用
```

## Subscribe
- /camera/rgb/image_raw [sensor_msgs:Image]
- /camera/depth/points [sensor_msgs:PoinCloud2]
- /tracker_ros/update_bbox [darknet_dnn:BoundingBox]

## Publish
- **kcf**
  - /tracker_ros/kcf/result_bbox [darknet_dnn:BoundingBox]
  - /tracker_ros/kcf/result_flag [std_msgs:Bool]
  - /tracker_ros/kcf/result_img [sensor_msgs:Image]

- **goturn**
  - /tracker_ros/goturn/result_bbox [darknet_dnn:BoundingBox]
  - /tracker_ros/goturn/result_flag [std_msgs:Bool]
  - /tracker_ros/goturn/result_img [sensor_msgs:Image]
