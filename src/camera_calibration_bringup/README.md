# Camera Calibration Bringup

项目级相机配置、单目标定和双目标定入口。相机序列号、namespace 和内参文件统一维护在
`config/cameras.yaml`，无需修改通用驱动包。

## 单目标定

```bash
ros2 launch camera_calibration_bringup mono_calibration.launch.py \
  camera:=left_camera board_size:=9x6 square_size:=0.025 pattern:=chessboard
```

`camera` 可以是启用相机的 namespace、序列号或唯一的 `camera_name`。launch 强制使用
`mono8`，并启动相机 component 与 ROS `cameracalibrator`。标定完成后点击 `COMMIT`，驱动会
把结果写入该相机的 `camera_info_url`；点击 `SAVE` 仍会生成标准的
`/tmp/calibrationdata.tar.gz` 归档。

## 双目标定

```bash
ros2 launch camera_calibration_bringup stereo_calibration.launch.py \
  output_file:=/tmp/stereo_extrinsics.yaml
```

左右两个 `HikCameraNode` 与 `StereoArucoCalibratorNode` 加载在同一个
`component_container_mt` 中并启用进程内通信。GUI 是独立的 Python 进程，可用
`gui:=false` 关闭。标定板、采样和显示参数位于 `config/stereo_calibration.yaml`。
