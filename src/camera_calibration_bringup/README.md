# Camera Calibration Bringup

项目级相机配置、单目标定和双目标定入口。相机序列号、namespace 和内参文件统一维护在
`config/cameras.yaml`，无需修改通用驱动包。

## 单目标定

```bash
ros2 launch camera_calibration_bringup mono_calibration.launch.py
```

`camera` 可以是启用相机的 namespace、序列号或唯一的 `camera_name`。launch 强制相机输出
`mono8`，并将 `HikCameraNode` 与 C++ `MonoCalibrationNode` 加载到同一个
`component_container_mt`。两个组件均启用进程内通信，原始图像不需要经过跨进程 DDS。

常用参数：

```bash
ros2 launch camera_calibration_bringup mono_calibration.launch.py \
  camera:=left_camera \
  board_size:=7x7 \
  square_size:=0.03 \
  pattern:=circles \
  output_file:=/tmp/left_camera.yaml
```

GUI 是独立 C++ 进程，只接收缩放后的 JPEG 预览和状态，不订阅原始图像。可通过
`gui:=false` 关闭。

GUI 按键：

- `G`：开始采样
- `X`：停止采样
- `R`：重置
- `C`：求解
- `S`：保存到 `output_file`
- `U`：提交到相机驱动的 `set_camera_info` 服务，并写入该相机配置的
  `camera_info_url`
- `Q` 或 `Esc`：退出 GUI

详细参数与服务说明见 `mono_camera_calibration/README.md`。

## 双目标定

```bash
ros2 launch camera_calibration_bringup stereo_calibration.launch.py \
  output_file:=/tmp/stereo_extrinsics.yaml
```

左右两个 `HikCameraNode` 与 `StereoArucoCalibratorNode` 加载在同一个
`component_container_mt` 中并启用进程内通信。GUI 是独立的 Python 进程，可用
`gui:=false` 关闭。标定板、采样和显示参数位于 `config/stereo_calibration.yaml`。
