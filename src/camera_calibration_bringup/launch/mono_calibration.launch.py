import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


_STRING_PARAMETERS = {
    "transport", "serial_number", "user_defined_name", "camera_name",
    "camera_info_url", "frame_id", "output_encoding", "trigger_mode",
}
_DRIVER_PARAMETERS = _STRING_PARAMETERS | {
    "use_sensor_data_qos", "grab_timeout_ms", "reconnect_interval_ms",
    "max_consecutive_timeouts", "sdk_buffer_count", "auto_exposure",
    "exposure_time", "auto_gain", "gain", "frame_rate",
}


def _load_camera(cameras_file, requested_camera):
    path = Path(os.path.expandvars(os.path.expanduser(cameras_file))).resolve()
    if not path.is_file():
        raise RuntimeError(f"Camera configuration does not exist: {path}")
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    defaults = document.get("defaults", {})
    cameras = document.get("cameras", [])
    if not isinstance(defaults, dict) or not isinstance(cameras, list):
        raise RuntimeError("cameras_file must contain a defaults mapping and cameras list")

    matches = [
        item for item in cameras
        if isinstance(item, dict) and item.get("enabled", True) and
        requested_camera in {
            item.get("namespace", "").strip("/"),
            str(item.get("serial_number", "")),
            item.get("camera_name", ""),
        }
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"camera={requested_camera!r} must match exactly one enabled namespace, "
            f"serial_number, or camera_name in {path}; found {len(matches)}")

    entry = matches[0]
    namespace = entry.get("namespace", "").strip("/")
    if not namespace:
        raise RuntimeError("The selected camera must have a non-empty namespace")
    parameters = {key: value for key, value in defaults.items() if key in _DRIVER_PARAMETERS}
    parameters.update({key: value for key, value in entry.items() if key in _DRIVER_PARAMETERS})
    parameters.setdefault("camera_name", namespace.replace("/", "_"))
    parameters.setdefault("frame_id", f"{namespace.replace('/', '_')}_optical_frame")
    parameters["output_encoding"] = "mono8"

    camera_info_url = parameters.get("camera_info_url", "")
    if not camera_info_url:
        raise RuntimeError("The selected camera must configure camera_info_url for COMMIT")
    if "://" not in camera_info_url:
        info_path = Path(os.path.expandvars(os.path.expanduser(camera_info_url)))
        if not info_path.is_absolute():
            info_path = path.parent / info_path
        parameters["camera_info_url"] = info_path.resolve().as_uri()
    return namespace, entry.get("node_name", "driver"), parameters


def _parse_board_size(value):
    parts = value.lower().split("x")
    if len(parts) != 2:
        raise RuntimeError("board_size must use COLUMNSxROWS format, for example 7x7")
    try:
        columns, rows = (int(part) for part in parts)
    except ValueError as error:
        raise RuntimeError("board_size must contain integer columns and rows") from error
    if columns < 2 or rows < 2:
        raise RuntimeError("board_size columns and rows must both be at least 2")
    return columns, rows


def _launch_setup(context):
    namespace, node_name, parameters = _load_camera(
        LaunchConfiguration("cameras_file").perform(context),
        LaunchConfiguration("camera").perform(context))
    columns, rows = _parse_board_size(
        LaunchConfiguration("board_size").perform(context))
    driver_parameters = {
        key: ParameterValue(value, value_type=str) if key in _STRING_PARAMETERS else value
        for key, value in parameters.items()
    }

    camera = ComposableNode(
        package="hik_camera_driver",
        plugin="hik_camera_driver::HikCameraNode",
        namespace=f"/{namespace}",
        name=node_name,
        parameters=[driver_parameters],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    calibrator = ComposableNode(
        package="mono_camera_calibration",
        plugin="mono_camera_calibration::MonoCalibrationNode",
        namespace="/mono_calibration",
        name="calibrator",
        parameters=[
            LaunchConfiguration("calibration_file"),
            {
                "image_topic": f"/{namespace}/image_raw",
                "set_camera_info_service": f"/{namespace}/set_camera_info",
                "camera_name": ParameterValue(parameters["camera_name"], value_type=str),
                "board.pattern": ParameterValue(
                    LaunchConfiguration("pattern"), value_type=str),
                "board.columns": columns,
                "board.rows": rows,
                "board.square_size_m": ParameterValue(
                    LaunchConfiguration("square_size"), value_type=float),
                "output_path": ParameterValue(
                    LaunchConfiguration("output_file"), value_type=str),
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="mono_calibration_pipeline",
        namespace="/",
        output="screen",
        emulate_tty=True,
        composable_node_descriptions=[camera, calibrator],
    )
    gui = Node(
        package="mono_camera_calibration",
        executable="mono_calibration_gui",
        name="mono_calibration_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    return [container, gui]


def generate_launch_description():
    bringup_share = get_package_share_directory("camera_calibration_bringup")
    return LaunchDescription([
        DeclareLaunchArgument(
            "cameras_file",
            default_value=os.path.join(bringup_share, "config", "cameras.yaml")),
        DeclareLaunchArgument(
            "calibration_file",
            default_value=os.path.join(
                bringup_share, "config", "mono_calibration.yaml")),
        DeclareLaunchArgument(
            "camera", default_value="left_camera",
            description="Enabled camera namespace, serial number, or unique camera_name"),
        DeclareLaunchArgument("board_size", default_value="7x7"),
        DeclareLaunchArgument("square_size", default_value="0.03"),
        DeclareLaunchArgument(
            "pattern", default_value="circles",
            description="chessboard, circles, or acircles"),
        DeclareLaunchArgument("output_file", default_value="/tmp/mono_camera.yaml"),
        DeclareLaunchArgument("gui", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
