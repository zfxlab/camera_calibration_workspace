import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
            f"serial_number, or camera_name in {path}; found {len(matches)}"
        )

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


def _launch_setup(context):
    namespace, node_name, parameters = _load_camera(
        LaunchConfiguration("cameras_file").perform(context),
        LaunchConfiguration("camera").perform(context),
    )
    component_parameters = {
        key: ParameterValue(value, value_type=str) if key in _STRING_PARAMETERS else value
        for key, value in parameters.items()
    }
    camera_name = parameters["camera_name"]

    container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="mono_calibration_pipeline",
        namespace="/",
        output="screen",
        emulate_tty=True,
        composable_node_descriptions=[ComposableNode(
            package="hik_camera_driver",
            plugin="hik_camera_driver::HikCameraNode",
            namespace=f"/{namespace}",
            name=node_name,
            parameters=[component_parameters],
            extra_arguments=[{"use_intra_process_comms": True}],
        )],
    )
    calibrator = Node(
        package="camera_calibration",
        executable="cameracalibrator",
        name="monocular_calibrator",
        output="screen",
        arguments=[
            "--size", LaunchConfiguration("board_size"),
            "--square", LaunchConfiguration("square_size"),
            "--pattern", LaunchConfiguration("pattern"),
            "--camera_name", camera_name,
        ],
        remappings=[
            ("image", f"/{namespace}/image_raw"),
            ("camera", f"/{namespace}"),
        ],
    )
    return [container, calibrator]


def generate_launch_description():
    share = get_package_share_directory("camera_calibration_bringup")
    return LaunchDescription([
        DeclareLaunchArgument(
            "cameras_file", default_value=os.path.join(share, "config", "cameras.yaml")),
        DeclareLaunchArgument(
            "camera", default_value="camera",
            description="Enabled camera namespace, serial number, or unique camera_name"),
        DeclareLaunchArgument("board_size", default_value="7x7"),
        DeclareLaunchArgument("square_size", default_value="0.03"),
        DeclareLaunchArgument(
            "pattern", default_value="circles",
            description="chessboard, circles, acircles, or charuco"),
        OpaqueFunction(function=_launch_setup),
    ])
