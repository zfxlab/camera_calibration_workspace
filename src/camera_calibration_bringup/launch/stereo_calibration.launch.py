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


def _component_for(entry, defaults, config_directory):
    namespace = entry.get("namespace", "").strip("/")
    if not namespace:
        raise RuntimeError("Each stereo camera must have a non-empty namespace")
    parameters = {key: value for key, value in defaults.items() if key in _DRIVER_PARAMETERS}
    parameters.update({key: value for key, value in entry.items() if key in _DRIVER_PARAMETERS})
    parameters.setdefault("camera_name", namespace.replace("/", "_"))
    parameters.setdefault("frame_id", f"{namespace.replace('/', '_')}_optical_frame")
    parameters["output_encoding"] = "mono8"
    camera_info_url = parameters.get("camera_info_url", "")
    if not camera_info_url:
        raise RuntimeError(f"Camera {namespace!r} must configure camera_info_url")
    if "://" not in camera_info_url:
        info_path = Path(os.path.expandvars(os.path.expanduser(camera_info_url)))
        if not info_path.is_absolute():
            info_path = config_directory / info_path
        parameters["camera_info_url"] = info_path.resolve().as_uri()
    typed_parameters = {
        key: ParameterValue(value, value_type=str) if key in _STRING_PARAMETERS else value
        for key, value in parameters.items()
    }
    return ComposableNode(
        package="hik_camera_driver",
        plugin="hik_camera_driver::HikCameraNode",
        namespace=f"/{namespace}",
        name=entry.get("node_name", "driver"),
        parameters=[typed_parameters],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def _launch_setup(context):
    cameras_path = Path(os.path.expandvars(os.path.expanduser(
        LaunchConfiguration("cameras_file").perform(context)))).resolve()
    if not cameras_path.is_file():
        raise RuntimeError(f"Camera configuration does not exist: {cameras_path}")
    with cameras_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    defaults = document.get("defaults", {})
    cameras = document.get("cameras", [])
    if not isinstance(defaults, dict) or not isinstance(cameras, list):
        raise RuntimeError("cameras_file must contain a defaults mapping and cameras list")

    selected = []
    requested_namespaces = []
    for role in ("left_camera", "right_camera"):
        requested = LaunchConfiguration(role).perform(context).strip("/")
        requested_namespaces.append(requested)
        matches = [
            entry for entry in cameras
            if isinstance(entry, dict) and entry.get("enabled", True) and
            entry.get("namespace", "").strip("/") == requested
        ]
        if len(matches) != 1:
            raise RuntimeError(
                f"{role}={requested!r} must match exactly one enabled camera namespace; "
                f"found {len(matches)}"
            )
        selected.append(_component_for(matches[0], defaults, cameras_path.parent))

    if requested_namespaces[0] == requested_namespaces[1]:
        raise RuntimeError("left_camera and right_camera must select different namespaces")

    selected.append(ComposableNode(
        package="stereo_aruco_calibration",
        plugin="stereo_calibration::StereoArucoCalibratorNode",
        namespace="/stereo_calibration",
        name="calibrator",
        parameters=[
            LaunchConfiguration("calibration_file"),
            {
                "left.image_topic": f"/{requested_namespaces[0]}/image_raw",
                "left.camera_info_topic": f"/{requested_namespaces[0]}/camera_info",
                "right.image_topic": f"/{requested_namespaces[1]}/image_raw",
                "right.camera_info_topic": f"/{requested_namespaces[1]}/camera_info",
                "output_path": ParameterValue(
                    LaunchConfiguration("output_file"), value_type=str),
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    ))

    container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="stereo_calibration_pipeline",
        namespace="/",
        output="screen",
        emulate_tty=True,
        composable_node_descriptions=selected,
    )
    gui = Node(
        package="stereo_aruco_calibration",
        executable="stereo_calibration_gui",
        name="stereo_calibration_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    return [container, gui]


def generate_launch_description():
    share = get_package_share_directory("camera_calibration_bringup")
    return LaunchDescription([
        DeclareLaunchArgument(
            "cameras_file", default_value=os.path.join(share, "config", "cameras.yaml")),
        DeclareLaunchArgument(
            "calibration_file",
            default_value=os.path.join(share, "config", "stereo_calibration.yaml")),
        DeclareLaunchArgument("left_camera", default_value="left_camera"),
        DeclareLaunchArgument("right_camera", default_value="right_camera"),
        DeclareLaunchArgument("output_file", default_value="/tmp/stereo_extrinsics.yaml"),
        DeclareLaunchArgument("gui", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
