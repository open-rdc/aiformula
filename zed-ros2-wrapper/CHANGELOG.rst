LATEST CHANGES
==============

2024-07-31
----------
- Add support for point cloud transport

2024-07-15
----------
- Fixed a bug while playing a ZED X stream on a "not-Jetson" host device

2024-06-24
----------
- Changed default DDS Middleware to FastDDS in Docker

2024-05-29
----------
- New Docker configuration files allow to easily create "ZED ROS2 Wrapper" images based on specific tag versions. [Read more](./docker/README.md)

2024-04-26 (ZED SDK v4.1)
-------------------------
- Updated the Docker files to the CUDA 12.4 (PC), L4T 35.4 (Jetson), SDK v4.1.0
- Added Local Streaming output
  - Added `enable_streaming` service to start/stop a streaming server
  - Added Streaming Server diagnostic
  - Added parameter 'stream_server.stream_enabled': enable the streaming server when the camera is open
  - Added parameter 'stream_server.codec': different encoding types for image streaming
  - Added parameter 'stream_server.port': Port used for streaming
  - Added parameter 'stream_server.bitrate': Streaming bitrate (in Kbits/s) used for streaming
  - Added parameter 'stream_server.gop_size': The GOP size determines the maximum distance between IDR/I-frames
  - Added parameter 'stream_server.adaptative_bitrate': Bitrate will be adjusted depending on the number of packets dropped during streaming
  - Added parameter 'stream_server.chunk_size': Stream buffers are divided into X number of chunks where each chunk is chunk_size bytes long
  - Added parameter 'stream_server.target_framerate': Framerate for the streaming output
- Added Local Streaming input
  - Added 'stream.stream_address' and 'stream.stream_port' parameter to configure the local streaming input
- GNSS Fusion temporarily disabled *(available with 4.1.1)*
- Moved parameter 'general.svo_file' to 'svo.svo_path'
- Moved parameter 'general.svo_loop' to 'svo.svo_loop'
- Moved parameter 'general.svo_realtime' to 'svo.svo_realtime'
- Removed obsolete launch files: 'zed.launch.pi','zed2.launch.pi', 'zed2i.launch.pi', 'zedm.launch.pi', 'zedx.launch.pi', 'zedxm.launch.pi'
- Removed obsolete display launch file: 'display_zed.launch.py', 'display_zed2.launch.py', 'display_zed2i.launch.py', 'display_zedm.launch.py', 'display_zedx.launch.py', 'display_zedxm.launch.py'
- Added support for custom virtual stereo cameras made with two calibrated ZED X One cameras *(available with 4.1.1)*
- Added parameter `pos_tracking.reset_odom_with_loop_closure` to automatically reset odometry when a loop closure is detected
- Added new positional tracking information to the `PosTrackStatus` message
- Added new `GnssFusionStatus` message with GNSS Fusion status information *(available with 4.1.1)*
- Added new parameters `gnss_fusion.h_covariance_mul` and `gnss_fusion.v_covariance_mul` to control the effects of the GNSS covariance
- Added support to Automatic ROI
  - Added ROI diagnostic
  - Added parameter `debug.debug_roi`
  - Publish ROI mask image on the topic `~/roi_mask` using image transport
  - Moved the parameter `general.region_of_interest` to `region_of_interest.manual_polygon`
  - Added automatic Region of Interest support
  - Added parameter `region_of_interest.automatic_roi`
  - Added parameter `region_of_interest.depth_far_threshold_meters`
  - Added parameter `region_of_interest.image_height_ratio_cutoff`
  - Added parameter `region_of_interest.apply_to_depth`
  - Added parameter `region_of_interest.apply_to_positional_tracking`
  - Added parameter `region_of_interest.apply_to_object_detection`
  - Added parameter `region_of_interest.apply_to_body_tracking`
  - Added parameter `region_of_interest.apply_to_spatial_mapping`
- Removed QoS parameters to use ROS 2 QoS overwrite -> https://design.ros2.org/articles/qos_configurability.html
- Added support for new `NEURAL_PLUS` depth mode
- Added new `<camera_name>_gnss_link` frame to URDF to set the position of the GNSS antenna with respect to the camera position

v4.0.8
------
- The parameter `general.sdk_verbose` has been moved to `debug.sdk_verbose` and set to `0` as default.
- Added new parameter `general.optional_opencv_calibration_file` to use custom OpenCV camera calibrations.
- Added [new tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_robot_integration) to illustrate how to integrate one or more ZED cameras on a robot
- Added 'simulation.sim_enabled' parameter to enable the simulation mode
- Added 'simulation.sim_address' parameter to set the simulation server address
- Added 'simulation.sim_port' parameter to set the simulation server port
- Added `/clock` subscriber to check the presence of the required message when `use_sim_time` is true
- Force `grab_frame_rate` and `pub_frame_rate` to 60 Hz in simulation
- Force `grab_resolution` to `HD1080` in simulation
- Removed the `general.zed_id` parameter. Always use `general.serial_number` to distinguish between different cameras in a multi-camera configuration.
- The multi-camera example has been updated to match the new TF configuration
- The old launch files are now obsolete: 'ros2 launch zed_wrapper <camera_model>.launch.py' is replaced by 'ros2 
  launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>'
- The reference link for positional tracking is no longer 'base_link' but `<camera_name>_camera_link`. 
  This will allow an easier ZED integration in existing robot configuration because the transform `base_link` -> `camera_link` 
  is no longer published by the ZED ROS2 Wrapper. Thanks to @SteveMacenski for the advice
  - Removed `parent` and `origin` parameters from `zed_macro.urdf.xacro`
  - Removed launch argument `cam_pose` from `zed_camera.launch.py`
- Moved parameter `publish_imu_tf` from `pos_tracking` to `sensors` to make it available also in "no depth" configurations of the node
- Added new parameter `pos_tracking.pos_tracking_mode` to exploit the new ZED SDK `QUALITY` mode for improved odometry and localization
- New Video/Depth processing throttling method by using the `grab_compute_capping_fps` ZED SDK parameter instead of a dedicated thread
- Advanced parameters to handle Thread scheduling policy and priorities (sudo required):`thread_sched_policy`,`thread_grab_priority`,
  `thread_sensor_priority`,`thread_pointcloud_priority`
- Added new GNSS calibration parameters: `enable_reinitialization`, `enable_rolling_calibration`, `enable_translation_uncertainty_target`, `gnss_vio_reinit_threshold`, `target_translation_uncertainty`, `target_yaw_uncertainty`
- Added new Plane Detection parameters: `pd_max_distance_threshold`, `pd_normal_similarity_threshold`

v4.0.5
----------
- The parameter `general.pub_resolution` can now take only `NATIVE` and `CUSTOM` values. 'NATIVE' to use the same `general.grab_resolution` - `CUSTOM` to apply the `general.pub_downscale_factor` downscale factory to reduce bandwidth in transmission
- Added new parameter `general.pub_downscale_factor` to be used with the new option `CUSTOM` for the parameter `general.pub_resolution`
- `ULTRA` is the new default value for `depth.depth_mode` (better performance for odometry and positional tracking)
- Added resolution `HD1080` for ZED X
- Fix issue with Body Tracking start/stop by service call. Now Body Tracking can be restarted multiple times
- Fix depth grab performance by removing a [not required `PNG Write` call](https://github.com/stereolabs/zed-ros2-wrapper/pull/164). Thank you Esteban Zamora @ezamoraa 
- Fix bug with `general.pub_resolution` value, not allowing to select the correct data publish resolution
- Added new launch parameter `ros_params_override_path` to provide the path to a custom YAML file to override the parameters of the ZED Node without modifying the original files in the `zed_wrapper/config` folder. Thank you David Lu @MetroRobots

v4.0.0
------
- Added support for ZED-X and ZED-X Mini

  - Moved `general.grab_resolution` and `general.grab_frame_rate` to the yaml file specific for the relative camera model (i.e. `zed.yaml`, `zedm.yaml`, `zed2.yaml`, `zed2i.yaml`, `zedx.yaml`, `zedxm.yaml`)
  - Added `zedx.launch.py` for ZED-X
  - Added `zedxm.launch.py` for ZED-X Mini
  - Improve `zed_macro.urdf.xacro` with specific configuration for the new camera models
  - Added `display_zedx.launch.py` for ZED-X to ZED-ROS2-Examples
  - Added `display_zedxm.launch.py` for ZED-X Mini to ZED-ROS2-Examples
  - Added ZED-X and ZED-X Mini STL files to ZED-ROS2-Interfaces

- Positional Tracking

  - Added `pos_tracking.set_as_static` parameters for applications with a static camera monitoring a robotics environment. See [PR #122](https://github.com/stereolabs/zed-ros2-wrapper/pull/122 ) Thx @gabor-kovacs
  - Added custom message type `PosTrackStatus`
  - Publish message on topic `~/pose/status` with the current status of the pose from the ZED SDK
  - Publish message on topic `~/odom/status` with the current status of the odometry from the ZED SDK

- Body Tracking

  - Added Support for the new Body Tracking module
  - Added parameter `body_tracking.bt_enabled` to enable Body Tracking
  - Added parameter `body_tracking.model` to set the AI model to be used
  - Added parameter `body_tracking.body_format` to set the Body Format to be used
  - Added parameter `body_tracking.allow_reduced_precision_inference` to improve performances
  - Added parameter `body_tracking.max_range` to set the max range for Body Detection
  - Added parameter `body_tracking.body_kp_selection` to choose the Body key points to be used
  - Added parameter `body_tracking.enable_body_fitting` to enable body fitting
  - Added parameter `body_tracking.enable_tracking` to enable the tracking of the detected bodies
  - Added parameter `body_tracking.prediction_timeout_s` to set the timeout of the prediction phase while tracking
  - Added parameter `body_tracking.confidence_threshold` to set the detection confidence threshold
  - Added parameter `body_tracking.minimum_keypoints_threshold` to set the minimum number of detected key points to consider a body valid
  - Publish new message on topic `~/body_trk/skeletons`
  - Added service `enable_body_trk` to start/stop body tracking

- GNSS fusion integration

  - New param `gnss_fusion.gnss_fusion_enabled` to enable GNSS fusion
  - New param `gnss_fusion.gnss_fix_topic` name of the topic containing GNSS Fix data of type `sensor_msgs/NavSatFix`
  - Added `nmea_msgs` dependency
  - Added GNSS Fix Diagnostic
  - Added new launch parameter `gnss_frame` to enable the GNSS link in the ZED URDF
  - Added new node parameter `gnss_fusion.gnss_zero_altitude` to ignore GNSS altitude information
  - Added new node parameter `gnss_fusion.gnss_frame` to set the name of the frame link of the GNSS sensor
  - Disable Area Memory (loop closure) when GNSS fusion is enabled
  - Added services `toLL` and `fromLL` to use the ZED ROS2 Wrapper with the Nav2 Waypoint Navigation package
  - Added `geographic_msgs::msg::GeoPoseStamped` message publisher
  - Added parameter `gnss_fusion.publish_utm_tf`
  - Added parameter `gnss_fusion.broadcast_utm_transform_as_parent_frame`
  - Added parameter `gnss_fusion.gnss_init_distance`
  - Publish message on topic `~/geo_pose/status` with the current status of the GeoPose from the ZED SDK
  - Publish message on topic `~/pose/filtered` with the current GNSS filtered pose in `map` frame
  - Publish message on topic `~/pose/filtered/status` with the current status of the GNSS filtered pose from the ZED SDK

- Object Detection

  - Added `object_detection.allow_reduced_precision_inference` to allow inference to run at a lower precision to improve runtime and memory usage
  - Added `object_detection.max_range` to defines a upper depth range for detections
  - Removed `object_detection.body_format`

- Docker

  - Added Docker files (see `docker` folder) ready to create Docker images for desktop host devices

- Examples/Tutorials

  - Added multi-camera example in `zed-ros2-examples` repository.

- Added full Terrain Mapping (local obstacle detection) support [EXPERIMENTAL FEATURE AVAILABLE ONLY FOR BETA TESTERS]

  - ZED SDK Terrain Mapping published as GridMap message
  - Added parameter `local_mapping.terrain_mapping_enabled` to enable terrain mapping publishing a local obstacle map
  - Added parameter `local_mapping.data_pub_rate` to set the Local Map data publish frequency
  - Added parameter `local_mapping.grid_resolution` to set the Local Map resolution in meters [min: 0.01 - max: 1.0]
  - Added parameter `local_mapping.grid_range` to set the maximum depth range for local map generation [min: 1.0 - max: 8.0]
  - Added parameter `local_mapping.height_threshold` to set the maximum height for obstacles
  - Publish gridmap on topic `local_map/gridmap`
  - Publish elevation map image on topic `local_map/elev_img`
  - Publish obstacle color map image on topic `local_map/col_img`
  - Added traversability cost computation for Terrain Mapping (local_mapping)

    - Change parameter `local_mapping.height_threshold` to `local_mapping.robot_heigth`
    - Added parameter `local_mapping.robot_radius` to set radius of the robot
    - Added parameter `local_mapping.robot_max_step` to set max height of a step that the robot can overcome
    - Added parameter `local_mapping.robot_max_slope` to set max slope (degrees) that the robot can overcome
    - Added parameter `local_mapping.robot_max_roughness` to set max roughness of the terrain that the robot can overcome

- Added support for simulated data [EXPERIMENTAL FEATURE AVAILABLE ONLY FOR BETA TESTERS]

  - Added parameter `use_sim_time` to enable SIMULATION mode
  - Added parameter `sim_address` tos set the local address of the machine running the simulator
  - Change StopWatch to use ROS clock instead of System Clock. In this way diagnostic and time checking work also in simulation
  - Disable camera settings control in simulation

- Others

  - Removed `sensing_mode`, no more available in SDK v4.0
  - Removed `extrinsic_in_camera_frame`, no more available in SDK v4.0
  - Added `zed_id` and `serial_number` launch parameters to open the correct camera in multi-camera configurations.
  - Code lint and re-formatting according to [ROS2 code rules](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
  - Added support for automatic lint tools to all the packages.
  - Removed node parameter `general.resolution`, replaced by `general.grab_resolution`.
  - Added node parameter `general.pub_resolution` used to reduce node computation and message bandwidth.

    - Available output resolutions: `HD2K`, `HD1080`, `HD720`, `MEDIUM`, `VGA`. `MEDIUM` is an optimized output resolution to maximize throughput and minimize processing costs.
  
  - Removed node parameters `video.img_downsample_factor` and `depth.depth_downsample_factor`. Use the new parameter `general.pub_resolution` instead.
  - Change `general.grab_resolution` and `general.pub_resolution` from integer to string.
  - Added new `LOW` value for `general.pub_resolution` (half the `MEDIUM` output resolution).
  - Removed `depth.quality` parameter (replaced with `depth.depth_mode`)
  - Added `depth.depth_mode` parameter: a string reflecting the ZED SDK `DEPTH_MODE` available value names
  - The parameter `depth.depth_stabilization` is now an integer in [0,100] reflecting ZED SDK behavior
  - Fix distortion model (see Issue [#128](https://github.com/stereolabs/zed-ros2-wrapper/issues/128))
  - Improve the code for Moving Average calculation for better node diagnostics.
  - Temperature diagnostic is now always updated even if `sensors.sensors_image_sync` is true and no image topics are subscribed.
  - Improve Grab thread and Video/Depth publishing thread elaboration time diagnostic.
  - Added a check on timestamp to not publish already published point cloud messages in the point cloud thread
  - Improve thread synchronization when the frequency of the `grab` SDK function is minor of the expected camera frame rate setting because of a leaking of elaboration power.
  - Added diagnostic warning if the frequency of the camera grabbing thread is minor than the selected `general.grab_frame_rate` value.
  - Removed annoying build log messages. Only warning regarding unsupported ROS2 distributions will be displayed when required.
  - Convert `shared_ptr` to `unique_ptr` for IPC support
  - Improve the `zed_camera.launch.py`

    - Added support for `OpaqueFunction` in order to automatically configure the launch file according to the value of the launch parameter `cam_model`.
    - Change parameters to set camera pose in launch files. From 6 separated parameters (`cam_pos_x`,`cam_pos_y`,`cam_pos_z`,`cam_roll`,`cam_pitch`,`cam_yaw`) to one single array (`cam_pose`).
    - Removed the workaround for empty `svo_path` launch parameter values thanks to `TextSubstitution`.
    - Modify the "display" launch files in [zed-ros2-examples](https://github.com/stereolabs/zed-ros2-examples) to match the new configuration.
    - Added `publish_tf` and `publish_map_tf` launch parameters useful for multi-camera configuretion or external odometry fusion.
  
  - Change LICENSE to Apache 2.0 to match ROS2 license.

v3.8.x
------
- Fixed `set_pose` wrong behavior. Now initial odometry is coherent with the new starting point.
- Added Plane Detection.
- Fixed "NO DEPTH" mode. By setting `depth/quality` to `0` now the depth extraction and all the sub-modules depending on it are correctly disabled.
- Added `debug` sub-set of parameters with new parameters `debug_mode` and `debug_sensors`.
- Added support for ROS2 Humble. Thx @nakai-omer.
  The two ROS2 LTS releases are now supported simoultaneously.
- Set `read_only` flag in parameter descriptors for non-dynamic parameters. Thx @bjsowa.
- Enabled Intra Process Communication. The ZED node no longer publishes topics with `TRANSIENT LOCAL` durability.
- Improved TF broadcasting at grabbing frequency
- Improved IMU/Left Camera TF broadcasting at IMU frequency
- Fixed data grabbing frame rate when publishing is set to a lower value
- Added TF broadcasting diagnostic
- The parameter `general.sdk_verbose` is now an integer accepting different SDK verbose levels.
- Moved Object Detection parameters from cameras configuration files to `common.yaml`
- Moved Sensor Parameters from cameras configuration files to `common.yaml`
- New data thread configuration to maximize data publishing frequency
  - Sensor data publishing moved from timer to thread
  - RGB/Depth data publishing moved from timer to thread
- Fixed random errors when closing the node
- Fixed wrong timing when playing SVO in `real-time` mode
- Fixed units for atmospheric pressure data. Now pressure is published in `Pascals` according to the [definition of the topic](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/FluidPressure.msg).
- Added new parameter `pos_tracking.transform_time_offset` to fix odometry TF timestamp issues
- Added new parameter `pos_tracking.depth_min_range` for removing fixed zones of the robot in the FoV of the camerafrom the visual odometry evaluation
- Added new parameter `pos_tracking.sensor_world` to define the world type that the SDK can use to initialize the Positionnal Tracking module
- Added new parameter `object_detection.prediction_timeout` for setting the timeout time [sec] of object prediction when not detected.
- Added support for ZED SDK Regiorn of Interest:
  - Added parameter `general.region_of_interest` to set the region of interest for SDK processing.
  - Added the service `resetRoi` to reset the region of interest.
  - Added the service `setRoi` to set a new region of interest.

v3.7.x
----------
- Added support for sport-related OD objects
- Added `remove_saturated_areas` dynamic parameter to disable depth filtering when luminance >=255
- Added `sl::ObjectDetectionParameters::filtering_mode` parameter
- Publish `depth_info` topic with current min/max depth information
- Fix parameter override problem (Issue #71). Thx @kevinanschau
- Added default xacro path value in `zed_camera.launch.py`. Thx @sttobia
- Fix `zed-ros2-interfaces` sub-module url, changing from `ssh` to `https`.

v3.6.x (2021-12-03)
-------------------
- Moved the `zed_interfaces` package to the `zed-ros2-interfaces` repository to match the same configuration of the ROS1 wrapper
- The `zed-ros2-interfaces` repository has been added as a sub-module to this repository
- Added new <zed>_base_link frame on the base of the camera to easily handle camera positioning on robots. Thx @civerachb-cpr
- Improve URDF by adding 3° slope for ZED and ZED2, X-offset for optical frames to correctly match the CMOS sensors position on the PCB, X-offset for mounting screw on ZED2i
- Added `zed_macro.urdf.xacro` to be included by other xacro file to easily integrate ZED cameras in the robot descriptions. See ROS1 PR [#771](https://github.com/stereolabs/zed-ros-wrapper/pull/771) for details. Thx @civerachb-cpr
- Fix URDF `height` value for ZED, ZED2 and ZED2i
- Fix performances drop on slower platforms. Thx @roncapat
- Fix SVO LOOP wrong behavior. Thx @kevinanschau
- Added xacro support for automatic URDF configuration
- Reworked launch files to support xacro and launch parameters
    - Use `ros2 launch zed_wrapper <launch_file> -s` to retrieve all the available parameters
- Added `svo_path:=<full path to SVO file>` as input for all the launch files to start the node using an SVO as input without modifying 'common.yaml`
- Improved diagnostic information adding elaboration time on all the main tasks
- Improved diagnostic time and frequencies calculation
- Added StopWatch to sl_tools
- Enabled Diagnostic status publishing
- Changed the default values of the QoS parameter reliability for all topics from BEST_EFFORT to RELIABLE to guarantee compatibility with all ROS2 tools
- Fixed tab error in `zedm.yaml`
- Fixed compatibility issue with ZED SDK older than v3.5 - Thanks @PhilippPolterauer
- Migration to ROS2 Foxy Fitzroy

v3.5.x (2021-07-05)
-------------------
- Added support for SDK v3.5
- Added support for the new ZED 2i
- Added new parameter `pos_tracking/pos_tracking_enabled` to enable positional tracking from start even if not required by any subscribed topic. This is useful, for example, to keep the TF always updated.
- Added support for new AI models: `MULTI_CLASS_BOX_MEDIUM` and `HUMAN_BODY_MEDIUM`
- Depth advertising is disabled when depth is disabled (see `sl::DETH_MODE::NONE`)
