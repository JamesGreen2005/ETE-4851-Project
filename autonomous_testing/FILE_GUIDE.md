# File guide

Paths are relative to `autonomous_testing/`. Every bundled file is listed below. Runtime source is under `outputs/slam_explorer/`; historical release ZIPs are alternatives, not extra programs to run.

| File | Purpose |
|---|---|
| `.gitignore` | Excludes caches, logs, local environments and credential files; allows the requested FW7 reference binary. |
| `calibration/README.txt` | Calibration snapshot provenance and instructions to preserve/recheck actual on-device calibration. |
| `calibration/robot_nav_calibration.last_known_2026-11-06.json` | Last-known actual Pi calibration snapshot from November 6; not fetched live today and not automatically installed. |
| `diagnostics/capture_stuck_map.py` | Captures the current raw map and pose for replay; writes its historical /tmp/stuck_map.json path. |
| `diagnostics/dashboard_015_smoke.py` | Read-only dashboard constructor/live LiDAR check using the robot calibration in a local TF buffer. |
| `diagnostics/footprint_017_probe.py` | Stationary candidate/path-service probe. Sends ComputePathToPose requests only, never NavigateToPose or drive-enable. |
| `diagnostics/reset_014_live_probe.py` | Authenticated dashboard HTTP Stop/Reset checks; intended only with allow_motion=false. Uses its historical /tmp log path. |
| `diagnostics/route_diagnostic.py` | Read-only map/pose/drive/exploration and candidate-planning diagnostic snapshot. |
| `diagnostics/stationary_013_probe.py` | Read-only ROS topic/status probe used for 0.13 stationary validation. |
| `firmware/fw7/build_vehicle.m` | Historical MATLAB helper for build vehicle. Supports original model setup/build/inspection. |
| `firmware/fw7/current_ina219.c` | Generated hardware-base support implementation for current_ina219. |
| `firmware/fw7/current_ina219.h` | Generated hardware-base declarations/configuration for current_ina219. |
| `firmware/fw7/START_HERE.txt` | Historical release/setup/validation notes: FW=7 independent GPIO/configuration diagnostics. This is NOT a confirmed repair.. Read the bundle validation record for the latest overall status. |
| `firmware/fw7/test_vehicle_protocol.m` | MATLAB regression tests for the corresponding motor, IMU or vehicle command/model behavior. |
| `firmware/fw7/Upload_Vehicle.ps1` | Explicit ST-LINK firmware upload/verification script; requires disconnected motor power/BEC. |
| `firmware/fw7/VALIDATION.txt` | Historical release/setup/validation notes: ARM compilation/linking passed. Both IRQ vectors verified in binary. FW=7 and diagnostic format verified. Existing dashboard parser accepts extra fields. Telemetry insertion and 254-byte length within 256-byte UART TX buffer verified. Not physically tested or flashed.. Read the bundle validation record for the latest overall status. |
| `firmware/fw7/vehicle_command.m` | Historical MATLAB helper for vehicle command. Supports original model setup/build/inspection. |
| `firmware/fw7/vehicle_crc16.m` | Historical MATLAB helper for vehicle crc16. Supports original model setup/build/inspection. |
| `firmware/fw7/Vehicle_Drive.bin` | Compiled FW7 Nucleo firmware reference. Uploader writes this image; no automatic flash occurs during Pi installation. |
| `firmware/fw7/Vehicle_Drive_generated.c` | Generated hardware-base support implementation for Vehicle_Drive_generated. |
| `firmware/fw7/vehicle_io.c` | Generated hardware-base support implementation for vehicle_io. |
| `firmware/fw7/vehicle_io.h` | Generated hardware-base declarations/configuration for vehicle_io. |
| `firmware/fw7/vehicle_text.m` | Historical MATLAB helper for vehicle text. Supports original model setup/build/inspection. |
| `firmware/motor_imu/bno055_controller.m` | Historical MATLAB helper for bno055 controller. Supports original model setup/build/inspection. |
| `firmware/motor_imu/bno055_text.m` | Historical MATLAB helper for bno055 text. Supports original model setup/build/inspection. |
| `firmware/motor_imu/build_motor_imu.m` | Historical MATLAB helper for build motor imu. Supports original model setup/build/inspection. |
| `firmware/motor_imu/inspect_motor_model.m` | Historical MATLAB helper for inspect motor model. Supports original model setup/build/inspection. |
| `firmware/motor_imu/INSTRUCTIONS.txt` | Historical release/setup/validation notes: MOTOR + BNO055 ORIENTATION TEST. Read the bundle validation record for the latest overall status. |
| `firmware/motor_imu/Motor_IMU_Test.slx` | Original Simulink motor plus BNO055 test model; a build input, not the current autonomous Pi program. |
| `firmware/motor_imu/open_motor_imu.m` | Historical MATLAB helper for open motor imu. Supports original model setup/build/inspection. |
| `firmware/motor_imu/test_motor_imu.m` | MATLAB regression tests for the corresponding motor, IMU or vehicle command/model behavior. |
| `firmware/pi_motor_imu/build_pi_motor_imu.m` | Historical MATLAB helper for build pi motor imu. Supports original model setup/build/inspection. |
| `firmware/pi_motor_imu/hardware/Core/Inc/main.h` | Generated hardware-base declarations/configuration for main. |
| `firmware/pi_motor_imu/hardware/Core/Inc/stm32_assert.h` | Generated hardware-base declarations/configuration for stm32_assert. |
| `firmware/pi_motor_imu/hardware/Core/Inc/stm32f7xx_hal_conf.h` | Generated hardware-base declarations/configuration for stm32f7xx_hal_conf. |
| `firmware/pi_motor_imu/hardware/Core/Inc/stm32f7xx_it.h` | Generated hardware-base declarations/configuration for stm32f7xx_it. |
| `firmware/pi_motor_imu/hardware/Core/Src/main.c` | Generated hardware-base support implementation for main. |
| `firmware/pi_motor_imu/hardware/Core/Src/stm32f7xx_hal_msp.c` | Generated hardware-base support implementation for stm32f7xx_hal_msp. |
| `firmware/pi_motor_imu/hardware/Core/Src/stm32f7xx_hal_timebase_tim.c` | Generated hardware-base support implementation for stm32f7xx_hal_timebase_tim. |
| `firmware/pi_motor_imu/hardware/Core/Src/stm32f7xx_it.c` | Generated hardware-base support implementation for stm32f7xx_it. |
| `firmware/pi_motor_imu/hardware/Core/Src/system_stm32f7xx.c` | Generated hardware-base support implementation for system_stm32f7xx. |
| `firmware/pi_motor_imu/hardware/motortest.ioc` | CubeMX hardware project: MCU clocks, USART3/USART6, TIM1 and board peripherals. Later direct-register encoder/I2C/servo setup is also in vehicle_io/current_ina219. |
| `firmware/pi_motor_imu/INSTRUCTIONS.txt` | Historical release/setup/validation notes: PI MOTOR CONTROL + LIVE BNO055 ORIENTATION. Read the bundle validation record for the latest overall status. |
| `firmware/pi_motor_imu/motor_crc16.m` | Historical MATLAB helper for motor crc16. Supports original model setup/build/inspection. |
| `firmware/pi_motor_imu/Motor_Pi_IMU.slx` | Original Pi-controlled motor/IMU Simulink base model used by vehicle model generation. |
| `firmware/pi_motor_imu/open_pi_motor_imu.m` | Historical MATLAB helper for open pi motor imu. Supports original model setup/build/inspection. |
| `firmware/pi_motor_imu/pi_motor_command.m` | Historical MATLAB helper for pi motor command. Supports original model setup/build/inspection. |
| `firmware/pi_motor_imu/pi_motor_text.m` | Historical MATLAB helper for pi motor text. Supports original model setup/build/inspection. |
| `firmware/pi_motor_imu/test_pi_motor.m` | MATLAB regression tests for the corresponding motor, IMU or vehicle command/model behavior. |
| `outputs/slam_explorer/AUTONOMY_0_13.txt` | Historical release/setup/validation notes: PI STATIONARY VALIDATION - 2026-11-06. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/behavior_trees/committed_path.xml` | Nav2 behavior tree for committed-path navigation and bounded recovery/retry behavior. |
| `outputs/slam_explorer/CAMERA_VALIDATION.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.4 CAMERA UPDATE - LOCAL VALIDATION. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/config/calibration.example.json` | Unverified template; null fields require real robot measurements before powered operation. |
| `outputs/slam_explorer/CONTINUOUS_EXPLORATION_UPDATE.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.11 - FORWARD PLANNING AFTER BLOCKED REVERSE. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/DASHBOARD_0_15.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.15 - DASHBOARD LAYOUT. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/DASHBOARD_UPDATE.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.9 - STOP DIAGNOSTICS, 20 SECTORS, LIVE MOTOR/SENSOR DISPLAY. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/FOOTPRINT_FIX.txt` | Historical release/setup/validation notes: 0.17 FOOTPRINT / OBSERVED MAP FIX. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/install_pi.sh` | Installs ROS/Python dependencies, backs up/copies source, builds robot_explorer and preserves existing Pi calibration. |
| `outputs/slam_explorer/launch/cameras.launch.py` | Starts front/rear usb_cam drivers on their configured USB port paths, topics, sizes and formats. |
| `outputs/slam_explorer/launch/explore.launch.py` | Starts navigation map, motion observer, serial bridge, EKF, SLAM, transforms, Nav2, explorer and dashboard. Motion defaults disabled. |
| `outputs/slam_explorer/measure_encoder.py` | Explicit supervised powered encoder-distance calibration utility; run-drive-test intentionally commands motors. |
| `outputs/slam_explorer/package.xml` | ROS package metadata and runtime dependency declarations. |
| `outputs/slam_explorer/PI_VALIDATION.txt` | Historical release/setup/validation notes: SLAM-EXPLORER-0.3 LIVE PI DIAGNOSIS. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/PLANNER_REWRITE.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.12 - STEERING-FEASIBLE FORWARD/REVERSE PLANNING. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/preflight.py` | Checks dependencies, ROS packages, robot calibration and any enabled camera calibration without opening motor serial. |
| `outputs/slam_explorer/REAR_CAMERA_SETUP.txt` | Historical release/setup/validation notes: FRONT + REAR USB CAMERA UPDATE - SLAM EXPLORER 0.4. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/REAR_COVERAGE_FIX.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.6 - REVERSE COVERAGE DIAGNOSTICS AND CORRIDOR FIX. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/REAR_COVERAGE_THRESHOLD.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.7 - USER-REQUESTED 50% REAR COVERAGE THRESHOLD. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/RECOVERY_UPDATE.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.5 - BOUNDED AUTOMATIC REVERSE RECOVERY. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/RESET_PI_VALIDATION.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.14 - PI VALIDATION 2026-11-06. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/RESET_UPDATE.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.14 - DASHBOARD RESET. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/resource/robot_explorer` | ROS ament package-index marker. |
| `outputs/slam_explorer/REVERSE_LIMIT_0_16.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.16 - REVERSE LIMIT FIX. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/REVERSE_TRACKING_UPDATE.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.8 - LONGER REVERSE RECOVERY AND TARGET COMPLETION FIX. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/robot_explorer/__init__.py` | Python package marker. |
| `outputs/slam_explorer/robot_explorer/bridge.py` | Sole motor-board serial owner. Publishes wheel/IMU/status messages, regulates duty and steering, enforces freshness/obstacle/reverse bounds. |
| `outputs/slam_explorer/robot_explorer/cameras.py` | Independent thread-safe front/rear JPEG caches with timestamp and stale-frame handling. |
| `outputs/slam_explorer/robot_explorer/configuration.py` | Builds runtime EKF, SLAM Toolbox and Nav2 parameters from the robot calibration. |
| `outputs/slam_explorer/robot_explorer/core.py` | Telemetry parsing, calibration checks, encoder scale/wrap handling, steering geometry and goal/failure selection helpers. |
| `outputs/slam_explorer/robot_explorer/dashboard.py` | Web dashboard: controls, authenticated Start/Stop/Reset, camera feeds, live LiDAR, map and motor telemetry. Publishes the operator heartbeat. |
| `outputs/slam_explorer/robot_explorer/explore.py` | Exploration state machine: selects/probes goals, follows Nav2 results, handles reverse recovery, cancellations and dashboard Reset. |
| `outputs/slam_explorer/robot_explorer/footprint.py` | Grid geometry, unknown-under-current-body overlay and finite observed-ray refresh; preserves occupied observations. |
| `outputs/slam_explorer/robot_explorer/fusion_math.py` | Rigid planar fit, scan ICP, body-frame motion conversion, current-weighted wheel variance and optional dual-encoder combination. |
| `outputs/slam_explorer/robot_explorer/motion_observer.py` | LiDAR scan odometry and optional calibrated ground-plane camera motion observations; rejects stale/weak/outlier observations. |
| `outputs/slam_explorer/robot_explorer/motor_base.py` | Shared serial-control handshake, command sequencing, watchdog state and original command-line motor utility. |
| `outputs/slam_explorer/robot_explorer/navigation_map.py` | Publishes navigation/map from raw SLAM plus current footprint and fresh finite scan evidence. Shared by explorer, Nav2 and dashboard. |
| `outputs/slam_explorer/robot_explorer/retrace.py` | Breadcrumb storage, bounded reverse route tracking, straight backup and rear LiDAR-sector coverage checks. |
| `outputs/slam_explorer/robot_explorer/route_planner.py` | Three-stage steering-lattice candidate search with oriented footprint checks, information gain and revisit penalties. Nav2 validates the final path. |
| `outputs/slam_explorer/robot_explorer/telemetry_view.py` | Converts board encoder/current/bus-voltage fields into timestamped display readings; missing sensor data remains unavailable. |
| `outputs/slam_explorer/robot_explorer/vehicle_control.py` | Vehicle command protocol, signed dual motor duty, steering pulse mapping, arming and serial framing. |
| `outputs/slam_explorer/ros_smoke.py` | Real ROS motion-observer constructor check, without opening serial or issuing drive commands. |
| `outputs/slam_explorer/setup.cfg` | ROS setuptools install locations for package executables. |
| `outputs/slam_explorer/setup.py` | ROS Python package version, installed resources and executable entry points. |
| `outputs/slam_explorer/SHA256SUMS.txt` | Historical release/setup/validation notes: f402fc6316a1a40189ba83f280cf41ca3cc8372eb36db7756d9d1f4b17cee41c  AUTONOMY_0_13.txt. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/START_HERE.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.17 - CURRENT RELEASE. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/STRAIGHT_REVERSE_UPDATE.txt` | Historical release/setup/validation notes: SLAM EXPLORER 0.10 - STRAIGHT REVERSE AFTER ROUTE DEVIATION, LOCKED STEERING. Read the bundle validation record for the latest overall status. |
| `outputs/slam_explorer/tests/fixtures/stationary_map.json` | Captured occupancy map and pose fixture used for offline path/footprint regression replay. |
| `outputs/slam_explorer/tests/fixtures/stuck_map.json` | Captured occupancy map and pose fixture used for offline path/footprint regression replay. |
| `outputs/slam_explorer/tests/test_arm_wait.py` | Regression tests: stale disarmed status does not cancel start; fresh armed status releases wait; no arm acknowledgment times out. |
| `outputs/slam_explorer/tests/test_backup_recovery.py` | Regression tests: straight backup is bounded and behind current heading; no forward history can use one short backup; no backup until navigation cancellation completes; stale odometry prevents backup; budget resets after forward progress not reverse. |
| `outputs/slam_explorer/tests/test_cameras.py` | Regression tests: separate feeds disconnect and reconnect; old future invalid and repeated source frames are rejected; source delay counts towards expiration; http routes authenticate and never return stale video; camera launch keeps devices and namespaces separate. |
| `outputs/slam_explorer/tests/test_core.py` | Regression tests: counter wrap both directions; accept actual fw7 diagnostics; measurements required; counts per rev alternative; nonfinite and servo limits; curvature and no spin; parameter sensor masks; turn radius matches bridge; positive defaults are serialized; frontier reachable and known; no unknown or occupied origin goal; symmetric open map has frontier goal not robot centroid; unreachable frontier not selected; failure memory and expiry; heading preference and alternative frontier; bt has no periodic retarget or spin. |
| `outputs/slam_explorer/tests/test_dashboard.cjs` | Executes the actual dashboard JavaScript with simulated browser/network responses: feeds, stale clearing, reset/authentication, layout, LiDAR and Start gating. |
| `outputs/slam_explorer/tests/test_dashboard_lidar.py` | Regression tests: calibrated mount transforms display into robot coordinates; expired scans are removed from display; stale source frame is rejected; missing transform does not display misleading points. |
| `outputs/slam_explorer/tests/test_drive_gates.py` | Regression tests: zero velocity centers steering before straight recovery; stale command sends zero; sensor failure disarms; operator expiry disarms; obstacle sends zero without retargeting; stopped encoder disarms; reverse not supported; excess curvature caps and checks actual turn; duty is bounded and same polarity; full duty ceiling and configured lower ceiling; passive launch refuses enable. |
| `outputs/slam_explorer/tests/test_footprint.py` | Regression tests: unknown under body is known but occupied is preserved; empty unknown world never authorizes a route; rotation changes intersection; live ray clears unknown but not wall or unseen side; ray endpoint is occupied; captured map no longer rejects current body. |
| `outputs/slam_explorer/tests/test_forward_after_reverse.py` | Regression tests: blocked reverse stays enabled then probes forward after pause; no clear forward goal waits without disarming or driving; sensor fault still disarms instead of switching direction; reverse timeout returns to forward planning. |
| `outputs/slam_explorer/tests/test_fusion.py` | Regression tests: scan motion sign scale and rotation; stationary scan is accepted; single wall and sparse scan are rejected; motion outliers and time gaps are rejected; load and missing current reduce wheel confidence; ekf fuses only independent twists from motion observers; second encoder disagreement increases uncertainty; missing unverified camera calibration rejected; calibrated ground flow produces metric motion. |
| `outputs/slam_explorer/tests/test_heartbeat.py` | Regression tests: authenticated heartbeat is independent of state render. |
| `outputs/slam_explorer/tests/test_reset.py` | Regression tests: reset clears failure state and never arms; reset waits for fresh stop and sensor health; stop clears pending and fences unanswered requests; late acceptance after reset is cancelled and fenced; old result cannot clear new pending or target; repeated reset start needs explicit start each time; planned reverse limit cancels then replans without disarm. |
| `outputs/slam_explorer/tests/test_retrace.py` | Regression tests: reverse budget resets only between stopped maneuvers; between sparse breadcrumbs is on route; real continuous path deviation still stops with measured error; recorded recovery can use two metres; endpoint overshoot finishes instead of faulting; no remaining point behind stops without reverse toward front; one metre reverse tracks for longer than old timeout; bridge allows reverse beyond old distance and time bounds; extended reverse still enforces time bound; paused time does not consume reverse budget; limit stays zero until stopped authorization is withdrawn; only recorded route and distance cap; stopped history still valid but expired or offset not; jump clears old route; reverse command negative and completes; route deviation stops; rear coverage is required; side gap outside straight corridor does not block reverse; small corridor gap is reported and allowed; all invalid scan is unknown and mount yaw is applied; exactly twenty sectors and half threshold; detected rear obstacle stops even with sufficient coverage; bridge reverses only with authorization and rear coverage; reverse distance bound stops without disarming; reverse checks rear not forward swept direction. |
| `outputs/slam_explorer/tests/test_ros_names.py` | Regression tests: explorer does not overwrite ros node handle. |
| `outputs/slam_explorer/tests/test_route_planner.py` | Regression tests: open map produces feasible arcs with correct goal heading; unknown or occupied routes are not authorized; reverse is available when front is a dead end; visited viewpoint has lower score; actual stationary robot map has a steering feasible viewpoint. |
| `outputs/slam_explorer/tests/test_straight_recovery.py` | Regression tests: pulse limits and straight are exact; reconfiguration cannot move center or reduce locked span; straight recovery does not depend on old route error; actual explorer switches without disarm and keeps original timer. |
| `outputs/slam_explorer/tests/test_telemetry_view.py` | Regression tests: both encoder rates handle rollover and current units; stale missing and offline do not show zero as measurement; custom shunt and invalid resistance. |
| `outputs/slam_explorer/Update-Autonomy.ps1` | Uploads the adjacent slam_explorer_pi.zip and runs its Pi installer; does not launch or flash. |
| `outputs/slam_explorer/Update-Cameras.ps1` | Older camera-update upload/install helper. Retained for history; use Update-Autonomy.ps1 for current installation. |
| `outputs/slam_explorer_pi.zip` | Versioned installable Pi release archive; includes that release's source, tests and notes. Current default archive is 0.17. |
| `README.md` | Bundle overview, capability limits, dependency/build scope and entry links. |
| `releases/slam_explorer_pi_0.12.zip` | Versioned installable Pi release archive; includes that release's source, tests and notes. Current default archive is 0.17. |
| `releases/slam_explorer_pi_0.13.1.zip` | Versioned installable Pi release archive; includes that release's source, tests and notes. Current default archive is 0.17. |
| `releases/slam_explorer_pi_0.14.zip` | Versioned installable Pi release archive; includes that release's source, tests and notes. Current default archive is 0.17. |
| `releases/slam_explorer_pi_0.15.zip` | Versioned installable Pi release archive; includes that release's source, tests and notes. Current default archive is 0.17. |
| `releases/slam_explorer_pi_0.16.zip` | Versioned installable Pi release archive; includes that release's source, tests and notes. Current default archive is 0.17. |
| `releases/slam_explorer_pi_0.17.zip` | Versioned installable Pi release archive; includes that release's source, tests and notes. Current default archive is 0.17. |
| `START_COMMANDS.md` | Complete installation and three-terminal sensor/navigation launch instructions. |
| `VALIDATION.md` | Version-by-version validation record, including the unresolved final 0.17 route check. |
| `FILE_GUIDE.md` | This per-file explanation index. |
| `MANIFEST_SHA256.txt` | SHA256 checksums for the bundle files, excluding the manifest itself. |
| `.gitattributes` | Preserves checked-in bytes across platforms so manifest hashes remain stable. |
