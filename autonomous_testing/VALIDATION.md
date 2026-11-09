# Validation record

The user reported satisfactory dashboard behavior during testing. That report is distinct from autonomous navigation validation.

- 0.12: release notes record bounded indoor powered tests, including corrected steering signs and successive goals. These were not indefinite outdoor tests.
- 0.13 and 0.13.1: multi-input motion observation support and a real ROS constructor correction. Camera odometry remained disabled without calibration.
- 0.14: 96 tests passed on Pi; two authenticated Stop/Reset HTTP cycles reached Ready on the same token/page, with zero motor output.
- 0.15: 100 tests passed on Pi. The dashboard was reordered and a calibrated live LiDAR view added.
- 0.16: 103 tests passed on Pi. Reverse output time replaced paused wall time; reaching a bound stopped/replanned without disarming. Rear coverage became one of twenty sectors by user request.
- 0.17: 109 tests passed on Pi. The current-footprint false rejection was removed; raw SLAM was preserved with a separate current-footprint/fresh-observation navigation map. The final stationary live check still rejected all 40 sampled candidates outside the footprint. No successful current-position Nav2 path was demonstrated by that check; motors stayed disabled. Basic LiDAR target alignment was subsequently reported validated by the user.

The later Navigation/drive service readiness check found the navigator, controller and planner active and both navigation and drive services available. This confirms startup readiness, not route feasibility.

The November 8 packaging run could not contact the Pi at its last-known address. Calibration is a November 6 snapshot. Neither the public repository overview nor these files should be read as evidence of calibrated camera clearance, both encoders working, calibrated motor protection, or uninterrupted grass exploration.
