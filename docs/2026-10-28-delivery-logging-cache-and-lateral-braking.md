# 2026-10-28 Delivery crash, trusted-pose I/O and recovery braking

## Problems and resulting behavior

- Regression in `242ee2d`: `_safe()` selected INFO/WARN on the same physical
  rclpy callsite. A change of severity raised `ValueError` and exited Delivery.
  The two levels now have distinct callsites. Fatal Delivery exceptions retain
  the original traceback, stop renewing the independent goal-guard lease,
  cancel/drain the accepted or pending goal, and attempt a continuously fresh
  one-second odometry stop confirmation within five seconds. A timeout never
  releases the guard or resumes service; the original exception still exits.
- A 3.956-second AMCL callback stall coincided with I/O pressure during the test.
  The synchronous trusted-pose write was a candidate, not a proven substep.
  Ongoing trusted-pose persistence now uses one background worker with one
  replaceable pending snapshot. No worker calls ROS or changes control state.
  Startup boot-marker writing remains synchronous. Atomic replacement, boot ID,
  map fingerprint and the original AMCL observation timestamp are retained.
  Delayed disk completion never refreshes an old observation's age.
- Lateral recovery exceeded the existing 0.33 m odometry limit by about 3.65 mm
  in the recording, so stopping before the second spin was correct. Recovery
  alone now caps lateral speed at 0.20 m/s and brakes earlier using a 0.25 m/s²
  envelope, 0.20 s reaction allowance plus current odometry receipt age, and a
  0.025 m stopping tail. Target remains 0.30 m; 0.33 m distance, 0.05 m cross-track
  and 5° yaw limits and real wheel/odometry stop checks remain enforced.

## Additional rosbag evidence

Existing `/localization/state` now includes:

- `pose_cache_io`: active I/O elapsed time, one-slot pending flag, submission,
  replacement, success/failure counters, last error and constant-space timings
  for serialization, file write, atomic replace and total write. Completed
  operation peak ages continue aging in snapshots.
- `lateral_motion_timing`: original odometry pose, progress, cross-track and yaw
  at start, motion authorization, first deceleration, first zero command and
  verified wheel stop. Time is elapsed monotonic seconds from this movement.

No recorder topics were removed or changed. Continue with:

```bash
bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh
```

## Validation and practical limits

- 613 functional/safety regression tests on the Raspberry Pi: real rcutils
  INFO→WARN→INFO transitions; accepted/pending action cleanup; no stop evidence
  and undrained actions; blocked file I/O with responsive AMCL callbacks;
  latest-only queue; write failures; bounded worker shutdown; original source
  time/map/boot checks; delayed braking simulation; unchanged stop lease and
  second-spin protections. Existing lint-only suites remain excluded by the
  established verification script.
- Localhost ROS domain 231 callback/status smoke covers real severity
  transitions without goals, motor commands or production-domain discovery.
- Real manager construction/state serialization uses a temporary pose-cache
  path and disables initialization/motion recovery. No executor is spun.
- Simulated braking is not a physical chassis calibration. Next field test
  must confirm stopping distance, successful second rotation when safe, full
  people analysis, and absence of long AMCL callback stalls.

Map, coordinates, normal delivery/rotation speed, scan timeouts, 15° people
blind-arc threshold, AMCL confidence/six-sample gates, battery and API contracts
are unchanged. No web migration or teammate changes are needed.

Predeployment backup:
`/home/kj0921/backups/delivery_io_braking_20261028_vXYfDD/dev_ws_src.tar.gz`

Validation directory:
`/home/kj0921/test_logs/validation_20261028_delivery_io_braking_UJX3DT`
