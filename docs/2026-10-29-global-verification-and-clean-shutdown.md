# 2026-10-29 Global pose gate, clean stop and recovery observations

## Resulting behavior

1. Global AMCL candidates and same-boot cached startup poses require a separate
   static-map ray consistency check before the original six confirmations can
   authorize navigation. Up to 60 valid non-near endpoints are selected over
   12 sectors. At most 512 grid cells per ray are traversed, capped at 8 m.
   Rays crossing a fixed wall more than 0.30 m before their measured return
   contradict the candidate. The conservative initial gate requires conflict
   fraction <=0.30, endpoint support >=0.55 and >=6 supported sectors (two
   matched endpoints each), with fresh source scans. A grid DDA avoids skipping
   thin walls at floating-point cell boundaries. Empty/unknown/invalid evidence
   never passes. The check runs with the existing ~1 Hz map update only while
   global verification is required, not during normal localized driving.
   A human RViz seed still uses the existing six-sample/map/covariance checks.
2. SIGINT/Ctrl+C and SIGTERM unwind Delivery while its ROS context remains
   valid. It first stops lease renewal, drains/cancels owned accepted or pending
   goals, and requires one second of fresh stopped feedback within the existing
   five-second cleanup budget. It then shuts down ROS, keeping the independent
   guard latched if stopping was unconfirmed. A second signal
   cannot interrupt normal cleanup. Invalid contexts return immediately with
   an unconfirmed-stop result instead of repeatedly attempting executor calls.
   Genuine exceptions still retain traceback/error reporting and the independent
   guard is never released on unproven stopping.
3. A successful lateral stop enters `second_guard_wait` with a new bounded
   observation window, preserving the current AMCL candidate. It refreshes map
   and no-motion evidence, allows six seconds to gather the original samples,
   and up to eight if evidence is accumulating. It can recover in place; otherwise
   the rotation scan gate must pass before any second global redistribution.
   The second-sidestep prohibition, two-global-attempt budget, 0.20 m/s cap,
   0.33 m/cross-track/yaw limits, 15° blind arc and 75% valid ratio remain intact.
4. Verified parking completion now uses ROS logging, including ordinary standby
   return. A logged safety pause has a corresponding recovery message. Healthy
   parked idle status reports remaining idle/cooldown time at most every 30 s.
   Only an otherwise safe, localized, stationary TF gap can pause auxiliary
   elapsed idle time for at most one second; the gap earns no time and cannot
   authorize motion. Persistent gaps, physical displacement, no parking proof
   and other unsafe conditions reset the timer as before. The 60 s idle and
   600 s analysis cooldown and existing arrival criteria are unchanged.
5. The navigation launch asynchronously saves only the Nav2 container's stdout,
   stderr and exit metadata alongside its launch logs:
   `~/.ros/log/<launch-directory>/nav2_container_<pid>.{stdout.log,stderr.log,exit.json}`.
   Capture uses a bounded 2 MiB pending buffer and records drops/write errors;
   it never blocks control or changes lifecycle state. Existing lifecycle/bond
   protection is unchanged. The recorder adds `/bond` and AMCL/map/controller/
   navigator/behavior lifecycle transitions, preserving all prior topics.

New diagnostics are included in the existing `/localization/state`:
`global_pose_gate`, `post_lateral_observation`, and geometry failure detail.
No API schema, task result contract or web migration is needed.

## Evidence and validation

Offline evaluation of the retained 10/29 recording uses the actual recorded
static/dynamic TF at each scan timestamp, not replay or a ROS executor. In the
known wrong-candidate interval 13:44:40–13:47:12, all 135 evaluated snapshots were
rejected (one extra TF sample unavailable). Across three reference intervals,
71 of 74 snapshots passed; the three transient rejections cannot alone authorize
or terminate navigation because the normal runtime gates remain in place.
On the Pi the pure geometry calculation averaged about 3.8 ms, peak 6.9 ms in
this offline run. This is a case regression, not proof of unique localization
in every symmetric or changed environment; field validation remains necessary.

Validation passed 631 functional/safety regression tests, real rcutils severity and
recovery messages, isolated real manager startup/state serialization, real
self-SIGINT/SIGTERM cleanup with live ROS callbacks in domain 231, and actual
launch stdout/stderr/exit events using a temporary shell helper. The helper's
intentional exit 7 verifies error capture and is not a real Nav2 crash.

The established lint-only suites remain excluded. No real robot goal, velocity
command or Wi-Fi disconnection test was executed during deployment. The actual
Nav2 SIGABRT cause remains open; the new transcript is intended to capture the
missing C++ evidence on a subsequent occurrence.

Backup: `/home/kj0921/backups/global_stop_idle_20261029_E70nPy/dev_ws_src.tar.gz`.
Validation: `/home/kj0921/test_logs/validation_20261029_global_stop_idle_Aalsuy`.

## Next field check

Run the same recorder command; no topics were removed:

```bash
bash ~/dev_ws/src/smart-carrier-robot/tools/record_full_system.sh
```

Check a normal service and standby return, completed/recovered/cooldown text,
moving the vehicle for global relocalization, safe lateral recovery followed
by in-place convergence or the second spin, and user Ctrl+C while supervised.
If geometry remains ambiguous, the robot must retain work and remain stopped
rather than declare a precise position. Keep current navigation speed and FPS.
