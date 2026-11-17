# People idle timing and nonblocking data-pause recovery — 2026-11-17

Base: `b0b476c`; local/vehicle main were clean and agreed before work.

## Changes

- Keep the original verified station point and its exact navigation goal. Only the auxiliary people idle-clock reference may rebase once after a gradual AMCL correction: continuous physical odometry/stop proof, fresh safe inputs, unchanged trusted localization quality gate (xy_std <= 0.20 m, yaw_std <= 8 degrees, map score >= 0.60), stable position/heading for three seconds, original XY <= 0.35 m and original yaw <= 20 degrees. The ordinary auxiliary counting hysteresis remains 0.20 m / 0.20 rad around the auxiliary reference.
- Preserve accumulated auxiliary time for at most thirty seconds while awaiting a bounded gradual correction. Expiry clears it once and retains the physical anchor. Continuing bad data cannot alternate reset/pause messages indefinitely; new healthy continuous evidence can subsequently confirm a reference and start fresh time. Abrupt map jumps, wheel movement, missing physical history, long TF gaps and invalid data disqualify rebasing until a new verified parking proof. No unobserved time is counted.
- StationNavigation submits one asynchronous native cancel request, then continues bounded executor pumping. A cancel response is not terminal completion: the original action retains ownership until its result is terminal. A transport exception cannot authorize a successor. BasicNavigator is not globally patched.
- Optional cancellation observes stopped evidence while the action drains, reusing the same continuous one-second stop dwell rather than always starting another dwell after the terminal result. Resume after a data pause requires post-pause TF/odom/scan source timestamps and a continuous 0.20-second fresh safety interval. A stale interval or executor gap resets confirmation. Original twenty-second wait, navigation/session deadlines, pause budgets, new-service priority and localization-escalation behavior remain.
- Console text distinguishes stopping/waiting on the route to observation, spin data pause, completed scan with insufficient usable people evidence, stationary map correction and cooldown. Cooldown seconds take precedence over the auxiliary sixty-second timer while cooldown is active. Repeated idle text is limited; existing progress messages retain their cadence.
- Existing optional_idle_state adds idle-clock reference/reset/confirmation evidence and data-resume source/confirmation diagnostics; station_state adds asynchronous cancel-request timing. Recording topics are unchanged.

## Evidence and validation

- 929 functional tests across delivery/power/vision/API passed (copyright/flake8/PEP257 lint excluded from the functional count).
- Offline reading of the 2026-11-17 evening bag, without a ROS node or replay: 170 published-status samples during verified parking, approximately 13.0 degrees of map-heading correction and essentially zero odometry translation. The new clock reset once. At the recorded window end trusted confirmation was still only about two seconds, so it correctly did not immediately rebase. A separately labelled synthetic continuation with fresh trusted stationary evidence verifies recovery after the required three seconds, with zero restoration of expired accumulated time. The old clock comparison is a model at the recorded status cadence, not a claim of the original application's exact reset count.
- Isolated Domain 231 / LOCALHOST integration with a three-second delayed cancel response: cancellation submission about 0.00015 seconds, polling about 0.10 seconds, terminal drainage about three seconds; no successor before terminal completion.
- Full OptionalIdleRuntime + StationNavigation + real action/executor/TF integration: source outage stops/drains, keeps the original deadline and observation target, waits for new source stamps/stop, resumes successfully; max polling about 0.11 seconds even with delayed cancel response.
- Existing production-callback-load/recorded-map station routing, remote obstacle reobservation, permission cancellation, bounded missing-data and restart, plus alignment TF pause/resume tests passed. No velocity publications in action integration fixtures.
- Real launch/bootstrap verification runs with software sensors in the isolated domain; no physical vehicle driving is performed.

## Deployment

Source/core-runtime backup: `/home/kj0921/backups/20261117_people_idle/`.
Candidate and deployment logs: `/home/kj0921/validation_20261117_people_idle/`.

No changes to speeds, navigation/vision frequencies, people confidence threshold, physical footprint, station coordinates, arrival tolerances, global localization thresholds, battery model, frontend or API migrations. Main-battery real home triggering remains disabled as before.

Next physical test: normal service, idle at CYCU EE after localization recovery, people-analysis navigation under short TF/network jitter, and new-order interruption. Confirm a stable corrected heading can restart auxiliary counting, expired accumulated time stays expired, actual displacement still blocks rebasing, and a data pause resumes only after fresh inputs. Overall CPU reduction and actual weak-network behavior are not established by the software fixtures.
