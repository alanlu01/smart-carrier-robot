# Stopped localization and transient station TF recovery — 2026-11-17

Base: `db112da` (vehicle, local main and origin agreed before work).

## Behavior

- Stationary AMCL displacement detection now requires the existing complete-wheel stop proof, fresh odometry and the existing one-second command-settling interval. A pose anchor must have a new, increasing AMCL source stamp after the settled boundary. A delayed braking pose cannot establish or move that anchor. Genuine subsequent displacement, the independent odometry guard and global geometry checks remain enabled.
- Trusted pre-outage map→odom references include TF/AMCL/odom source stamps, associated odometry and quality. Only healthy map/AMCL observations with fresh independent source data create a reference. Cache use is bounded to ten seconds, twelve seconds of TF source age, and existing 20 cm / 10 degree odometry displacement limits. AMCL's configured one-second future TF tolerance is accounted for. Human seeds and pose recovery invalidate the old cache.
- A stopped pipeline with fresh sources but no trusted reference immediately enters the existing bounded no-motion pose verification; it cannot accept recovered TF as a historical reference. The pipeline outage deadline remains thirty seconds, and reference-based resume still requires six independent map/AMCL observations.
- An active alignment encountering stale TF/costmap/odometry cancels its native action first. Only after terminal ACK, fresh post-pause TF/odom/costmap, continuous 0.5-second stopped evidence, unchanged physical translation within 5 cm and clear remaining turn can it resume. There is one data pause, two alignment submissions total, one six-second grace and the unchanged 240-second mission sequence ceiling. Cancellation, permission revocation, repeated outage, displacement, blockage and deadline remain failures. No velocity bypass or parallel goal is added.
- `/delivery/station_state` reports separate failure kinds, pause state and obstacle cell/world position plus static/dynamic/unknown classification. Console messages retain brief icons and existing throttling. Geometry uses the identical body/cell overlap check for decisions and diagnostics.

## Validation

- 910 functional tests across delivery, power, vision and API; existing copyright/flake8/PEP257 lint suites are excluded from this functional count.
- Added deterministic production-method regressions for late braking AMCL, genuine subsequent displacement, old/future source stamps, bounded historical references, stale receipt/source mismatch, alignment action drainage, new-source and stop gating, cancellation, physical displacement, blocked turns and fixed deadlines.
- Isolated ROS Domain 231 / LOCALHOST integration: real DeliveryNavigator and action cancellation/terminal ACK with a controlled TF publishing outage; one resumed native alignment, final parking successful, zero velocity publications.
- Production callback-load and recorded static-map action test: remote obstacle reobservation, parking/departure, permission cancellation, bounded missing-data failure and restart all passed, zero velocity publications.
- Real launch entrypoint bootstrap: map/localization/Nav2 ready without external map TF, zero navigation goals and zero nonzero commands, all children exited cleanly.

## Deployment and field follow-up

Vehicle source and core runtime backup: `/home/kj0921/backups/20261117_recovery_safety/`.
Candidate logs: `/home/kj0921/validation_20261117_recovery/`.
No speed, frequency, localization quality threshold, footprint, service point, power model or frontend/API schema changes. No physical driving was performed.

Next field recording should cover braking while heading changes, people-analysis navigation, a short weak-network/TF gap during alignment and service-point departure. Confirm no false kidnapped recovery, no empty thirty-second reference wait, bounded resume after fresh data and strict final parking. Real remote-network behavior and physical obstacle clearance still require this field test.
