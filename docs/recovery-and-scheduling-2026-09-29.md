# Recovery and scheduling corrections after the September 24 test

## Localization

- Keep six independent confirmation samples and the existing covariance,
  scan-to-map, footprint, collision and chassis-watchdog safety thresholds.
- Request no-motion AMCL updates every 1 second while verifying, suspect or
  recovering; normal localized operation retains the existing 3-second cadence.
- Local recovery waits at least 12 seconds. A computed lower bound additionally
  budgets map-window warmup, six independent updates and scheduling slack, so
  changing the configured refresh interval cannot recreate a nine-second wait
  for six three-second samples.
- Verification deadlines count usable observation time. Missing or stale raw
  scan, filtered scan, odometry, AMCL or map/TF observations pause the deadline
  and reset confirmation samples. Receiving an old scan does not make its
  timestamp fresh. No new localization spin begins while those data are missing.
- A continuous data-pipeline outage of 30 seconds requests manual assistance
  while stopped; this is reported as a pipeline failure, not vague low AMCL
  confidence. The short transient-recovery window also excludes data outages
  and ongoing physical take/return inhibition. Two independently confirmed
  healthy samples can still clear a stationary transient fault without spinning.
- Publish localization state at approximately 1 Hz even during stationary AMCL
  periods. New optional fields include confirmation count, effective verification
  elapsed time and unavailable-data blockers. Existing state/ready fields and
  frontend/API contracts are unchanged.
- Initial-pose cache policy, spin speed, heading correction and power-bank
  confirmation logic are unchanged. This correction does not prove that the
  underlying scan/odom/TF/DDS interruption has been eliminated.

## Multi-order scheduling

Feasible first stops retain the original priority `distance * 0.7` for borrow
and `distance` otherwise. Complete unweighted route length is the next
comparison key; original claim order is only the final tie-breaker. Inventory
simulation still rejects routes that double-allocate a bank or return into a
full carrier. Single-order and partially filled batch behavior is unchanged.

The regression scenario uses start `(2.4289, 4.9260)`, borrow at `(-2.6, 4.7)`
and return at `(7.5, 5.6)`. It must choose borrow then return, rather than discount
the 10.14m leg to a postponed borrow. This is a first-stop-priority policy, not an
unconstrained globally shortest route or a measured wall-aware travel-time model.

## Static validation and next field test

Run the functional pytest suites using ROS 2 Jazzy on the Raspberry Pi. Manager
tests use production callbacks with a fake observation clock and real ROS
message classes; they do not start a robot node or send chassis commands.

At a verified map location with a safe operator present:

1. Submit one order; verify it departs without waiting for two more.
2. Submit the September 24 borrow/return pair together from CYCU EE; verify both
   share one batch and borrow is first. Verify two and three tasks execute without
   returning to standby between legs, unless inventory makes that impossible.
3. Observe a temporary SUSPECT event; high map match plus fresh healthy AMCL
   should accumulate six confirmations and resume without unnecessary spin.
4. During a controlled network interruption, verify safe stop if local data
   become unavailable, paused verification timing, then new confirmation samples
   after data return. An actual pipeline fault can still require repair/help.
5. During physical take/return, verify no autonomous spin and that a short
   recovered sensor interruption does not exhaust the transient window merely
   because the slot operation is ongoing.
6. If real localization is lost, verify staged collision-checked recovery still
   occurs. Poor/ambiguous covariance must not be accepted just because map match
   is high. Record raw/filtered scan, odom, AMCL, TF and localization state.

No frontend/API migration is required for this revision. Real motion, stopping
distance, Wi-Fi/DDS interruption behavior and AMCL convergence require field
verification; static regression tests are not a substitute.
