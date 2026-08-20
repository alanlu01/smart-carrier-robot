# Raspberry Pi API migration deployment

Deployment completed on 2026-08-20 (Asia/Taipei) for `kj0921@172.20.10.5`.

## Deployed source

- Repository: `alanlu01/smart-carrier-robot`
- Branch: `codex/integrate-api-bridge`
- Functional integration commit: `4324b0f811c9442a1d9ec3751305e331314942d6`
- Raspberry Pi checkout: `/home/kj0921/dev_ws/src/smart-carrier-robot`
- Repository-specific SSH key configuration is stored in the checkout's local Git config.

The top level of `dev_ws/src` now contains:

- `camera_ros`
- `rf2o_laser_odometry`
- `smart-carrier-robot`

Colcon recursively discovers the five project packages and customized RPLIDAR package inside the monorepo. There are no duplicate ROS package names.

## Recovery material

- Complete pre-migration archive: `/home/kj0921/backups/pre_api_integration_20260820_222122.tar.gz`
- Archive SHA-256: `ace646b7e94cc658caaba991ce12cf3a922b0644e3fee614c1e7008f09b2d09f`
- Archive validation: `gzip -t` passed; 3,826 archived entries.
- Original project package directories: `/home/kj0921/backups/dev_ws_src_pre_api_20260820_222122/`
- Retired standalone workspace: `/home/kj0921/backups/smart_carrier_ws_retired_20260820_222122/`
- Pre-migration shell configuration: `/home/kj0921/.bashrc.pre_api_20260820_222122`

No original project package or standalone workspace was permanently deleted. Restore operations should only be performed after stopping robot processes and confirming the exact target paths.

## Shell path update

The `start_nav` alias in `/home/kj0921/.bashrc` was updated so its map and Nav2 parameter paths point to:

```text
/home/kj0921/dev_ws/src/smart-carrier-robot/packages/smart_delivery_core/
```

Aliases that resolve installed ROS packages required no changes.

## Validation

- Full `colcon build --symlink-install`: 8/8 packages completed.
- Python and ROS integration tests in the deployed workspace: 8/8 passed.
- `power_monitor`, `smart_carrier_api`, and `smart_delivery_core` resolve from `/home/kj0921/dev_ws/install`.
- INA3221 hardware access succeeded with all three observed channels at `0.0 A`, `empty`, and `sensor_ok: true` during the test.
- The deployed API bridge received the three-slot `power_status` payload.
- Authenticated heartbeat to `https://api.138-2-34-203.sslip.io` succeeded.
- Task polling was set to 3,600 seconds during validation, so the test did not claim or alter an order.

The RPLIDAR SDK emitted its existing compiler warnings; no RPLIDAR build error remained after refreshing the CMake source-path cache.
