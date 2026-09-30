# canopen_402_driver

`ros2_canopen::Cia402Driver` (and `LifecycleCia402Driver`) drives CiA 402 motor controllers. One
CANopen node can carry up to three motor channels (CiA 402 multi-axis profile: channel 2 at
+0x800, channel 3 at +0x1000). Each channel is one joint. In this stack the driver is used from
[`canopen_ros2_control/Cia402System`](../canopen_ros2_control/README.md): there, the
`MotorManager` owns init, recovery, mode switching and PDO repair.

It is built on the proxy driver, so it keeps the NMT/SDO services of
[`canopen_proxy_driver`](../canopen_proxy_driver/readme.md) (disable them with
`enable_ros_interfaces: false`).

## Requirements and capabilities

| Question | Answer |
|---|---|
| EDS needed? | Yes, with the CiA 402 objects below. The master checks the device identity (0x1000, 0x1018) against the EDS during boot, and a mismatch fails the boot. |
| SDO only? | Yes, if the objects are not mapped in the EDS. See [SDO-only operation](#sdo-only-operation). It works, but it is slow and puts CAN round trips into the control loop. |
| PDO operation | Recommended. Map the objects in the [recommended mapping](#recommended-pdo-mapping). |
| Who configures the PDOs on the device? | The master, during boot (`boot: true`): it downloads the COB-IDs, transmission types and mappings from `bus.yml` via the generated `<name>.bin`. Mappings not given in `bus.yml` keep the EDS/factory values. With `Cia402System`, `MotorManager` also re-checks the COB-IDs of RPDO/TPDO 1-4 on the device and repairs them (see the [PDO repair caveat](#pdo-repair-caveat)). |
| NMT start | The master sends NMT Start after the boot process (`boot: true`, master `start_nodes: true`). |
| Auto init after power loss | Yes. On a boot-up message the master reconfigures and restarts the node. With `Cia402System`, `MotorManager` notices that the drive has left Operation Enabled and re-runs init: fault reset, Operation Enabled, default mode. If the node stops answering, `MotorManager` sends an NMT Reset Node at most every 10 s. |
| Auto enable | Yes with `Cia402System`: init, fault reset and mode switch run on activation and are retried forever (e.g. during a long e-stop). With the plain ROS interface, call `~/<joint>/init` and then a mode service. |
| Startup retries | The device container retries a failing driver init 5 × 2 times, with an NMT Reset Node between the blocks. After that, bring-up fails. |
| Heartbeat | Optional (`heartbeat_producer`). Loss of communication is detected through SDO errors. In PDO mode, cached values never fail, so a silent drive is not detected until an SDO access fails. |

## Objects used

For channel 1. Channel 2 adds 0x800, channel 3 adds 0x1000.

| Object | Name | Use | Access |
|---|---|---|---|
| 0x6040 | controlword | written every `period` | RPDO or SDO |
| 0x6041 | statusword | read every `period` | TPDO or SDO |
| 0x6060 | modes of operation | written on mode switch | RPDO or SDO |
| 0x6061 | modes of operation display | read every `period`; always polled by SDO during a mode switch | TPDO or SDO |
| 0x6064 | position actual value | read in ros2_control `read()` | TPDO or SDO |
| 0x606C | velocity actual value | read in ros2_control `read()` | TPDO or SDO |
| 0x6502 | supported drive modes | read once per init to register modes | SDO |
| 0x607A | target position | PP (1), CSP (8) | RPDO or SDO |
| 0x60FF | target velocity | PV (3), CSV (9) | RPDO or SDO |
| 0x6042 | vl target velocity | VL (2) | RPDO or SDO |
| 0x6071 | target torque | PT (4), CST (10) | RPDO or SDO |
| 0x60C1:01 | interpolation data | IP (7) | RPDO or SDO |
| `home_switch_index:subindex` | home switch input | homing only | SDO (or TPDO) |

Modes are registered only if 0x6502 reports them. Homing mode (6) is registered but not used:
homing is done in `Cia402System` with velocity commands and a home switch (see below). For
channels 2 and 3, PP and homing are not supported.

## Recommended PDO mapping

All of these must fit in 8 bytes per PDO.

| PDO | Objects | Transmission |
|---|---|---|
| RPDO1 | 0x6040 controlword (16), 0x6060 mode (8) | `0x01` (synchronous) or `0xFF` |
| RPDO2 | target of the default mode, e.g. 0x60FF (32) | `0x01` or `0xFF` |
| TPDO1 | 0x6041 statusword (16), 0x6061 mode display (8) | `0x01` (every SYNC) |
| TPDO2 | 0x6064 position (32), 0x606C velocity (32) | `0x01` (every SYNC) |

With synchronous RPDOs the master sends the values on the next SYNC. With `0xFF`/`0xFE` it sends
them as soon as the driver writes them. Set master `sync_period` (µs) ≥ the driver `period` (ms).

## SDO-only operation

Disable all PDOs in `bus.yml`. The driver then accesses every object by SDO, each call with a
20 ms timeout:

- per `period` and channel: 2 uploads (0x6041, 0x6061) and 1-2 downloads (0x6040, target),
- per ros2_control `read()`: 2 uploads (0x6064, 0x606C) **in the control loop thread**.

This is fine for bring-up and bench tests. At 2 drives, 10 ms `period` and a 50 Hz controller,
expect roughly 1000 SDO transfers/s.

> **Limitation.** `LelyDriverBridge` decides between PDO and SDO from the **mapping entries** in
> the EDS/DCF (0x16xx/0x1Axx sub 1..8). It ignores the PDO valid bit and sub 0. If one of the
> objects above is still listed in the EDS mapping of a disabled PDO:
>
> - reads return a stale local cache,
> - writes go to a master PDO that does not exist.
>
> SDO-only works only if the EDS does not map these objects (e.g. `roboteq_example.eds` maps only
> vendor objects).

### PDO repair caveat

`Cia402System`'s `MotorManager` checks 0x1400-0x1403 and 0x1800-0x1803 sub 1 on every drive,
after start, after an NMT reset and when init keeps failing. It rewrites any COB-ID that is
disabled or not the CiA 301 default (`0x200/0x300/0x400/0x500 + id` for RPDO,
`0x180/0x280/0x380/0x480 + id` for TPDO). This fixes drives that lose their PDO config, but it
also means:

- PDOs disabled in `bus.yml` get **enabled again on the device**. The master still has no
  matching PDO, so it ignores them, but they add bus load, and a drive with an RPDO timeout may
  fault.
- Custom (non-default) COB-IDs are overwritten.

## Init and state handling

`init_motor(channel)` (called by `MotorManager`, or by the `~/<joint>/init` service):

1. Register the modes supported by the drive (0x6502, SDO).
2. Read the statusword and request a fault reset.
3. Walk the CiA 402 state machine to Operation Enabled. Each transition times out after
   `state_switch_timeout_ms`.
4. Select No_Mode. `set_default_operation_mode()` then switches to `default_operation_mode`: it
   goes through `switching_state`, writes 0x6060 and polls 0x6061 for up to 5 s.

While the drive is not in Operation Enabled, the controlword keeps the Halt bit set. The target
is only accepted in Operation Enabled.

## Configuration reference (`bus.yml`)

Common driver keys:

| Key | Default | Description |
|---|---|---|
| `driver` | – | `"ros2_canopen::Cia402Driver"` (or `LifecycleCia402Driver` for managed operation) |
| `package` | – | `"canopen_402_driver"` |
| `node_id`, `dcf` | – | node ID and EDS file |
| `period` | 10 | state machine cycle in ms (read statusword, write controlword/target) |
| `polling` | true | `false` runs the cycle on every SYNC instead of a timer |
| `state_switch_timeout_ms` | 1000 | timeout per CiA 402 transition. Keep it short: with an e-stop engaged, every failing transition costs this long |
| `boot` | true | master configures (downloads `.bin`) and starts the node |
| `revision_number`, `serial_number` | EDS | identity values checked at boot |
| `heartbeat_producer` | EDS | 0x1017 in ms, written during boot |
| `enable_ros_interfaces` | true | per-joint services and topics (see below) |
| `diagnostics.enable` / `.period` | false / 1000 | per-joint CiA 402 state, mode, init and communication status |
| `rpdo` / `tpdo` | EDS | PDO overrides (`enabled`, `cob_id`, `transmission`, `mapping`, `event_timer`, ...) downloaded during boot |
| `sdo` | – | extra `{index, sub_index, value}` writes downloaded during boot |

Per channel, under `motor_channels: {<1|2|3>: {...}}`:

| Key | Default | Description |
|---|---|---|
| `joint_name` | required | ros2_control joint name |
| `default_operation_mode` | 0 | mode set by `MotorManager` after init (e.g. 3 = PV, 9 = CSV, 1 = PP) |
| `switching_state` | 5 | CiA 402 state used for mode switches (5 = Operation Enabled, 4 = Switched On) |
| `position_conversion` | `linear` | `linear` or `table` |
| `scale_pos_to_dev` / `scale_pos_from_dev` | 1000 / 0.001 | linear position scales (raw = rad × to_dev) |
| `position_table_to_dev` / `position_table_from_dev` | – | `[[raw, rad], ...]` for `table` |
| `velocity_conversion` | `linear` | `linear` or `table` |
| `scale_vel_to_dev` / `scale_vel_from_dev` | 1000 / 0.001 | linear velocity scales |
| `velocity_table_to_dev` / `velocity_table_from_dev` | – | `[[raw, rad/s], ...]` for `table` |
| `homing_enabled` | false | allow `Cia402System`'s `~/home_joint` for this joint |
| `homing_fast_speed` / `homing_slow_speed` | 0 | approach velocities (rad/s); the sign sets the direction |
| `homing_backoff_speed` / `homing_backoff_time` | 0 / 0.5 | backoff away from the switch before each approach |
| `home_offset` | 0 | joint position reported at the switch |
| `home_switch_index` / `home_switch_subindex` / `home_switch_active_value` | 0 / 0 / 1 | u8 object that reads `active_value` when the switch triggers |
| `home_max_travel` | 2π | abort an approach after this travel |
| `homing_timeout` | 30 | total homing timeout in s |

Tables are `[raw, value]` points, strictly monotonic, interpolated piecewise linearly and
extrapolated beyond the outer points. A negative scale inverts a mirrored motor.

ROS interfaces per joint (`enable_ros_interfaces: true`): `~/<joint>/joint_states` and the
Trigger services `init`, `halt`, `recover`, `position_mode`, `velocity_mode`,
`cyclic_velocity_mode`, `cyclic_position_mode`, `interpolated_position_mode` and `torque_mode`,
plus the `COTargetDouble` service `target`. With `Cia402System` these are not needed; set
`enable_ros_interfaces: false`.

## Examples

### PDO (recommended)

Profile velocity drive, PDOs remapped by the master during boot.

```yaml
master:
  node_id: 1
  driver: "ros2_canopen::MasterDriver"
  package: "canopen_master_driver"
  sync_period: 10000              # us

drive_left:
  node_id: 6
  dcf: "drive.eds"
  driver: "ros2_canopen::Cia402Driver"
  package: "canopen_402_driver"
  period: 10
  state_switch_timeout_ms: 500
  boot: true
  heartbeat_producer: 100
  enable_ros_interfaces: false
  diagnostics: {enable: true, period: 1000}
  motor_channels:
    1:
      joint_name: rear_left_wheel
      default_operation_mode: 3   # profile velocity, target 0x60FF
      scale_pos_to_dev: -500000.0 # mirrored motor
      scale_pos_from_dev: -0.000002
      scale_vel_to_dev: -500000.0
      scale_vel_from_dev: -0.000002
  rpdo:
    1:
      enabled: true
      cob_id: "auto"
      transmission: 0x01
      mapping:
        - {index: 0x6040, sub_index: 0}   # controlword
        - {index: 0x6060, sub_index: 0}   # modes of operation
    2:
      enabled: true
      cob_id: "auto"
      transmission: 0x01
      mapping:
        - {index: 0x60FF, sub_index: 0}   # target velocity
  tpdo:
    1:
      enabled: true
      cob_id: "auto"
      transmission: 0x01
      mapping:
        - {index: 0x6041, sub_index: 0}   # statusword
        - {index: 0x6061, sub_index: 0}   # modes of operation display
    2:
      enabled: true
      cob_id: "auto"
      transmission: 0x01
      mapping:
        - {index: 0x6064, sub_index: 0}   # position actual value
        - {index: 0x606C, sub_index: 0}   # velocity actual value
```

### SDO only (bring-up)

Only valid if the EDS does not map the CiA 402 objects (see the limitation above). With
`Cia402System`, the PDOs will be re-enabled on the device (see the PDO repair caveat).

```yaml
drive_right:
  node_id: 7
  dcf: "roboteq_example.eds"
  driver: "ros2_canopen::Cia402Driver"
  package: "canopen_402_driver"
  period: 10
  state_switch_timeout_ms: 500
  revision_number: 0xFFFFFFFF
  enable_ros_interfaces: false
  motor_channels:
    1:
      joint_name: rear_right_wheel
      default_operation_mode: 3
      scale_pos_to_dev: 500000.0
      scale_pos_from_dev: 0.000002
      scale_vel_to_dev: 500000.0
      scale_vel_from_dev: 0.000002
  rpdo:
    1: {enabled: false}
    2: {enabled: false}
    3: {enabled: false}
  tpdo:
    1: {enabled: false}
    2: {enabled: false}
    3: {enabled: false}
```

### Two channels on one node, with homing

```yaml
steering_drive:
  node_id: 10
  dcf: "dual_channel.eds"
  driver: "ros2_canopen::Cia402Driver"
  package: "canopen_402_driver"
  period: 10
  motor_channels:
    1:
      joint_name: front_left_steering
      default_operation_mode: 3
      scale_pos_to_dev: 10000.0
      scale_pos_from_dev: 0.0001
      scale_vel_to_dev: 10000.0
      scale_vel_from_dev: 0.0001
      homing_enabled: true
      homing_fast_speed: 0.3
      homing_slow_speed: 0.05
      homing_backoff_speed: 0.1
      homing_backoff_time: 0.5
      home_offset: 0.0
      home_switch_index: 0x2101
      home_switch_subindex: 1
      home_switch_active_value: 1
      home_max_travel: 3.2
      homing_timeout: 30.0
    2:
      joint_name: front_right_steering    # uses 0x6840/0x6841/0x68FF/...
      default_operation_mode: 3
      scale_pos_to_dev: 10000.0
      scale_pos_from_dev: 0.0001
      scale_vel_to_dev: 10000.0
      scale_vel_from_dev: 0.0001
```
