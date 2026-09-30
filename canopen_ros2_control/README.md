# canopen_ros2_control

ros2_control hardware interfaces for CANopen buses. The package exports three plugins:

| Plugin | Status |
|---|---|
| `canopen_ros2_control/Cia402System` | **Supported.** The only system documented here. |
| `canopen_ros2_control/CanopenSystem` | Deprecated. It is the base class of `Cia402System`; do not use it as a plugin. |
| `canopen_ros2_control/RobotSystem` | Deprecated. |

## Cia402System

`Cia402System` runs a complete CANopen stack inside the ros2_control hardware component:

- the master,
- one driver per node from `bus.yml`,
- a motor manager thread that brings the drives up and keeps them up.

It exposes the joints to ros2_control and needs no service calls or lifecycle transitions of its
own.

### Supported drivers

| Driver (`bus.yml` `driver`) | Device | Joints | Docs |
|---|---|---|---|
| `ros2_canopen::Cia402Driver` | CiA 402 motor controllers, 1-3 channels per node | one per channel, position/velocity (torque see below) | [canopen_402_driver](../canopen_402_driver/README.md) |
| `ros2_canopen::ThomsonDriver` | Thomson linear actuator, vendor-specific PDOs | one, position | [canopen_thomson_driver](../canopen_thomson_driver/README.md) |

Any other driver type in `bus.yml` is loaded but ignored by `Cia402System`, with an error in
the log.

### How it works

```
controller_manager ── read()/write() ──► Cia402System
                                            │  (never blocks on the bus in write())
                                            ├── MotorManager thread: init, fault reset, mode switch,
                                            │                        PDO repair, NMT reset (CiA 402 only)
                                            └── DeviceContainer (own executor thread)
                                                 ├── MasterDriver  (lely master: SYNC, NMT, boot)
                                                 ├── Cia402Driver  (per node, poll timer = period)
                                                 └── ThomsonDriver (per node, poll timer = period)
```

**on_configure** loads the master and all drivers from `bus.yml`, then adds each node to the
master. A node that is not ready yet is booted: identity check, `.bin` config download, NMT
Start. A failing driver init is retried 5 × 2 times, with an NMT Reset Node between the blocks.
After that, configure fails.

**on_activate** returns immediately, without touching the bus, even with the e-stop engaged.
It then starts `MotorManager`, which services the CiA 402 motors one after another:

| Motor health | Action | Retry |
|---|---|---|
| communication failure | NMT Reset Node | at most every 10 s |
| uninitialized | check/repair PDO COB-IDs, then init (fault reset, Operation Enabled), then default mode | forever, every 100 ms pass |
| faulty (e-stop, over-current, ...) | fault reset and re-enable; re-check PDOs after 5 failures | forever |
| wrong mode | switch to `default_operation_mode`; full re-init after 10 failures | forever |

**write()** only forwards commands while *all* CiA 402 motors are operational and all Thomson
actuators report feedback without a fault. Otherwise it sets every drive to target 0 and
disables every actuator, which then holds its position. The command interface used depends on
each motor's active mode: position for PP/CSP/IP, velocity for PV/VL/CSV, effort for PT/CST.
The actuators are enabled every cycle; the driver runs their enable sequence and holds the
actual position until it is done. An actuator gets no target until a controller writes one.

**read()** reports position and velocity. A CiA 402 node with a communication failure, or an
actuator whose feedback timed out, reports 0 for its joints.

### Requirements and capabilities (summary)

| Topic | CiA 402 drive | Thomson actuator |
|---|---|---|
| EDS / identity check at boot | required / yes | required / yes |
| SDO-only possible | yes, slow; objects must not be mapped in the EDS | yes; objects must not be mapped in the EDS |
| PDOs configured by the master | yes, during boot (`boot: true`) from `bus.yml` | yes, during boot (`boot: true`) from `bus.yml` |
| PDOs changed at runtime | `MotorManager` re-enables RPDO/TPDO 1-4 with default COB-IDs | no |
| NMT Start | master after boot | master after boot, plus the driver in `activate()` |
| Re-init after power loss | master reboots the node; `MotorManager` re-inits | master reboots the node; driver re-arms the enable sequence |
| Enable without service calls | yes | yes |
| Communication loss detection | SDO errors (not seen in PDO-only reads) | feedback timeout |
| Homing / offsets | optional, via services below | not needed (absolute) |

Details, caveats and the full per-driver options are in the driver READMEs linked above.

### URDF parameters

Hardware parameters:

| Parameter | Default | Description |
|---|---|---|
| `bus_config` | required | path to `bus.yml` (the installed copy) |
| `master_config` | required | path to the generated `master.dcf` |
| `master_bin` | – | path to the generated `master.bin`; the literal value `""` (two quote characters) means none |
| `can_interface_name` | required | SocketCAN interface, e.g. `can0` |
| `position_offset_file` | `~/offsets.txt` | where joint offsets are persisted |
| `cold_start_threshold` | 0.1 | rad. At startup, if a raw position differs from the saved one by more than this, the drive is assumed to have lost its position (cold start) and the offset is recomputed |

Joint parameters:

| Parameter | Default | Description |
|---|---|---|
| `enable_position_offset` | false | CiA 402 only: keep a persisted position offset (see below). Ignored for Thomson actuators |
| `node_id` | – | optional: also export the raw CANopen interfaces of that node (`nmt/state`, `rpdo/*` state; `nmt/reset`, `nmt/start`, `tpdo/*` commands) |

Joints are matched **by name**: a URDF joint must have the same name as a `joint_name` in
`bus.yml`. The URDF has no device or node parameter.

Interfaces per joint: state `position`, `velocity`; command `position`, `velocity`, `effort`.
Declare only the ones your controller uses.

> Torque modes: `Motor402::setTarget` does not convert or pass a value for PT/CST, so the
> `effort` command is not usable yet.

### Position offsets and homing

Some drives reset their encoder to 0 on power loss. For joints with
`enable_position_offset: true`:

- `Cia402System` reports `raw + offset` and saves `raw offset` to `position_offset_file` every
  0.5 s.
- At startup (once all motors are operational), it compares the current raw position with the
  saved one: within `cold_start_threshold` it is a warm restart and the saved offset is kept.
  Otherwise the offset is recomputed, so the joint position is unchanged.

Services, on node `/cia402_system_services`:

| Service | Type | Description |
|---|---|---|
| `~/reset_position_home` | `std_srvs/Trigger` | current position becomes 0 for all offset-enabled joints |
| `~/adjust_position_offset` | `canopen_ros2_control/AdjustPositionOffset` | add `offset_delta` to one joint's offset |
| `~/home_joint` | `canopen_ros2_control/HomeJoint` | blocking homing of one joint (`homing_enabled` in `bus.yml`). Sequence: backoff, fast approach to the home switch, backoff, slow approach; then the offset is set so the joint reports `home_offset`. While any joint homes, all other motors are held at velocity 0 |

### Build: generating the DCFs

The configuration package generates `master.dcf`, `master.bin` and one `<node>.bin` per slave
from `bus.yml` at build time. `dcfgen -r` also generates the matching master-side PDOs:

```cmake
find_package(lely_core_libraries REQUIRED)
generate_dcf(canopen)   # config/canopen/{bus.yml, *.eds}
```

The slave `.bin` files are what the master downloads during boot. Changes to `rpdo`/`tpdo`/`sdo`/
`heartbeat_producer` only reach the device if the device boots with `boot: true`.

## Full example

Bus with two CiA 402 wheel drives (PDO) and a Thomson steering actuator, as on the toro.

### bus.yml

```yaml
options:
  dcf_path: "@BUS_CONFIG_PATH@"
  boot_timeout_ms: 30000          # per node, covers reading large EDS files on a cold boot

master:
  node_id: 1
  driver: "ros2_canopen::MasterDriver"
  package: "canopen_master_driver"
  sync_period: 20000              # us

drive_left:
  node_id: 6
  dcf: "drive.eds"
  driver: "ros2_canopen::Cia402Driver"
  package: "canopen_402_driver"
  period: 10
  state_switch_timeout_ms: 500
  enable_ros_interfaces: false
  diagnostics: {enable: true, period: 1000}
  motor_channels:
    1:
      joint_name: rear_left_wheel
      default_operation_mode: 3   # profile velocity
      scale_pos_to_dev: -500000.0
      scale_pos_from_dev: -0.000002
      scale_vel_to_dev: -500000.0
      scale_vel_from_dev: -0.000002
  rpdo:
    1: {enabled: true, cob_id: "auto", transmission: 0x01,
        mapping: [{index: 0x6040, sub_index: 0}, {index: 0x6060, sub_index: 0}]}
    2: {enabled: true, cob_id: "auto", transmission: 0x01,
        mapping: [{index: 0x60FF, sub_index: 0}]}
  tpdo:
    1: {enabled: true, cob_id: "auto", transmission: 0x01,
        mapping: [{index: 0x6041, sub_index: 0}, {index: 0x6061, sub_index: 0}]}
    2: {enabled: true, cob_id: "auto", transmission: 0x01,
        mapping: [{index: 0x6064, sub_index: 0}, {index: 0x606C, sub_index: 0}]}

drive_right:
  node_id: 7
  dcf: "drive.eds"
  driver: "ros2_canopen::Cia402Driver"
  package: "canopen_402_driver"
  period: 10
  state_switch_timeout_ms: 500
  enable_ros_interfaces: false
  diagnostics: {enable: true, period: 1000}
  motor_channels:
    1:
      joint_name: rear_right_wheel
      default_operation_mode: 3
      scale_pos_to_dev: 500000.0
      scale_pos_from_dev: 0.000002
      scale_vel_to_dev: 500000.0
      scale_vel_from_dev: 0.000002
  rpdo:
    1: {enabled: true, cob_id: "auto", transmission: 0x01,
        mapping: [{index: 0x6040, sub_index: 0}, {index: 0x6060, sub_index: 0}]}
    2: {enabled: true, cob_id: "auto", transmission: 0x01,
        mapping: [{index: 0x60FF, sub_index: 0}]}
  tpdo:
    1: {enabled: true, cob_id: "auto", transmission: 0x01,
        mapping: [{index: 0x6041, sub_index: 0}, {index: 0x6061, sub_index: 0}]}
    2: {enabled: true, cob_id: "auto", transmission: 0x01,
        mapping: [{index: 0x6064, sub_index: 0}, {index: 0x606C, sub_index: 0}]}

steering:
  node_id: 35
  dcf: "thomson.eds"
  driver: "ros2_canopen::ThomsonDriver"
  package: "canopen_thomson_driver"
  period: 20
  boot: true
  heartbeat_producer: 500
  enable_ros_interfaces: false
  diagnostics: {enable: true, period: 1000}
  joint_name: virtual_front_steering
  center_position: 880
  scale_pos_to_dev: 917.0
  min_raw: 160
  max_raw: 1600
  current_limit: 0xDC
  speed: 0x32
  rpdo:
    1: {enabled: true, cob_id: "auto", transmission: 0x01}
  tpdo:
    1: {enabled: true, cob_id: "auto", transmission: 0x01}
```

### ros2_control URDF

```xml
<ros2_control name="toro" type="system">
  <hardware>
    <plugin>canopen_ros2_control/Cia402System</plugin>
    <param name="bus_config">$(find toro_control)/config/canopen/bus.yml</param>
    <param name="master_config">$(find toro_control)/config/canopen/master.dcf</param>
    <param name="master_bin">""</param>
    <param name="can_interface_name">can0</param>
    <param name="position_offset_file">~/offsets.txt</param>
    <param name="cold_start_threshold">0.1</param>
  </hardware>

  <joint name="virtual_front_steering">      <!-- ThomsonDriver -->
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="rear_right_wheel">            <!-- Cia402Driver, profile velocity -->
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="rear_left_wheel">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

Use the paths of the installed configuration package (`bus.yml` and the generated
`master.dcf`), as `toro_control/launch/toro_control_launch.xml` does.
