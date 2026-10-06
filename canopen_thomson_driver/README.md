# canopen_thomson_driver

`ros2_canopen::ThomsonDriver` drives a Thomson linear actuator over CANopen. The actuator does
**not** implement CiA 402. It uses a vendor-specific object dictionary: one command frame
(target, current limit, speed, aux and a motion enable byte) and one feedback frame (position,
current, status, flags). The driver is used as a position joint in
[`canopen_ros2_control/Cia402System`](../canopen_ros2_control/README.md), next to CiA 402 drives
on the same bus.

It is built on the proxy driver, so it keeps the NMT/SDO services of
[`canopen_proxy_driver`](../canopen_proxy_driver/readme.md) (disable them with
`enable_ros_interfaces: false`).

## Requirements and capabilities

| Question | Answer |
|---|---|
| EDS needed? | Yes. The master needs the object dictionary and the PDO byte layout. `thomson_placeholder.eds` in `toro_control` was reverse engineered from a CAN dump. Replace it with the vendor EDS when available. |
| Identity check at boot | With `boot: true` the master compares the device's 0x1000 and 0x1018 against the EDS values. If they differ, the boot fails. The placeholder uses vendor ID `0x4D5` and product/revision/serial `0`. |
| SDO only? | Yes for **commands and feedback**, if the objects are not mapped in the EDS. See [SDO-only operation](#sdo-only-operation). |
| PDO operation | Recommended: RPDO1 synchronous (`transmission: 0x01`) for the command and TPDO1 for the feedback. |
| Who configures the PDOs on the device? | The master, but only when the device accepts it (`boot: true`). During boot the master downloads everything in the generated `<name>.bin`: PDO COB-IDs, transmission types, mappings given in `bus.yml`, heartbeat time and the `sdo:` list. If the device does not accept writes to 0x14xx/0x16xx/0x18xx/0x1Axx, its factory PDO layout must match the EDS exactly. |
| NMT start | The driver sends NMT Start itself: in `activate()`, after every boot-up, when the actuator comes online and while it keeps reporting motion disabled. The master's boot process cannot be relied on for this, it aborts when the actuator rejects part of the configuration download. |
| Auto init after power loss | Yes (e.g. e-stop). Recovery cycle in the driver: while the actuator does not answer it is NMT reset every `offline_reset_interval_ms`; once it answers (or sends a boot-up) it gets NMT Start and a new enable sequence; if it then keeps reporting motion disabled, NMT Start and re-arm repeat every second and after 5 failed attempts the node is reset. Feedback is read by SDO, which works in PRE-OPERATIONAL too. |
| Auto enable | Yes. While `Cia402System` is active and the actuator reports feedback without a fault, the driver runs the 1 → 0 → 1 enable sequence on its own. No service call is needed. |
| Heartbeat | Optional. `heartbeat_producer` in `bus.yml` sets 0x1017 on the device. The driver does not use heartbeats itself: it detects a dead actuator through `feedback_timeout_ms`. |
| Absolute position | Yes. The actuator reports an absolute position, so no homing or offset handling is needed (`enable_position_offset` is ignored for it). |

## Protocol

Default objects, all overridable in `bus.yml` as `[index, subindex]`:

| Direction | Key | Default | Type | Content |
|---|---|---|---|---|
| command | `cmd_position_object` | `0x2100:0` | u16 | target position (raw) |
| command | `cmd_current_limit_object` | `0x2101:0` | u16 | `current_limit` |
| command | `cmd_speed_object` | `0x2102:0` | u8 | `speed` |
| command | `cmd_aux_object` | `0x2103:0` | u8 | low byte of `aux` |
| command | `cmd_aux2_object` | `0x2104:0` | u8 | high byte of `aux` |
| command | `cmd_enable_object` | `0x2105:0` | u8 | motion enable (0/1) |
| feedback | `fb_position_object` | `0x2200:0` | u16 | actual position (raw) |
| feedback | `fb_current_object` | `0x2201:0` | u16 | actual current (raw) |
| feedback | `fb_status_object` | `0x2204:0` | u8 | status byte |
| feedback | `fb_flags_object` | `0x2205:0` | u8 | flags byte (`rearm_flag_mask`, `fault_flag_mask`) |

The object sizes are fixed in the driver and must match the mapping (16/16/8/8/8/8 bits for the
command, 8 bytes in total).

Every `period` ms the driver:

1. reads the feedback objects (PDO cache if mapped in a TPDO, SDO otherwise),
2. advances the enable state machine,
3. writes all six command objects. If they are mapped in an RPDO, the master sends them together.
   With `transmission: 0x01` that happens on the next SYNC. Otherwise each is a separate SDO
   download.

### Enable sequence

Motion only starts after the enable byte goes **1 → 0 → 1**. Each step lasts
`enable_step_cycles` periods. Until the sequence has finished, the driver commands the actual
position, so enabling never makes the actuator jump. The sequence runs again when:

- the actuator (re)boots: NMT boot-up, reported as `START`,
- feedback returns after a timeout,
- the actuator sets `rearm_flag_mask` in the flags byte while enabled (at most once per second).

`disable()` (on deactivate, or while any CiA 402 drive on the bus is not operational) sets the
enable byte to 0 and drops the target, so the actuator holds its position.

### Position conversion

The joint value is in rad. It is converted with
[`ValueConversion`](../canopen_base_driver/include/canopen_base_driver/value_conversion.hpp):

- `position_conversion: linear` (default): `raw = center_position + rad * scale_pos_to_dev`.
- `position_conversion: table`: piecewise linear interpolation in
  `position_table_to_dev` (command) and `position_table_from_dev` (feedback), both
  `[[raw, rad], ...]`. The points must be strictly monotonic, and the outer segments are
  extrapolated. Use this mode when the actuator drives a nonlinear linkage.

The command is always clamped to `[min_raw, max_raw]`. The velocity state is differentiated
from consecutive position samples.

## SDO-only operation

The driver works with `tpdo: {1: {enabled: false}}` and/or `rpdo: {1: {enabled: false}}`. Feedback
is then polled and commands are written by SDO. At a 20 ms `period` this costs 4 SDO uploads and
6 SDO downloads per cycle, each with a 20 ms timeout.

> **Limitation.** `LelyDriverBridge` decides between PDO and SDO from the **mapping entries** in
> the EDS/DCF (0x16xx/0x1Axx sub 1..8). It ignores the PDO valid bit and sub 0. If an object is
> still listed in the EDS mapping of a disabled PDO:
>
> - reads return the stale local cache and never reach the device (the position stays at 0),
> - writes go to a master PDO that does not exist.
>
> For true SDO-only operation, remove the objects from the EDS mapping, or keep that PDO enabled.

## Configuration reference (`bus.yml`)

Common driver keys:

| Key | Default | Description |
|---|---|---|
| `driver` | – | `"ros2_canopen::ThomsonDriver"` |
| `package` | – | `"canopen_thomson_driver"` |
| `node_id`, `dcf` | – | node ID and EDS file |
| `period` | 10 | command/feedback cycle in ms; also the unit of `enable_step_cycles` |
| `polling` | true | `false` runs the cycle on every SYNC instead of a timer |
| `boot` | true | master configures (downloads `.bin`) and starts the node |
| `heartbeat_producer` | EDS | 0x1017 in ms, written during boot |
| `enable_ros_interfaces` | true | proxy driver topics/services (NMT, SDO, PDO) |
| `diagnostics.enable` / `.period` | false / 1000 | diagnostics with joint, enable state, raw position/current, status, flags |
| `rpdo` / `tpdo` | EDS | PDO overrides (`enabled`, `cob_id`, `transmission`, `mapping`, `event_timer`, ...) downloaded during boot |
| `sdo` | – | extra `{index, sub_index, value}` writes downloaded during boot |

Thomson-specific keys:

| Key | Default | Description |
|---|---|---|
| `joint_name` | required | ros2_control joint name |
| `position_conversion` | `linear` | `linear` or `table` |
| `center_position` | 900 | raw value at 0 rad (linear) |
| `scale_pos_to_dev` | required for linear | raw per rad (linear); the reverse scale is `1 / scale_pos_to_dev` |
| `position_table_to_dev` / `position_table_from_dev` | required for table | `[[raw, rad], ...]` |
| `min_raw` / `max_raw` | 0 / 0xFFFF | clamp for the raw command |
| `current_limit` | 0xDC | sent in every command frame |
| `speed` | 0x32 | sent in every command frame |
| `aux` | 0x00F0 | 16-bit value, split into `cmd_aux_object` (low) and `cmd_aux2_object` (high) |
| `enable_step_cycles` | 3 | periods per step of the 1 → 0 → 1 sequence (min 1) |
| `feedback_timeout_ms` | 1000 | no feedback for this long = communication failure |
| `rearm_flag_mask` | 0x01 | flags bit(s) meaning "motion not enabled", triggers a re-arm |
| `fault_flag_mask` | 0x00 | flags bit(s) meaning fault; blocks enabling |
| `cmd_*_object` / `fb_*_object` | see [Protocol](#protocol) | `[index, subindex]` overrides |

## Examples

### PDO (recommended)

The command goes as synchronous RPDO1 on every SYNC. The feedback comes as TPDO1, sent by the
actuator after every SYNC.

```yaml
master:
  node_id: 1
  driver: "ros2_canopen::MasterDriver"
  package: "canopen_master_driver"
  sync_period: 20000            # us

steering:
  node_id: 35
  dcf: "thomson.eds"
  driver: "ros2_canopen::ThomsonDriver"
  package: "canopen_thomson_driver"
  period: 20
  boot: true
  enable_ros_interfaces: false
  heartbeat_producer: 500
  diagnostics: {enable: true, period: 1000}

  joint_name: virtual_front_steering
  position_conversion: linear
  center_position: 880
  scale_pos_to_dev: 917.0
  min_raw: 160
  max_raw: 1600
  current_limit: 0xDC
  speed: 0x32
  aux: 0x00F0
  enable_step_cycles: 3
  feedback_timeout_ms: 1000
  rearm_flag_mask: 0x01
  fault_flag_mask: 0x00

  rpdo:
    1: {enabled: true, cob_id: "auto", transmission: 0x01}   # master sends on every SYNC
  tpdo:
    1: {enabled: true, cob_id: "auto", transmission: 0x01}   # actuator answers every SYNC
  sdo:                                                       # vendor setup, sent during boot
    - {index: 0x2005, sub_index: 0, value: 5000}
    - {index: 0x2006, sub_index: 0, value: 100}
    - {index: 0x2009, sub_index: 0, value: 0xFFFF}
    - {index: 0x2013, sub_index: 0, value: 1}
```

### SDO feedback, PDO command (current toro setup)

```yaml
steering:
  # ... as above ...
  rpdo:
    1: {enabled: true, cob_id: "auto", transmission: 0x01}
  tpdo:
    1: {enabled: false}   # feedback polled by SDO - see the limitation above
```

### Nonlinear steering linkage

```yaml
steering:
  # ...
  position_conversion: table
  # Same sensor for command and feedback, so both tables share the points.
  position_table_to_dev: &steering_table [[160, -0.785], [520, -0.38], [880, 0.0], [1240, 0.38], [1600, 0.785]]
  position_table_from_dev: *steering_table
  min_raw: 160
  max_raw: 1600
```

### URDF

The joint name must match `joint_name`. Only the `position` command interface is used.

```xml
<joint name="virtual_front_steering">
  <command_interface name="position">
    <param name="min">-0.4</param>
    <param name="max">0.4</param>
  </command_interface>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```
