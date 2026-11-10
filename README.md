# dynamixel_hardware_interface (kid_rl fork)

> **This is an unofficial, modified fork of [ROBOTIS-GIT/dynamixel_hardware_interface](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface)**, based on the `jazzy` branch (1.5.2).
> ROBOTIS does not maintain or endorse it. DYNAMIXEL is a trademark of ROBOTIS.
> The original code is licensed under the Apache License 2.0 (see [LICENSE](LICENSE)), and the modifications are released under the same license.

This package is the hardware interface used to run **[kid_rl](https://github.com/parkgeon1202/kid_rl)**, a `ros2_control` controller that runs a reinforcement-learning walking policy on a kid-size humanoid. The robot-side setup and configuration are documented in kid_rl.

Every new feature is **opt-in** through a hardware parameter or a URDF/xacro entry. If you set none of them, the driver behaves like upstream, apart from the bug fixes.

---

## Contents

1. [Installation](#1-installation)
2. [Changes from upstream](#2-changes-from-upstream)
3. [Interfaces added by this fork](#3-interfaces-added-by-this-fork)
4. [kid_rl branches that use this fork](#4-kid_rl-branches-that-use-this-fork)
5. [Extra dynamixel_interfaces messages for the IRC competition](#5-extra-dynamixel_interfaces-messages-for-the-irc-competition)
6. [Bug fixes](#6-bug-fixes)
7. [Upstream documentation](#7-upstream-documentation)

---

## 1. Installation

```bash
cd ~/${WORKSPACE}/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
git clone -b kid-rl https://github.com/parkgeon1202/dynamixel_hardware_interface.git
cd ~/${WORKSPACE}
colcon build --packages-up-to dynamixel_hardware_interface
source install/setup.bash
```

Only the hardware interface is forked. DynamixelSDK and dynamixel_interfaces are the ROBOTIS packages, and this driver builds against them unchanged. The fork was developed and tested on ROS 2 Jazzy.

The kid_rl competition branches (`for_irc_contest`, `irc_hurdle`) additionally need five messages added to `dynamixel_interfaces`. The driver does not use them. For those branches, clone the [dynamixel_interfaces fork](https://github.com/parkgeon1202/dynamixel_interfaces/tree/kid-rl) instead of the ROBOTIS one:

```bash
git clone -b kid-rl https://github.com/parkgeon1202/dynamixel_interfaces.git
```

See [§5](#5-extra-dynamixel_interfaces-messages-for-the-irc-competition).

---

## 2. Changes from upstream

For each change: its purpose, how it is implemented, how to enable it, and where kid_rl uses it.

| # | Change | Purpose | Implementation | How to enable | Use in kid_rl |
|---|---|---|---|---|---|
| ① | **Input Voltage Error is logged, not treated as a fault** | When the robot walks, the battery voltage dips for a moment and the servos raise Hardware Error bit 0 (Input Voltage). Upstream treats this as a fault and goes to Reboot or ERROR. Torque then drops during walking, or the servos keep rebooting. | Added `kInputVoltageErrorMask = 0x01` to `dynamixel.hpp`. When bit 0 is the **only** bit set:<br>• `InitDxlComm`: continues without rebooting.<br>• `WriteItem`: checks with `IsOnlyInputVoltageAlert()`, then accepts the write as successful.<br>• `CheckError`: logs a warning at most once per second and leaves the bit out of the fault decision.<br>If any other bit (overheating, overload, ...) is set as well, it is a fault, as upstream. | Always on | The policy keeps running through voltage dips. Voltage is recorded separately in the CSV `vin_*` columns, and errors in the `hwerr_*` columns. |
| ② | **Switch to turn Fast Sync Read on or off** | Fast Sync Read receives the replies of all servos in one packet, so if a single servo fails, the whole cycle is lost. On top of that, once it has succeeded, `fast_read_permanent_` is set and the driver never falls back to plain SyncRead, even when it fails later. | Added `SetFastReadProtocol()`, set from a hardware parameter. | `use_fast_read_protocol` (default `true`) | Set to `false` in `kid_dynamixel.ros2_control.xacro`, so it uses plain SyncRead, where each servo replies separately. |
| ③ | **Per-servo reception tracking:** `SyncReadRxTracker` + `rx_missing_id` (2 new files) | The SDK's `GroupSyncRead::rxPacket()` returns at the first failure, so it cannot tell **which servo's reply was missing**. | • `sync_read_rx_tracker.cpp` receives every packet that arrives before the timeout, and records for each ID whether it arrived, plus the number of CRC-corrupt packets. Its buffers are allocated in advance, and it allocates no memory while receiving.<br>• Failure logs print `[missing IDs: ...] [corrupt packets: n]`.<br>• Each joint exports an `rx_missing_id` state interface: the ID of the missing servo, or −1 if everything arrived.<br>• Upstream pinged every servo one by one in the control thread whenever a read failed. That stalled the loop for 870 ms when the bus was down, so it was removed. | `track_sync_read_rx` (default off; when off, the original path is used unchanged) | `rx_missing_monitor.cpp` reads this value and records it in the CSV `rx_missing_count` and `rx_missing_ids` columns. |
| ④ | **Virtual gpio for bus timing measurement** | To measure how many ms old the joint readings are, how long a read took, and whether a write actually went out. | • A virtual gpio that is not a servo (`SetupBusTiming`) exports 7 values: `read_ms`, `read_ok`, `read_ok_start_s`, `read_ok_end_s`, `write_sent`, `write_ms`, `write_drain_ms`.<br>• Times are based on `steady_clock`, so the controller can subtract them directly from its own clock.<br>• This gpio is skipped during ping and register initialisation.<br>• With `measure_write_drain` on, the driver finds the port's fd in `/proc/self/fd` (`FindPortFd`) and uses `tcdrain()` to measure until transmission has actually finished. | A gpio with `type=bus_timing`; optional `measure_write_drain` | `bus_timing_monitor.cpp` computes `joint_age_min_ms` / `joint_age_max_ms` and records them in the CSV together with the `read_ms`, `read_ok` and `prev_write_*` columns. |
| ⑤ | **Limit on read reply wait time** | Upstream waits for replies for the whole control period (20 ms). If even one reply was missing, that cycle overran its period and the next tick was skipped. | The control-loop SyncRead uses `read_timeout_ms` as its wait limit instead of the period. | `read_timeout_ms` (`0` = upstream behaviour) | Set to 14 ms in the xacro. |
| ⑥ | **Send the write even when the read failed** | Upstream skips the write when the read ends in a communication error (`COMM_ERROR`), so the new policy target did not reach the servos for about 60 ms. SyncWrite does not wait for replies, so it can be sent regardless of the read failure. | `write()` also sends commands when the status is `COMM_ERROR`. | `write_on_read_failure` | Set to `true`. The target joint angles from the policy go out every tick. |
| ⑦ | **Write only when the command changed + detect write failures** | Reduce bus load, and stop commands from silently not reaching the servos. | • With `MultiDxlCommandChanged()` and `RememberMultiDxlCommands()`, SyncWrite is skipped when the values are the same as the last ones sent.<br>• In upstream, a failure that happens only on transmit is not caught by `dxl_status_`, so commands silently stopped. Now the failure time is accumulated, and once it exceeds `err_timeout_ms`, ERROR is returned. | Always on | Checked with the CSV `prev_write_sent` column. When the target stays the same, as in SAFE_HOLD, it shows 0. |
| ⑧ | **Torque off on error and shutdown** (`on_error`, `on_shutdown`) | In upstream, when read/write ends in ERROR, `on_deactivate` is never called. The servos then **keep their last goal position with torque on**. This is a robot safety problem. | Both cases turn torque off with `stop()`. Because `on_error` returns SUCCESS, the component goes to UNCONFIGURED and can be brought back without restarting the process. | Always on | The robot goes limp instead of holding its last pose when the bus fails or the controller manager shuts down. |
| ⑨ | **Initial value of gpio commands taken from the param** (`InitialCommandValue`) | When `Position P Gain` is opened as a command interface, the first SyncWrite in `start()` writes **P gain 0** to the servos before the controller has written a value. | If there is a `<param>` with the same name, the command starts from that value. Otherwise it starts from 0, as before. Goal Position is not affected, because `start()` overwrites it with the present position. | A `<param>` with the same name as the command interface | `configure_gains()` in `kid_rl_controller.cpp` uses this command interface to switch gains: the trained gain while walking, and `motion_gain.p_gains` during motions such as SAFE_HOLD. In the xacro the initial value is given with `<param name="Position P Gain">${kp}</param>`. |
| ⑩ | **Bug fixes in upstream** | See [Bug fixes](#6-bug-fixes) below. | | No configuration needed | Not specific to kid_rl; these could be sent upstream as pull requests. |

---

## 3. Interfaces added by this fork

| Where | Name | Type | Enabled by |
|---|---|---|---|
| `<hardware>` | `use_fast_read_protocol` | param (`true`/`false`) | — |
| `<hardware>` | `track_sync_read_rx` | param (`true`/`false`) | — |
| `<hardware>` | `read_timeout_ms` | param (ms, `0` = upstream) | — |
| `<hardware>` | `write_on_read_failure` | param (`true`/`false`) | — |
| `<hardware>` | `measure_write_drain` | param (`true`/`false`) | `bus_timing` gpio |
| `<joint>` | `rx_missing_id` | state interface | `track_sync_read_rx` |
| `<gpio>` with `type=bus_timing` | `read_ms`, `read_ok`, `read_ok_start_s`, `read_ok_end_s`, `write_sent`, `write_ms`, `write_drain_ms` | state interfaces | — |
| `<gpio>` with `type=dxl` | any command interface with a matching `<param>` | initial command value | — |

---

## 4. kid_rl branches that use this fork

Every [kid_rl](https://github.com/parkgeon1202/kid_rl) branch runs on this fork. They differ in which changes they turn on and in whether they talk to other competition packages.

| kid_rl branch | Robot | Changes used (§2) | Extra messages (§5) |
|---|---|---|---|
| `main` | Dance robot, 25 servos, gamepad | ①–⑧. P gains are fixed by the xacro `<param>`, so ⑨ is not used | No |
| `irc` | IRC robot, 12 leg servos, gamepad | ①–⑧ | No |
| `for_irc_contest` | IRC robot, 21 servos (12 legs + 9 upper body). Walking commands, motions, manipulation and camera tilt come from other competition packages | ①–⑨. ⑨ lets kid_rl switch P gains between walking and motions | Yes |
| `irc_hurdle` | As `for_irc_contest`, with a hurdle-vision policy | ①–⑨ | Yes |

From `for_irc_contest` on, kid_rl replaces the team's previous DYNAMIXEL node. It is the only process on the servo bus, and the walking policy, scripted motions and manipulation all go through this driver. The other packages reach the servos only through kid_rl's topics, which use the messages in §5.

---

## 5. Extra dynamixel_interfaces messages for the IRC competition

These five messages are **not** part of [ROBOTIS-GIT/dynamixel_interfaces](https://github.com/ROBOTIS-GIT/dynamixel_interfaces), and this driver does not use them. They are published in the fork [parkgeon1202/dynamixel_interfaces](https://github.com/parkgeon1202/dynamixel_interfaces/tree/kid-rl) (branch `kid-rl`). They were added to `dynamixel_interfaces` so that kid_rl (`for_irc_contest`, `irc_hurdle`) can exchange motor commands and motor states with the other IRC competition packages.

They keep the fields of the messages that the team's previous DYNAMIXEL node used, and add a servo `id` to every motor element. The previous node matched motors by their position in the array. kid_rl matches them by `id` instead, using the `ID` of each `<joint>_dxl` gpio in `robot_description`. So the array order does not matter, a message may name only some motors, and a message that contains an unknown or duplicate `id` or a non-finite value is dropped as a whole.

| Topic | Message | From → To | Purpose |
|---|---|---|---|
| `/dynamixel_control` | `DynamixelControlMsgs` | motion_operator, humanoid_manipulation → kid_rl | Goal positions (with profile velocity / acceleration) for scripted motions and manipulation. Upper-body servos follow it in every mode except FAULT. Leg servos follow it only while the walking policy is stopped |
| `/gripper_control` | `DynamixelMsgs` | humanoid_manipulation → kid_rl | Gripper (ID 6) goal position |
| `/RealsensePanTilt` | `PanTilt` | irc_vision `pan_tilt` node → kid_rl | Camera tilt (ID 23) in degrees. There is no pan motor, so `pan` is ignored |
| `/motor_status` | `DynamixelStateMsgs` | kid_rl → motion_operator, humanoid_manipulation | Position and velocity of every servo at 50 Hz, stamped with the control tick that read them |

Profile velocity and acceleration are not written to the servos. kid_rl emulates them every tick, because the walking policy was trained with Profile 0 and the servo registers must stay at 0.

### Definitions

The fork contains these files in `msg/` (comments shortened here):

```
# DynamixelMsgs.msg — command for one motor
std_msgs/Header header
uint8 id                        # DYNAMIXEL ID
float32 goal_position           # rad
float32 profile_acceleration    # rad/s^2, 0 = no limit
float32 profile_velocity        # rad/s,   0 = no limit
```

```
# DynamixelControlMsgs.msg — commands for several motors, matched by id
DynamixelMsgs[] motor_control
```

```
# PanTilt.msg — camera pan/tilt targets [degree]
int32 pan
int32 tilt
```

```
# DynamixelMotorState.msg — state of one motor
uint8 id                # DYNAMIXEL ID
float64 position        # rad
float64 velocity        # rad/s
```

```
# DynamixelStateMsgs.msg — state of several motors (/motor_status)
std_msgs/Header header  # stamp = the control tick that read these states
DynamixelMotorState[] motor_state
```

and lists them in `rosidl_generate_interfaces()` in its `CMakeLists.txt`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DynamixelState.msg"
  "msg/DynamixelMsgs.msg"
  "msg/DynamixelControlMsgs.msg"
  "msg/PanTilt.msg"
  "msg/DynamixelMotorState.msg"
  "msg/DynamixelStateMsgs.msg"
  "srv/GetDataFromDxl.srv"
  "srv/SetDataToDxl.srv"
  "srv/RebootDxl.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
```

kid_rl, motion_operator and humanoid_manipulation must all be built against the same definitions.

---

## 6. Bug fixes

These fixes are not specific to kid_rl, and need no configuration.

- **Memory leak in Sync/Bulk Write.** `SetDxlValueToSyncWrite()` and `SetDxlValueToBulkWrite()` allocated `new uint8_t[]` for every servo on every cycle and never freed it. `addParam()` copies the buffer, so these are now `std::vector`s. At 50 Hz with 21 servos, the leak grew without limit during long runs.
- **Write path stuck after one failed `addParam()`.** The ID was left in the group, so every later `addParam()` for it failed too. The driver now calls `clearParam()` on failure.
- **`exit(-1)` in `on_init()`.** A missing `ID` or `type`, or an unknown gpio type, killed the whole process. `on_init()` now returns `CallbackReturn::ERROR`.
- **Count checks.** The joint and transmission count checks used `&&` where `||` was meant, so a mismatch was only caught when both sizes were wrong. The joint error message also printed the command vector size twice; it now prints the command and state sizes.
- **`package.xml`.** Added the missing `rclcpp_lifecycle` and `ament_index_cpp` dependencies, which `CMakeLists.txt` already used.

---

## 7. Upstream documentation

For general usage, see the original ROBOTIS user guide:
**https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface/blob/jazzy/README.md**

It covers port settings, joints, transmissions, gpio configuration, control-table model files, topics and services.
Its installation section clones the ROBOTIS repository. To use this fork, follow [Installation](#1-installation) instead.
