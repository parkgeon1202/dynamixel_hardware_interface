# dynamixel_hardware_interface (kid_rl fork)

> **This is an unofficial, modified fork of [ROBOTIS-GIT/dynamixel_hardware_interface](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface)**, based on the `jazzy` branch (1.5.2).
> ROBOTIS does not maintain or endorse it. DYNAMIXEL is a trademark of ROBOTIS.
> The original code is licensed under the Apache License 2.0 (see [LICENSE](LICENSE)), and the modifications are released under the same license.

This fork was made to run **kid_rl** on a small humanoid. kid_rl is a `ros2_control` controller that runs a reinforcement-learning walking policy, trained in simulation, on the real robot. This page covers three things:

- what kid_rl needs from the hardware interface,
- what was changed from upstream, why, and how,
- how kid_rl uses each change.

Every new feature is **opt-in** through a hardware parameter or a URDF/xacro entry. If you set none of them, the driver behaves like upstream, apart from the bug fixes listed at the end. You can use this fork without kid_rl.

---

## Contents

1. [The kid_rl setup](#1-the-kid_rl-setup)
2. [Installation](#2-installation)
3. [Configuration used with kid_rl](#3-configuration-used-with-kid_rl)
4. [Changes from upstream, and how kid_rl uses them](#4-changes-from-upstream-and-how-kid_rl-uses-them)
5. [Interfaces added by this fork](#5-interfaces-added-by-this-fork)
6. [Bug fixes](#6-bug-fixes)
7. [Upstream documentation](#7-upstream-documentation)

---

## 1. The kid_rl setup

| Item | Value |
|---|---|
| Robot | Kid-size humanoid with 21 Dynamixel servos (MX-28 / MX-64 / MX-106 (Protocol 2.0), XH540) on one U2D2 bus at 1 Mbps |
| Policy joints | 12 leg joints, driven by an ONNX policy at **50 Hz** (`controller_manager` `update_rate: 50`) |
| Held joints | 9 upper-body joints (arms, gripper, torso yaw, camera tilt), held at a fixed pose or driven by external motions |
| Control mode | Position control (Operating Mode 3), P-gain only (I = D = 0, no profile). This matches the actuator model used in training |

A walking policy depends much more on timing than a manipulator does:

- **Stale or missing joint readings change the policy's observation.**
- **A skipped write means the servos keep the previous target for another 20 ms or more.**
- **A spurious fault, such as an input-voltage alert from battery sag, drops torque in the middle of a step.**

Most of the changes below fix one of these three problems. The rest add diagnostics that kid_rl records in its CSV log, so that problems on the real robot can be traced to a specific servo or tick.

---

## 2. Installation

```bash
cd ~/${WORKSPACE}/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
git clone -b kid-rl https://github.com/parkgeon1202/dynamixel_hardware_interface.git
cd ~/${WORKSPACE}
colcon build --packages-up-to dynamixel_hardware_interface
source install/setup.bash
```

Only the hardware interface is forked. DynamixelSDK and dynamixel_interfaces are the unmodified ROBOTIS packages. The fork was developed and tested on ROS 2 Jazzy.

---

## 3. Configuration used with kid_rl

Below is a shortened version of the `ros2_control` xacro that kid_rl uses. It shows one joint. The new parameters and interfaces are marked `NEW`.

```xml
<ros2_control name="KidDynamixel" type="system" rw_rate="50">
  <hardware>
    <plugin>dynamixel_hardware_interface/DynamixelHardware</plugin>
    <param name="port_name">/dev/ttyUSB-U2D2</param>
    <param name="baud_rate">1000000</param>
    <param name="number_of_joints">21</param>
    <param name="number_of_transmissions">21</param>
    <param name="error_timeout_ms">400</param>
    <param name="disable_torque_at_init">true</param>
    <param name="use_fast_read_protocol">false</param>   <!-- NEW (§4.2) -->
    <param name="track_sync_read_rx">true</param>        <!-- NEW (§4.3) -->
    <param name="measure_write_drain">false</param>      <!-- NEW (§4.4) -->
    <param name="read_timeout_ms">14</param>             <!-- NEW (§4.5) -->
    <param name="write_on_read_failure">true</param>     <!-- NEW (§4.6) -->
  </hardware>

  <joint name="left_knee_pitch">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="hardware_state"/>
    <state_interface name="rx_missing_id"/>              <!-- NEW (§4.3) -->
  </joint>

  <gpio name="left_knee_pitch_dxl">
    <param name="type">dxl</param>
    <param name="ID">16</param>
    <command_interface name="Goal Position"/>
    <command_interface name="Position P Gain"/>          <!-- switched at runtime by kid_rl (§4.9) -->
    <state_interface name="Present Position"/>
    <state_interface name="Present Velocity"/>
    <state_interface name="Hardware Error Status"/>
    <state_interface name="Present Temperature"/>
    <state_interface name="Present Input Voltage"/>
    <state_interface name="Present PWM"/>
    <param name="Position P Gain">149</param>            <!-- also the initial command value (§4.9) -->
    <param name="Position I Gain">0</param>
    <param name="Position D Gain">0</param>
    <param name="Profile Velocity">0</param>
    <param name="Profile Acceleration">0</param>
    <param name="Operating Mode">3</param>
    <param name="Return Delay Time">10</param>
  </gpio>
  <!-- transmission omitted: SimpleTransmission, reduction 1.0 -->

  <!-- NEW (§4.4): a virtual gpio that exports the driver's own bus timing -->
  <gpio name="dxl_bus">
    <param name="type">bus_timing</param>
    <param name="ID">255</param>                         <!-- placeholder, never used -->
    <state_interface name="read_ms"/>
    <state_interface name="read_ok"/>
    <state_interface name="read_ok_start_s"/>
    <state_interface name="read_ok_end_s"/>
    <state_interface name="write_sent"/>
    <state_interface name="write_ms"/>
    <state_interface name="write_drain_ms"/>
  </gpio>
</ros2_control>
```

kid_rl checks `robot_description` when it is configured:

- **Optional diagnostics** (`rx_missing_id`, `dxl_bus`, `Present Input Voltage` / `Present PWM`): kid_rl turns its matching CSV logging on only if **every** joint declares the interface. If none or only some declare it, kid_rl leaves that logging off and does not request the interfaces.
- **`Position P Gain` command:** declaring it on only some joints is a configuration error. Declare it on all joints or on none (§4.9).

So you can remove any of these from the xacro, and the controller still runs. It just logs less.

---

## 4. Changes from upstream, and how kid_rl uses them

Each subsection is organised as:

- **Upstream:** how the original driver behaves,
- **Problem:** why that broke walking,
- **Change:** what was implemented,
- **Enable:** how to turn it on,
- **In kid_rl:** where the change is used.

### 4.1 Input Voltage Error is diagnostic-only

- **Upstream:** In Hardware Error Status, bit 0 (Input Voltage Error) is handled like a real fault:
  - `InitDxlComm()` reboots the servo and fails.
  - `WriteItem()` rejects a write whose status packet has the alert bit set.
  - `CheckError()` reports `DXL_HARDWARE_ERROR`, which puts the interface in a fault or reboot loop.
- **Problem:**
  - When the legs push off, battery voltage dips for a moment and several servos raise bit 0. The servo itself keeps working, because this robot leaves bit 0 out of the servos' Shutdown mask.
  - Upstream still treats the alert as a fatal error, so torque dropped or the servos rebooted in the middle of a step.
- **Change** (`dynamixel.hpp`, `dynamixel.cpp`, `dynamixel_hardware_interface.cpp`):
  - New constant `kInputVoltageErrorMask = 0x01`.
  - If bit 0 is the **only** bit set:
    - `InitDxlComm()` logs it and continues without rebooting.
    - `WriteItem()` checks with `IsOnlyInputVoltageAlert()` (it reads Hardware Error Status) and accepts the completed write.
    - `CheckError()` logs a warning at most once per second and removes bit 0 before deciding whether there is a fault.
  - If any other bit is set (overheating, overload, electrical shock, encoder, ...), the error is still a fault, exactly as upstream.
- **Enable:** Always on.
- **In kid_rl:**
  - The controller's `hardware_state_ok()` check reads the joint `hardware_state` interface and stops the policy (it never resumes by itself) on any nonzero value.
  - Because bit 0 is masked out, a voltage dip no longer stops the policy.
  - Supply voltage and PWM are still recorded, through `Present Input Voltage` / `Present PWM`, as the `vin_<joint>` / `duty_<joint>` CSV columns. So voltage dips stay visible in the log.

### 4.2 Fast Sync/Bulk Read can be turned off

- **Upstream:**
  - The driver first tries Fast Sync Read, in which all servos answer in **one** combined packet. It falls back to plain Sync Read only if Fast keeps failing at startup.
  - Once Fast has succeeded, `fast_read_permanent_` is set and the driver never falls back, however often Fast fails later.
- **Problem:**
  - With 21 servos on one bus, a single late or corrupted reply makes the combined packet fail (`-3001` timeout / `-3002` corrupt), so the whole cycle's readings are lost.
  - Under walking load (current spikes, cable motion) this happened often enough to matter.
- **Change:** `Dynamixel::SetFastReadProtocol(bool)` plus the hardware parameter `use_fast_read_protocol`.
- **Enable:** `<param name="use_fast_read_protocol">false</param>`. If the parameter is missing, the default stays enabled, as upstream.
- **In kid_rl:** Set to `false`. Every servo answers in its own status packet, so a bad reply costs at most that servo's reading, not the whole tick. It also makes the per-servo tracking in §4.3 possible.

### 4.3 Per-servo Sync Read reception tracking (`rx_missing_id`)

- **Upstream:**
  - `GroupSyncRead::rxPacket()` returns at the first failure. It does not say which servo was missing, or whether the servos after it answered.
  - On a read failure, the driver pinged every servo one by one, in the control thread, to find the missing one.
- **Problem:**
  - Intermittent read failures could not be traced to a servo or a cable segment.
  - The ping loop blocked the control thread: about 25 ms when the servos answered, and about 870 ms when the bus was down. That made the controller skip many cycles.
- **Change** (new files `dynamixel/sync_read_rx_tracker.hpp/.cpp`):
  - `SyncReadRxTracker` replaces the receive step of plain Sync Read. Transmitting the request and setting the timeout stay in `Dynamixel`.
  - It keeps reading status packets until the timeout, or until every expected ID has arrived or been counted as corrupt. For each ID it records whether a valid reply arrived, and it counts CRC-corrupt packets.
  - Its buffers are sized once in `Configure()`, and `Receive()` does not allocate memory.
  - A failed read now logs `SyncRead Rx Fail ... [missing IDs: 18 19 20 21] [corrupt packets: 0]`, in bus order. One missing ID means one lost packet. A run of missing IDs up to the end of the bus means the daisy chain is broken at that point.
  - `DynamixelHardware::UpdateRxMissingIds()` exports a per-joint state interface `rx_missing_id`: the ID of the joint's servo if its reply was missing this tick, otherwise `-1`. It uses `-1` rather than a 0/1 flag because ID 0 exists.
  - The ping loop was removed.
- **Enable:**
  - `<param name="track_sync_read_rx">true</param>`
  - `<state_interface name="rx_missing_id"/>` on each joint.
  - It only applies to plain Sync Read, so also set `use_fast_read_protocol` to `false`.
  - When it is off (the default), the original SDK receive path is used.
- **In kid_rl:** The controller's `RxMissingMonitor` reads every joint's `rx_missing_id` each tick and writes the `rx_missing_count` and `rx_missing_ids` CSV columns. You can line up servo dropouts with falls, voltage dips or specific motions.

### 4.4 Bus timing diagnostics (`type=bus_timing` gpio)

- **Upstream:** No way to see how long a read took, whether it succeeded, how old the joint readings are, or whether a write actually went out.
- **Problem:** The policy was trained with a known observation delay. To compare the real robot with simulation, kid_rl needs to know how old each joint reading is when the policy uses it.
- **Change:**
  - A new gpio type `bus_timing` describes no servo. It exports the driver's own measurements. `SetupBusTiming()` checks the names, the gpio is skipped during ping and register initialisation, and only one is allowed.

    | Interface | Meaning |
    |---|---|
    | `read_ms` | Duration of this tick's Sync Read [ms] |
    | `read_ok` | 1 if this tick's read succeeded, else 0 |
    | `read_ok_start_s`, `read_ok_end_s` | `steady_clock` time [s] at the start and end of the **last successful** read |
    | `write_sent` | 1 if a Sync Write was sent this tick, else 0 (unchanged command, write skipped, or failure) |
    | `write_ms` | Time to hand the Sync Write to the kernel [ms] |
    | `write_drain_ms` | Time until the bytes have actually left the port, via `tcdrain()` [ms]. `-1` if not measured |

  - Times use `std::chrono::steady_clock`, so a controller in the same process can subtract them from its own `steady_clock` time.
  - The servos sample their values while answering the read, so the time between `read_ok_start_s` and `read_ok_end_s` brackets the sampling moment. After a failed read these timestamps stay at the last success, so the computed age keeps growing, as it should.
  - `write_drain_ms` finds the port's file descriptor in `/proc/self/fd` (`FindPortFd()`), because the SDK does not expose it, and calls `tcdrain()` after each write. This blocks the control thread for a few ms, and USB-serial completion is only reported at kernel tick granularity. So it is off unless `measure_write_drain` is `true`.
- **Enable:** Add the `dxl_bus` gpio shown in §3. Optionally set `<param name="measure_write_drain">true</param>`.
- **In kid_rl:** The controller's `BusTimingMonitor` reads these values every tick and writes these CSV columns:
  - `read_ms`, `read_ok`
  - `joint_age_min_ms = now − read_ok_end_s` and `joint_age_max_ms = now − read_ok_start_s`. These are the lower and upper bound on how old the joint readings in the policy's observation are.
  - `prev_write_sent`, `prev_write_ms`, `prev_write_drain_ms`.

  These columns set the read timeout in §4.5. They are also used when comparing real-robot logs with simulation rollouts.

### 4.5 Read timeout (`read_timeout_ms`)

- **Upstream:** The control-loop Sync Read waits up to the whole control period (20 ms at 50 Hz) for replies.
- **Problem:** If even one reply was missing, `read()` used the full 20 ms. With `update()` and `write()` added on top, the tick overran its period, and `ros2_control` skipped the next tick. So a single lost packet cost the policy one action.
- **Change:** New parameter `read_timeout_ms`. If it is greater than 0, it replaces the period as the read wait limit. With 0 (the default) the driver behaves as upstream.
- **Enable:** `<param name="read_timeout_ms">14</param>`.
- **In kid_rl:**
  - Chosen from the `read_ms` column (§4.4) over 6,452 successful reads: median 8.4 ms, p99.9 11.1 ms, max 12.25 ms. So 14 ms leaves margin.
  - In the worst case, a failed read (14 ms) plus `update()` and `write()` (about 4 ms at most) is about 18 ms. That fits inside the 20 ms tick.
  - If you change the baud rate, the number of servos or the items read, check `read_ms` again.

### 4.6 Write even when the read failed (`write_on_read_failure`)

- **Upstream:** If the last read ended with a communication error (`COMM_ERROR`), `write()` skips sending commands.
- **Problem:** A new policy target was not sent until the next successful read, about 60 ms when combined with the overrun from §4.5. Sync Write does not wait for replies, so a failed read does not prevent the command from going out.
- **Change:** New parameter `write_on_read_failure`. If it is `true`, `write()` also sends commands when the status is `COMM_ERROR`. Hardware errors and the rebooting state are handled as upstream.
- **Enable:** `<param name="write_on_read_failure">true</param>`.
- **In kid_rl:**
  - The policy's target joint positions go out every tick, even when that tick's read failed. The action in that case is computed from the previous tick's joint readings.
  - The policy was trained with a 0–1 step observation delay, which covers exactly this case.

### 4.7 Sync Write only when commands changed, and transmit failures detected

- **Upstream:**
  - `write()` sends a Sync Write every tick, even when nothing changed.
  - A failure that happens only on the transmit side is never reported. `dxl_status_` is updated only by `read()`, so `write()` keeps returning OK while no commands reach the servos.
- **Change** (`dynamixel_hardware_interface.cpp`):
  - `MultiDxlCommandChanged()` compares the current transmission and gpio commands with the last packet sent successfully, which `RememberMultiDxlCommands()` stores. If they are the same, the write is skipped.
  - If `WriteMultiDxlData()` fails, the failure time is added up. Once it reaches `error_timeout_ms`, `write()` returns `ERROR`.
- **Enable:** Always on.
- **In kid_rl:**
  - While the robot holds a pose (SAFE_HOLD, unchanged targets), no Sync Write is sent, which leaves more bus time for reads.
  - `prev_write_sent` (§4.4) shows each tick whether a write went out.
  - A transmit failure that lasts long enough now leads to the ERROR transition in §4.8, instead of the commands silently stopping.

### 4.8 Torque off on `on_error` and `on_shutdown`

- **Upstream:**
  - When `read()` or `write()` returns `ERROR`, `ros2_control` moves the component through the lifecycle ERROR transition, which never calls `on_deactivate()`.
  - The servos therefore **keep torque and their last goal position** with nothing driving them. Shutdown also skips deactivation.
- **Problem:** On a walking robot this freezes a mid-step pose with torque on until the operator cuts power. That is unsafe.
- **Change:**
  - New `on_error()` and `on_shutdown()` overrides that call `stop()`, which disables torque on all servos.
  - `on_error()` returns `SUCCESS`, so the component goes to UNCONFIGURED and can be reactivated with `set_hardware_component_state` without restarting the process. Returning `ERROR` would finalize it.
- **Enable:** Always on.
- **In kid_rl:** If the bus fails for longer than `error_timeout_ms` (400 ms), or the controller manager shuts down, the robot goes limp instead of locking up.

### 4.9 Gpio command interfaces start from the matching `<param>` value

- **Upstream:** Every gpio command interface starts at `0.0`. `start()` sends one Sync Write before any controller has written a value. `Goal Position` is fine, because `start()` first copies the present position into it. Any other register opened as a command interface, however, is written as 0.
- **Problem:** kid_rl opens `Position P Gain` as a command interface. The first write would set every servo's P gain to **0**, leaving the robot limp on activation.
- **Change:**
  - `InitialCommandValue()`: when the gpio has a `<param>` with the same name as the command interface, the command starts at that value. Otherwise it starts at 0, as upstream.
  - A value that is not a number is logged as an error, and the command starts at 0.
- **Enable:** Declare both `<command_interface name="Position P Gain"/>` and `<param name="Position P Gain">...</param>` on the gpio, as in §3.
- **In kid_rl:**
  - The controller (`configure_gains()` / `write_gain_commands()`) switches every servo's P gain at runtime:
    - **Walking:** the P gains from the xacro `<param>`, which match the actuator model used in training.
    - **SAFE_HOLD with the policy disabled**, when scripted motions or manipulation take over: the stiffer `motion_gain.p_gains` from `controllers.yaml` (for example 800 for the legs).
  - The policy starts only after the trained gains have been written for at least two ticks.
  - Declare the interface on **all** servos. If the write items differ between servos, the driver falls back from Sync Write to Bulk Write.
  - If the xacro has no `Position P Gain` command (for example on a mock system), kid_rl turns gain switching off by itself.

---

## 5. Interfaces added by this fork

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
Its installation section clones the ROBOTIS repository. To use this fork, follow [§2](#2-installation) instead.
