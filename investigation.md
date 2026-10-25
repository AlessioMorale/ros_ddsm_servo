# DDSM210 comms timeout investigation

## Symptom
Running `ros2 launch kvn_robot_bringup test_01_locomotion.launch.py` (real hardware,
`ddsm210_hardware_interface`), the hardware interface constantly logs corrupted/timed-out
packets, e.g.:

```
Timeout waiting for drive response from motor 1 (0 bytes seen, 0 discarded)
Timeout waiting for mode switch response from motor 4 (0 bytes seen, 0 discarded)
```

Robot: 4x DDSM210 servos on a shared UART bus. Bus topology per hardware owner: full duplex
UART per motor, but all motor TX lines are tied together and all motor RX lines are AND-ed
onto a single host RX line (host TX is broadcast to all motors' RX).

## Fixes already applied (all landed in ddsm210_driver / ddsm210_hardware_interface)

1. **Request/response serialization** (`Motors::send_packet_and_wait_response`, in
   `ddsm210_driver/src/motors.cpp`): `set_mode`/`set_target` now block (with a timeout) until
   the matching response for that id/cmd is observed, instead of firing commands at all
   motors back-to-back with no pacing. Prevents multiple motors replying/overlapping on the
   shared bus.
2. **RX framing bug fix** (`Motors::receive_callback`): the original code assumed one serial
   read == one 10-byte packet. Serial reads are a byte stream and don't align with packet
   boundaries (a read can contain a partial packet, several merged packets, or the tail of one
   + the start of the next). Rewrote to accumulate into a fixed-capacity ring buffer
   (`rx_buffer_`, `std::array`, no heap allocation) and resync on checksum failure (drop one
   byte and retry) instead of discarding the whole read. This was a real, high-impact bug —
   before the fix, "Invalid packet size N" was extremely common.
3. **Preallocated memory**: `rx_buffer_` is a fixed `std::array<uint8_t, packet_size*4>` with a
   `rx_len_` counter and `memmove`/`memcpy` in place, instead of a `std::vector` doing
   `insert`/`erase` (which could reallocate/shift on the hot receive path).
4. **Diagnostics added** in `send_packet_and_wait_response`: on timeout, logs
   `(<bytes_seen> bytes seen, <bytes_discarded> discarded)` using atomic counters
   (`rx_byte_count_`, `rx_discarded_count_`) sampled before/after the wait. 0 bytes seen means
   the motor never replied at all within the window; bytes seen+discarded means something
   arrived but couldn't be framed as a valid response.
5. Reverted an experimental change to the `CMD_DRIVE` payload's reserved bytes
   (`drive._res = 0x01; drive._res2 = 0x02`) that was based on a possibly-newer Waveshare
   protocol revision suggesting these bytes select feedback content
   (0x01=speed, 0x02=current). All example frames in the current Waveshare wiki
   (https://www.waveshare.com/wiki/DDSM210#Protocol_1:_Drive_Motor_to_Rotate) use 0x00 for
   these bytes; setting them to 0x01/0x02 correlated with *all* motors going silent (0 bytes
   seen). Reverting alone did not fix the original per-motor timeout pattern (see below), but
   the non-zero values are clearly not what the current motor firmware expects and should stay
   at 0 (`prepare_packet`'s `memset` default).
6. `RESPONSE_TIMEOUT_US` raised from 4ms-ish default to 12ms then to 50ms while investigating
   (see below — this did **not** fix the core issue; the "failing" motors show 0 bytes seen
   even at 50ms, so it isn't purely a round-trip-latency problem).

## Current, still-unresolved symptom (as of this file's writing)

Extremely deterministic pattern, reproduced multiple times:

- Joint/motor order from `kvn_rover_macro.ros2_control.xacro`:
  `front_left(id=3), front_right(id=1), back_left(id=4), back_right(id=2)`.
- `HardwareInterfaceDDSM210::write()` iterates joints in that exact order every cycle.
- **Motor 3 (first in the per-cycle iteration) always succeeds**, fast (~2.5-3ms measured
  round trip).
- **Motors 1, 4, 2 (processed after) time out on every single cycle**, both for `mode switch`
  and `drive` commands, with "0 bytes seen" even with a 50ms timeout.
- This is not intermittent/random — it is the same pattern every cycle, every run.

### What has been ruled out

- **Raw protocol / physical bus**: a standalone Python script
  (`ddsm210_driver/scripts/test_motor_comm.py`, uses the "Obtain Mode Feedback" 0x75 command,
  no motion) gets **100% success** for all 4 motor ids individually, pairwise (alternating),
  and round-robin, with zero added delay. Also tested with the actual `CMD_DRIVE` (0x64,
  target=0) command via the same script (`--cmd drive`): **100% success**, all ids, all
  patterns. This rules out a fundamentally broken bus/wiring/CRC/motor.
- **The Motors/SerialPort C++ library itself**: a small standalone C++ program
  (compiled ad hoc, see `/tmp/driver_stress_test.cpp` — not committed, recreate if needed)
  that constructs a single `ddsm210_driver::Motors` with ids `{3,1,4,2}` (same order as the
  xacro) and repeatedly calls `set_mode`+`set_target` for each id every 100ms (50 cycles = 200
  requests): **zero timeouts**. This rules out a bug in `Motors`/`SerialPort` in isolation.
- **Duplicate hardware interface instances / stale processes holding the port**: checked via
  logs (`Setting up joint ...` appears exactly once per motor, one "Motor interface activated")
  and `fuser -v /dev/ttyAMA0` — only one process ever holds the fd.
- **Real-time scheduling starving the serial I/O thread**: `controller_manager` logs
  `Could not enable FIFO RT scheduling policy: with error number <1>(Operation not permitted)`
  — RT priority elevation fails in this environment, confirmed via `chrt -p <tid>` on every
  thread in the process: all threads are `SCHED_OTHER`, priority 0. No priority disparity
  exists between the RT/update thread and the boost::asio io thread.
- **CPU affinity pinning**: `taskset -cp <tid>` for every thread in the process shows all
  threads allowed on all 4 cores (`0-3`). No pinning/isolation that could starve one thread.
- **Extra ROS nodes / DDS load**: reproduced with a minimal launch containing only
  `robot_state_publisher` + `controller_manager` + `spawner` for `joint_state_broadcaster`
  (no `skid_steer_controller`, no `joy_to_twist`). Same exact failure pattern. So it's not
  about total node/graph size — it happens with only 3 lightweight processes running (though
  `ros2_control_node` itself spawns ~27 threads, presumably rmw/DDS internals).
- **Boost.Asio canonical/line-buffered termios mode**: checked
  `/usr/include/boost/asio/detail/impl/posix_serial_port_service.ipp` — it does call
  `cfmakeraw()` and explicitly clears `ICANON|ECHO|ECHONL|ISIG|IEXTEN`. Not a canonical-mode
  read-stall bug.
- **Two competing opens on `/dev/ttyAMA0`**: `sudo fuser -v /dev/ttyAMA0` while a stale
  duplicate process happened to exist (from a broken earlier CLI invocation) showed only the
  current, intended process holds the fd. No serial console (`agetty`) is attached to
  `ttyAMA0` (only tty1).

### The most important clue (via `strace`)

Attached `strace -f -tt -e trace=read,write -yy` to the live `ros2_control_node` process
while it was exhibiting the failure. Key findings:

- Outgoing `write()` syscalls for the drive command **do** happen for every motor id
  (`\1d\0\0\0\0\0\0\0P` = id=1, cmd=0x64/drive, target=0, crc=0x50, etc.) — confirmed for ids
  1, 2, 3, 4.
- Valid, checksum-correct **response** bytes for ids 1, 4, 2 **do** eventually show up in
  subsequent `read()` calls (e.g. `"\1d\0\0\0;\1%\0\31\4d\0\0\377\377"` — a full id=1 response
  immediately followed by the start of an id=4 response in the same read).
- However, **zero** `CMD_MODE_SWITCH` (0xA0) bytes were ever observed on the wire in an entire
  multi-second trace, even though our own code was logging "Timeout waiting for mode switch
  response from motor 1/4/2" repeatedly. Cross-referencing our own instrumentation (see next
  point) explains this: the mode only needs to be (re-)sent while `motor_modes_[id]` isn't yet
  confirmed as `VELOCITY`; once a late/stray `CMD_MODE_SWITCH_RESPONSE` is processed (even
  after our synchronous wait already gave up and logged a timeout), `process_packet()` still
  updates `motor_modes_[id]`, so subsequent cycles stop re-sending the mode switch — explaining
  why "mode switch" timeout log lines for a given id stop appearing after the first cycle even
  though the *drive* timeouts continue every cycle.
- Added temporary high-resolution tracing (`std::chrono::steady_clock` around the
  `response_cv_.wait_for` call) directly in `send_packet_and_wait_response`. Confirmed our own
  wait genuinely blocks for very close to the configured timeout (~12000-12070us measured for
  a 12000us configured timeout) for ids 1/2/4, and only ~2500-3100us for id=3 (a real,
  fast, successful round trip). This rules out a "wait_for returns instantly" logic bug — the
  wait duration is accurate; the responses for 1/2/4 simply are not arriving inside that
  window from the C++ code's point of view, while they eventually do show up in the raw
  syscall trace (implying the actual round trip for those ids is longer than the window, at
  least occasionally, from the process's perspective).
- Increasing `RESPONSE_TIMEOUT_US` from 12ms to 50ms did **not** fix it — motors 1/2/4 still
  show "0 bytes seen" (not just "arrived late"), meaning within the code's own accounting, no
  bytes are attributed to that wait window at all even at 50ms. This is inconsistent with a
  simple "just needs a longer timeout" explanation and suggests something more structural.

### Working hypotheses not yet confirmed

1. **Software/ordering bug, not hardware**: since motor 3 is always first in
   `HardwareInterfaceDDSM210::write()`'s iteration and always succeeds, while whichever motors
   come after it always fail, this smells like an ordering-dependent bug rather than something
   special about motor id 3 specifically. **Suggested next test (not yet run): reorder the
   joints in `kvn_rover_macro.ros2_control.xacro`** (e.g. put `front_right_joint` (id=1) first
   instead of `front_left_joint` (id=3)) and rerun. If motor 1 now succeeds every cycle and
   motor 3 starts failing, this proves it's positional and not motor-specific, and the
   investigation should focus purely on `HardwareInterfaceDDSM210::write()`/`Motors` ordering
   effects instead of hardware/bus theories.
2. Something about running inside the full `ros2_control_node` process (rclcpp/DDS/rmw,
   ~27 threads) vs. a bare-metal single-threaded test changes the effective timing enough that
   only the *first* command per cycle reliably completes inside our timeout window, while
   later ones in the same cycle don't — even though the raw syscalls show the data eventually
   arriving. Not yet root-caused. RT scheduling and CPU affinity were checked and ruled out as
   the mechanism (see above), so if it is scheduling-related, it's something subtler (e.g.
   DDS/rmw participant threads causing periodic scheduling latency spikes, page fault/memory
   behavior, or something in how `boost::asio::io_context` interacts with a heavily
   multi-threaded host process). No confirmed root cause yet.

## Known latent bug (found but not yet fixed, unrelated to the above)

`HardwareInterfaceDDSM210::emergency_stop()` unconditionally does
`std::lock_guard<std::mutex> lock(interface_mutex_);`. Several call sites
(`read()`, `write()`, `on_configure()`, `on_activate()`, `on_deactivate()` catch blocks, and
part of `safety_monitor()`) call `emergency_stop()` **while already holding
`interface_mutex_`** via their own `lock_guard` — this is a self-deadlock (`std::mutex` is not
recursive) if any of those code paths ever actually throws/triggers. It hasn't been observed
to fire in current testing because `set_mode`/`set_target` failures don't throw (they just log
and return), so `motor_states_` never flips away from `ACTIVE`/`INITIALIZED` and
`is_motor_operational()` stays true. Still worth fixing defensively (e.g. split into a
lock-free `emergency_stop_locked()` used by callers that already hold the lock, and a public
`emergency_stop()` that acquires it) since it will hang the whole control loop the first time
it's triggered.

## Useful artifacts / repro commands

- Diagnostic script: `ddsm210_driver/scripts/test_motor_comm.py` — raw pyserial tester, no ROS
  deps. `python3 test_motor_comm.py /dev/ttyAMA0 --count 30 --cmd drive` (or `--cmd get-mode`).
  Stop `ros2_control_node` first so it doesn't fight over the port.
- Minimal repro launch used above (not committed, recreate if needed at
  `/tmp/minimal_repro_launch.py`): just `robot_state_publisher` + `controller_manager`
  (using the real `kvn_robot_bringup/config/controllers.yaml`) + a `joint_state_broadcaster`
  spawner. Reproduces the exact same per-motor timeout pattern as the full
  `test_01_locomotion.launch.py`.
- `strace -p <ros2_control_node PID> -f -tt -e trace=read,write -yy -s 200 -o out.trace` (needs
  `sudo` since ptrace_scope=1 blocks unrelated-process attach) is the most useful tool for
  seeing what's actually happening on `/dev/ttyAMA0` at the syscall level, decoupled from our
  own code's bookkeeping.
- `RESPONSE_TIMEOUT_US` lives in `ddsm210_driver/include/ddsm210_driver/protocol.hpp` (currently
  50000 = 50ms after this investigation; was 12000 before).
