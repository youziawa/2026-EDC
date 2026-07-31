---
name: lingshu-uav-communications
description: Design, implement, integrate, and debug the simple LXS1 framed communication protocol for the Lingxiao flight controller, STM32F407 aircraft/car controllers, K230 vision nodes, and the competition ground station. Use when working on frame headers/tails, length fields, command, telemetry, vision-result, task-state, UART/DMA, transparent wireless serial, or flight/car cooperative communication; do not use the retired KGS1/RTSP video-ground-station stack.
---

# LXS1 陆空协同通信

Use this skill to keep the aircraft, car, vision nodes, flight controller adapter, and ground station on one deterministic binary protocol. Read [references/lxs1-protocol.md](references/lxs1-protocol.md) before changing message IDs, frame fields, payload layouts, or timeout rules.

The canonical project specification is `LXS1_通信协议/LXS1_PROTOCOL.md`; reusable C and Python implementations are in `LXS1_通信协议/include`, `src`, and `python`. Preserve those constants across all nodes.

## Architecture

- Use LXS1 for every control, vision-result, state, heartbeat, fault, and telemetry path.
- Use UART LXS1 between each F407 and its K230. Default to 115200 8N1 with DMA+IDLE on STM32.
- Put the Lingxiao vendor protocol behind an aircraft-main `FC Adapter`. Never assume the flight controller accepts LXS1 bytes; only the adapter may translate semantic `FC_CMD`/`FC_STATE` messages.
- Let the aircraft F407 own the mission state machine; let the car F407 own line following and motor control; let K230 nodes only publish perception results; let the ground station monitor, log, query, and abort.
- Keep video out of the protocol. The retired KGS1/RTSP system and the old `AA FF F1` K230 UART format are not compatibility targets.

## Implementation workflow

1. Confirm the actual board revision, UART pins, transceiver, voltage levels, baud rate, and Lingxiao interface before coding. Treat unknown hardware facts as integration parameters, not guesses.
2. Configure UART, DMA, IDLE interrupt, timers, GPIO, and NVIC in CubeMX. Regenerate code, then place custom code in `USER CODE` regions or separate BSP/application files.
3. Use static buffers. Feed received bytes into an incremental LXS1 parser from the main loop or a non-blocking task. Keep interrupt callbacks short: copy/mark data only; never parse a complete frame, delay, print, allocate, or block in an ISR.
4. Validate the two-byte SOF, length, source, destination, message ID, and fixed two-byte EOF before dispatching a message. Count valid frames, format failures, overflow, and timeout events; do not add checksum or CRC fields.
5. Make commands with side effects idempotent. Use `run_id` for `TASK_START` and let the mission state machine trigger each action once; do not add ACK/retry fields to the simple frame.
6. Apply link timeouts from the reference. On timeout, transition to a safe state rather than continuing stale motion commands.
7. Verify interoperability with the Python reference before connecting hardware, then test each physical link and finally the complete task sequence.

## Device contracts

- Aircraft F407 receives local `VISION_PIXEL` from sky K230, combines it with fresh FC height to close the tracking loop and mission state machine, sends semantic `FC_CMD` to the Lingxiao adapter, and publishes `TASK_STATE`/telemetry. The pixel frame never leaves the K230—F4 link.
- Car F407 receives `VISION_LINE` from car K230, closes the motor/line-following loop, broadcasts `TASK_START`, and publishes `CAR_STATE`/`CAR_POSE`.
- Sky K230 publishes target/platform errors with confidence and age. It must publish `valid=0` when a result is stale or unavailable; never repeat the last valid target without a validity flag.
- Car K230 publishes line lateral error, heading error, curvature, confidence, and lost count. It must not own car speed or mission transitions.
- Ground station displays and records UAV/car pose, state, elapsed time, confidence, battery, link quality, and fault code. In competition mode it may send only management messages such as `TASK_ABORT`, `RESET_FAULT`, and `PING`.

## Message and timing rules

- Frame: `AA 55 | LEN | SRC | DST | MSG | DATA[0..N] | 0D 0A`.
- `LEN = 3 + DATA长度`，统计 `SRC、DST、MSG、DATA`；最大数据长度64字节。
- 不使用版本、序号、时间戳、校验位、ACK、重试或CRC字段。
- Unknown messages may be dropped after parser resynchronization.
- Send sky vision at 10–20 Hz, car vision at 40 Hz, `TASK_STATE` and car telemetry at 10 Hz, heartbeat at 1 Hz, and ground UI refresh at least 5 Hz.
- Treat sky vision as invalid after 300 ms, flight state as stale after 500 ms, car K230 transport as stale after 180 ms, and ground-station telemetry as offline after 2 s.

## Testing checklist

- Run Python parser tests for round-trip, fragmented frames, coalesced frames, bad frame tails, payload layouts, and resynchronization.
- Connect each UART with a logic analyzer or USB-UART and verify baud, byte order, frame length, source/destination, message ID, and fixed frame tail.
- Test duplicate `TASK_START` and duplicate action commands; confirm they do not trigger duplicate takeoff, drop, landing, or motor start.
- Unplug K230, flight controller, car controller, and radio one at a time. Confirm safe behavior and a visible fault code.
- Log real test values for B-point time, D-point time, height, platform dwell time, lap time, and task result for the design report.
- Do not delete or revive the retired KGS1/RTSP code as part of LXS1 work unless the user explicitly requests a separate migration.

## Reference

Load [references/lxs1-protocol.md](references/lxs1-protocol.md) for the node IDs, payload formats, state machine, timeout values, and review checklist.
