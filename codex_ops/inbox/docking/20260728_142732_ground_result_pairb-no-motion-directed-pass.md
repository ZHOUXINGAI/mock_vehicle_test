# Ground result: Pair B directed no-motion link passed

Date: 2026-07-28 (Asia/Shanghai)

## Outcome

The real LR24 Pair B control-plane link between Orin1/Carrier and Orin2/Mini
passed the targeted offline benchmark:

- `pass=true`
- `directed_acceptance=true`
- all 20 checks passed
- `failures=[]`
- no arming, Offboard transition, setpoint publication, Arduino output, wheel
  motion, actuator command, or motor command occurred

The operator confirmed before the run that both rover motor-power circuits were
physically disconnected. Mini was also observed through MAVROS as
`connected: true` and `armed: false` both before the run and immediately before
MAVROS shutdown.

## Network and roles

- Ground Wi-Fi: `HUAWEI_F5BF`
- Ground: `192.168.1.35`
- Orin1/Carrier: `192.168.1.34`
- Orin2/Mini: `192.168.1.36`
- Carrier directed endpoint: `1.242->2.242`
- Mini directed endpoint: `2.242->1.242`
- Pair B was used only for compact runtime messages. It did not carry files,
  Git, logs, Codex traffic, or NATS traffic.

## Software deployed and reviewed

The no-motion live-follower implementation is commit:

`8491bc3d05c664abab281157cd7c7d5cd9fb3ea1`

Orin1 implemented the eight-file change. Orin2 reviewed it through a direct
Orin1-to-Orin2 Codex peer request and returned a structured passing verdict.
Orin1 then acknowledged that verdict without Ground relaying the message.

Focused review evidence on Orin2's clean coordination checkout:

- 50 tests collected
- 49 passed
- 1 allowed integration skip because the separate real EasyDocking checkout
  is absent on Orin2
- 0 failed

The exact eight reviewed files were then copied into Orin2's existing dirty
task checkout without pulling, resetting, staging, or overwriting unrelated
work. The five core test groups available in that older checkout produced:

- 39 tests collected
- 38 passed
- 1 allowed integration skip
- 0 failed

## Mini TELEM2 and MAVROS evidence

The read-only TELEM2 check returned the required values, so no PX4 parameter
write or reboot was needed:

```text
MAV_SYS_ID=2
MAV_PROTO_VER=2
MAV_1_CONFIG=102
MAV_1_MODE=7
MAV_1_FORWARD=1
MAV_1_FLOW_CTRL=0
MAV_1_RADIO_CTL=0
MAV_1_RATE=1200
SER_TEL2_BAUD=57600
```

Mini MAVROS connected to PX4 system/component `2.1`. The Router exposed:

```text
/mini_mavros/mavros_router/add_endpoint
/pairb_tunnel/mavlink_sink
/pairb_tunnel/mavlink_source
```

The formal no-motion run used:

- Mini: 150-second endpoint, 10 Hz simulated `MINI_STATE`
- Carrier: 120-second endpoint, 2 Hz `HOLD`
- Carrier corridor plan: 0.2 Hz
- Carrier stale-state threshold: 300 ms
- both local executors: `NoMotionExecutor`

## Raw counters

Carrier:

```text
states_rx=1151
state_seq_gaps=0
commands_tx=237
corridor_plans_tx=24
field_origins_tx=24
executor_decisions=1
zero_output_count=1
blocked_motion_count=0
nonzero_output_count=0
```

Mini:

```text
states_tx=1447
commands_rx=237
command_seq_gaps=0
corridor_plans_rx=24
corridor_plan_seq_gaps=0
rejected=0
aborts_rx=0
executor_decisions=2770
zero_output_count=2770
blocked_motion_count=0
nonzero_output_count=0
```

## Targeted benchmark metrics

- MiniState paired coverage: `1.0` (`1151/1151`)
- PlanCommand paired coverage: `1.0` (`237/237`)
- CorridorPlan paired coverage: `1.0` (`24/24`)
- missing, duplicate, out-of-order, or unexpected packets: `0`
- maximum received MiniState gap: `220 ms`
- maximum Carrier-reported state age: `143.7 ms` (limit `300 ms`)
- maximum Mini-local command gap: `585 ms` (watchdog limit `750 ms`)
- nonzero HOLD command violations: `0`
- Mini command-gate rejects: `0`
- Mini aborts: `0`

Ground artifacts:

```text
C:\Users\14291\Documents\Codex\2026-07-23\ni\pairb_results\
  20260728_141925_nomotion\
    carrier.log
    carrier.csv
    mini.log
    mini.csv
    pairb_report.json
```

Remote artifacts:

```text
Orin1:
/home/jetson/mock_vehicle_test/results/lr24_pairb_dry_run/
  20260728_141956_carrier_formal_nomotion/

Orin2:
/home/seeed/mock_vehicle_test/results/lr24_pairb_dry_run/
  20260728_141925_mini_formal_nomotion/
```

## Analyzer compatibility defect found and fixed

The first offline analysis correctly refused to claim a result because the
endpoint parser required `1.242->2.242` or `2.242->1.242` to be the final text
on the startup line. Real dry-run logs append runtime fields such as
`state_rate=10.0Hz`, so valid artifacts were rejected as lacking an endpoint.

The parser now accepts an endpoint followed by whitespace or end-of-line, and
a regression test exercises real trailing runtime fields. The original logs
were not edited. All 12 analyzer tests pass, and the same four original
artifacts then produced the passing targeted report above.

## Shutdown state and blockers

Both Pair B endpoints ended naturally. Mini MAVROS was stopped after a final
`armed: false` observation. Follow-up process checks found no Pair B, MAVROS,
Offboard, or vehicle process running.

Current blocker for the next motion stage:

- no production wheel-command executor is connected yet
- no wheels-lifted command-executor watchdog/ABORT test has been authorized or
  performed
- real rover movement remains a separate explicitly authorized stage

