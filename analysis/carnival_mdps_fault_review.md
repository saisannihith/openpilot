# 2024 Kia Carnival MDPS fault review

## Scope

This review targets the sudden `Steering Assist Temporarily Unavailable` interruption seen while the driver manually
repositions the wheel against openpilot. It does not retune ordinary lateral tracking, lower the Carnival's 409-unit
steering ceiling, suppress a real EPS fault, or change the LKA request bit.

The saved analyzer report covers 370 locally archived segments, 35 fault-bearing candidate segments, and 52 historical
`steerFaultTemporary` rising edges. Older software cohorts are retained in the report for context, but only the latest
route cohort is used to select the runtime prevention.

## MDPS state decoding

The Hyundai CAN-FD `MDPS` frame at address `0xEA` now exposes the official two-bit states separately:

- LKA plug-in state: bit 46
- torque-overlay active state: bit 48
- torque-overlay unavailable state: bit 50
- torque-overlay fault state: bit 52
- LKA fail state: bit 54

Hyundai's DBC definition distinguishes temporary unavailability, such as torque limiting or low supply voltage, from a
TOI fault caused by an MDPS, CAN, or controller-logic failure. Historical Carnival interruptions are overwhelmingly TOI
fault transitions, not temporary-unavailable transitions.

## Current-cohort result

After removing duplicate archive copies, commit `cf23fee1a46159a7074c0305838aef8fac9a702a` contains seven unique relevant
TOI fault transitions. Every one has all of the following properties:

- strong driver torque opposing the transmitted steering command in the final 0.5 seconds;
- no outgoing `ToiFltSta` request from openpilot;
- no LKA request-bit drop before the initial MDPS fault;
- no sustained high-angle request-guard condition;
- peak opposing driver torque of at least 415 raw column-torque units.

Two highway faults arrived 0.143 and 0.193 seconds after the last opposing driver torque above 300 units. This delayed
MDPS response explains why an unwind that stops immediately when the driver releases the wheel can still reapply torque
before the EPS reports its fault.

## Prevention selected

The controller now applies one Carnival-only driver-conflict state:

- trigger only when driver torque is at least 300 units and opposes a transmitted command of at least 40 units;
- retreat toward zero by at most 10 units per control frame, matching Panda's driver-retreat safety limit;
- retain neutral authority for 20 frames (0.20 seconds) after the strong conflict ends;
- resume through the existing speed-dependent torque rate limiter;
- preserve all 409 units when the driver is not opposing openpilot.

Historical counterfactual replay places the steering command at zero at all seven current-cohort fault edges. The broad
alternative of winding down on any high column torque was rejected because it would suppress valid steering during
roughly 11% of engaged frames. Lowering global steering torque and changing request-bit behavior were also rejected
because neither matches the decoded cause and both reduce useful lateral authority.

## Verification

- 7 focused Carnival controller and DBC tests pass.
- StarPilot MDPS schema serialization passes.
- 42 Hyundai radar/DBC parser tests and 109 parser subtests pass.
- The broader Hyundai suite reports 252 tests and 636 subtests passing. Its 13 unrelated fingerprint/non-SCC firmware
  failures reproduce unchanged at the unmodified starting commit.
- The generated Hyundai CAN-FD DBC is synchronized with its generator source.
- Python compilation and changed-file lint checks pass.

## Proof boundary

This is route-replay and unit-test proof, not engaged-drive proof. It prevents the decoded driver-conflict sequence while
respecting the existing steering safety contract, but only a hash-matched on-road route can prove the vehicle's EPS no
longer enters TOI fault under the same manual-reposition scenario. A genuine MDPS hardware, CAN, or voltage fault must
still disengage lateral control and remain visible to the driver.
