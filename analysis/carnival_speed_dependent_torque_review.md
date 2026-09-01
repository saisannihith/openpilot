# Carnival Speed-Dependent Torque Review

## Decision

Do not give the proposed speed-dependent torque learner steering authority on
the 2024 Kia Carnival. The route-held-out physical-response replay rejects it.

This decision does not disable current `torqued`. The existing single-factor
learner remains active and is the production baseline.

## Evidence

- Vehicle: `KIA_CARNIVAL_4TH_GEN`, torque steering.
- Dataset: 21 routes, 833,672 strict usable samples, 138.95 minutes at 100 Hz.
- Physical response: calibrated `livePose` yaw rate and roll compensation.
- Input: applied actuator torque from `carOutput`, not requested PID output.
- Delay: each command is paired with physical response at logged lateral delay.
- Isolation: two continuous clean seconds, no override/fault/saturation context.
- Fit: torqued-compatible total least squares with route and torque-bucket balance.
- Validation: leave one complete route out; no sample-level random split.

The current global fit was `1.59295` with friction `0.13579`, close to the
production source tuple and recent valid live-torque estimates. The six-bin
candidate produced factors `[1.9893, 1.8209, 1.9032, 1.5553, 1.5739, 1.5884]`.

Held-out results versus a global torqued fit:

- Weighted mean error: `-0.204%` improvement (regression).
- Weighted low-speed mean error: `-0.900%` improvement (regression).
- Weighted p95 error: `-0.196%` improvement (regression).
- Worst route: `-8.149%` regression.
- Strict data duration gate: 138.95 of 300 required minutes.

See `analysis/carnival_speed_dependent_torque_report.json` for every route and
gate. The analyzer exits with status 2 when actuation is rejected.

## Architecture Review

1. Independent online speed bins fragment the same torque dataset into six
   smaller learners. The recorded low-speed relationship is not stable across
   held-out routes, so interpolation turns route-specific conditions into a
   persistent control policy.
2. Below 15 m/s, steering response includes tire, steering geometry, and
   transient vehicle effects that the controller already addresses with its
   speed-dependent feedback gains. Learning those effects into the feedforward
   factor duplicates controller responsibility.
3. The upstream draft mutates `latAccelFactor` and friction every frame. Its
   manual torque override path can then overwrite those values periodically,
   making behavior depend on update order when both features are enabled.
4. A profile that cannot beat the current global learner on unseen routes does
   not justify another runtime state machine, cache schema, toggle, or failure
   mode in the safety-critical steering path.
5. NNFF and NNFF Lite remain off. This review does not re-enable either model
   and does not alter the vehicle torque or panda safety limits.

## Changes Kept

- The dataset extractor now records applied actuator torque, calibrated
  physical lateral acceleration, logged lateral delay, and live-torque state.
- The deterministic acceptance analyzer and tests make future candidates prove
  route-held-out improvement before receiving steering authority.

## Proof Boundary

This is an offline rejection result. No speed-dependent actuation was installed,
so no engaged-drive claim is made or needed for the rejected candidate.
