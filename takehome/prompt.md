# Adaptive QAM PA Power-Backoff Controller

Design and verify a small host-activated Vitis HLS accelerator that estimates whether a QAM transmitter is operating too close to PA compression and recommends a transmit-power setting for an external PMIC, PA bias-control circuit, or transmitter power-control block.

Use the Waveflow command-response pattern and `stream_inband` example as the reference architecture. Configuration must travel in-band on the input stream. AXI-Lite is used only for accelerator control/status such as `ap_start`, `ap_done`, `halted`, and error reporting.

The accelerator is intended as a simplified digital control block, not a complete RF transmitter, QAM modem, digital predistortion system, or standards-compliant EVM measurement instrument.

## 1. Motivation

Higher-order QAM requires good amplitude and phase fidelity. A PA operated too close to saturation or compression can distort the transmitted constellation.

The accelerator receives windows of:

- ideal/reference constellation I/Q samples; and
- measured I/Q samples from a hypothetical transmitter observation path.

For each window, it measures:

- constellation error; and
- clipping behavior.

It then recommends whether the requested transmit-power setting should:

- remain unchanged; or
- decrease by one programmable backoff step.

The intended operating procedure is:

1. begin with a relatively high requested transmit-power setting;
2. observe a window of transmitted constellation samples;
3. determine whether distortion is excessive;
4. if distortion is excessive, reduce the requested TX power;
5. repeat until the measured constellation satisfies the configured quality limits.

A typical wireless transmitter may operate near a maximum conducted output power on the order of 20 to 25 dBm, depending on the device, band, modulation, bandwidth, regulatory limits, and implementation.

The exact PA characteristics are outside the scope of this accelerator.

The accelerator does not directly generate a raw PMIC register value. It produces a requested transmit-power setting expressed in physical units. A downstream hardware or software control layer may convert that setting into the actual PMIC, DAC, bias-controller, or PA-control code used by a specific transmitter.

## 2. I/Q numeric representation

Use signed 16-bit fixed-point values for all reference and measured I/Q samples.

Use Q1.14 representation:

- 1 sign bit;
- 1 integer bit;
- 14 fractional bits.

Therefore the represented numerical range is approximately:

`-2.0 <= x <= 1.99994`

These I/Q numbers are normalized complex-baseband amplitudes.

They are not:

- volts;
- watts;
- dBm;
- PA output-power settings.

The reference constellation is expected to be normalized to a convenient range near `-1.0` to `+1.0`.

The Q1.14 range intentionally provides headroom above the nominal normalized constellation so measured samples may exhibit gain error, overshoot, or distortion without immediately overflowing.

The reference and measured I/Q samples must use the same normalization convention.

The agent must ensure that all intermediate subtraction, multiplication, squaring, addition, and accumulation operations use sufficient width so that no unintended intermediate overflow occurs.

Do not silently rely on C/C++ signed integer overflow.

Where a value must be reduced to a smaller representation, use deterministic rounding.

Use round-to-nearest with ties away from zero unless otherwise specified.

Saturate externally visible reduced-width results rather than wrapping on overflow.

## 3. TX-power representation

Represent the requested transmitter power using a signed 16-bit integer named `power_code`.

The unit of `power_code` is:

`0.1 dBm`

Therefore:

`TX_power_dBm = power_code / 10`

Examples:

- `250` represents `+25.0 dBm`;
- `235` represents `+23.5 dBm`;
- `200` represents `+20.0 dBm`;
- `50` represents `+5.0 dBm`;
- `0` represents `0.0 dBm`;
- `-50` represents `-5.0 dBm`.

The use of a signed representation is intentional so the control interface may represent requested powers below 0 dBm if desired.

The accelerator does not need to convert dBm to watts and does not perform logarithmic calculations.

The configured power-backoff step uses units of `0.1 dB`.

For example:

`power_step = 5`

means:

`0.5 dB backoff per failed window`.

Because the power setting is represented in dBm units, applying a power backoff is implemented as subtraction on the integer code.

## 4. Commands

Support three commands:

- `CONFIG`
- `PROCESS`
- `END`

The agent may choose:

- numeric command codes;
- exact header layouts;
- response layouts;
- Waveflow schema names;
- packing conventions.

All choices must be documented in `results.md`.

## 5. CONFIG command

CONFIG establishes the operating parameters for the current accelerator activation.

It contains:

- initial requested TX-power code;
- minimum allowed TX-power code;
- power decrement step;
- constellation-error threshold;
- clipping threshold;
- maximum allowed clipped-sample count per observation window.

### Power fields

`initial_power_code`:

- signed 16-bit integer;
- units of 0.1 dBm.

`minimum_power_code`:

- signed 16-bit integer;
- units of 0.1 dBm.

`power_step`:

- unsigned integer;
- units of 0.1 dB;
- must be greater than zero.

Example:

`initial_power_code = 250`

means:

`25.0 dBm`

and:

`power_step = 5`

means:

`0.5 dB`.

A second CONFIG command in the same activation replaces the previous configuration and resets the current requested power setting to the new `initial_power_code`.

At the beginning of every accelerator activation, no valid configuration exists.

Configuration and requested TX-power state must not persist between accelerator activations.

## 6. PROCESS command

A PROCESS command represents one measured observation window.

It contains:

- transaction ID `tx_id`;
- number of samples `nsamp`;
- followed by exactly `nsamp` complex sample records.

Each sample record contains four signed Q1.14 values:

- `I_ref`
- `Q_ref`
- `I_meas`
- `Q_meas`

`I_ref` and `Q_ref` describe the ideal/reference QAM constellation point expected for that transmitted symbol.

`I_meas` and `Q_meas` describe the corresponding measured point from the hypothetical TX observation path.

Require:

`nsamp >= 1`

The implementation may impose a reasonable maximum observation-window size if required for hardware design, but any maximum must be explicitly documented and verified.

## 7. Constellation-error metric

For every sample calculate:

`eI = I_meas - I_ref`

`eQ = Q_meas - Q_ref`

Then calculate:

`error_sample = eI^2 + eQ^2`

Accumulate this metric over the complete observation window:

`error_sum = sum(error_sample)`

Then calculate:

`mean_error = error_sum / nsamp`

This quantity is a simplified mean squared constellation-error metric.

It is intentionally simpler than standards-compliant EVM.

Do not calculate:

- square-root EVM;
- RMS EVM;
- percentage EVM;
- EVM in dB;
- BER;
- SNR.

The purpose is to produce a simple distortion metric that:

- reflects amplitude and phase displacement of the measured constellation;
- is practical to implement in HLS;
- is easy to verify by hand.

The configured `error_threshold` must use the same externally visible fixed-point representation as `mean_error`.

The agent must define and document the exact fixed-point width and fractional-bit count used for:

- `error_sample`;
- `error_sum`;
- `mean_error`;
- `error_threshold`.

The representation must provide sufficient width for all legal input samples and configured maximum window sizes.

## 8. Clipping detection

For each measured complex sample, independently examine the absolute value of both measured components.

A measured sample is considered clipped if:

`abs(I_meas) >= clipping_threshold`

OR

`abs(Q_meas) >= clipping_threshold`

Count each complex sample at most once.

Therefore, if both I and Q exceed the clipping threshold for one complex sample, the sample contributes exactly one to `clip_count`.

Clipping detection uses only:

- `I_meas`;
- `Q_meas`.

It does not use the reference samples.

`clipping_threshold` uses the same Q1.14 representation as the I/Q samples.

The clipping threshold must be positive.

## 9. Window decision

At the end of every PROCESS observation window, determine whether the measured TX signal quality is acceptable.

The observation window FAILS if either condition is true:

`mean_error > error_threshold`

OR

`clip_count > max_clip_count`

Otherwise the window PASSES.

Equality with a configured limit is acceptable.

Therefore:

`mean_error == error_threshold`

passes the error criterion.

Likewise:

`clip_count == max_clip_count`

passes the clipping criterion.

## 10. TX-power control behavior

Maintain a state variable:

`current_power_code`

during each accelerator activation.

CONFIG initializes:

`current_power_code = initial_power_code`

After each successfully completed PROCESS window:

### If the observation window PASSES

Leave the requested TX-power setting unchanged:

`current_power_code = current_power_code`

### If the observation window FAILS

Attempt to reduce requested TX power by one configured step:

`candidate_power_code = current_power_code - power_step`

Do not allow the requested TX power to fall below:

`minimum_power_code`

Therefore use:

`current_power_code = max(minimum_power_code, candidate_power_code)`

This produces a one-directional adaptive PA-backoff controller:

high requested TX power  
-> observe constellation  
-> excessive distortion  
-> reduce requested TX power  
-> observe again  
-> stop reducing once quality becomes acceptable.

There is no automatic increase in TX power.

The accelerator does not autonomously terminate when a window passes.

The host may continue sending PROCESS commands to collect additional measurements.

## 11. PROCESS response

For every successfully completed PROCESS command, produce a response containing at least:

- echoed `tx_id`;
- `mean_error`;
- `clip_count`;
- PASS/FAIL decision;
- resulting `current_power_code`.

The exact response packing and Waveflow schema may be selected by the agent.

All externally visible field representations must be documented.

Example conceptual response:

`tx_id = 12`

`mean_error = ...`

`clip_count = 2`

`decision = FAIL`

`current_power_code = 235`

The last field means:

`23.5 dBm requested TX power`.

## 12. END command

END terminates the current accelerator activation normally.

END has no sample burst.

END produces no PROCESS response.

## 13. Required error handling

Expose status registers equivalent to:

- `halted`;
- `error`;
- offending `tx_id`.

Clear all status registers at the beginning of each accelerator activation.

At minimum distinguish errors for:

1. PROCESS received before CONFIG.
2. `nsamp = 0`.
3. input TLAST arrives before all `nsamp` sample records have been received.
4. TLAST is absent from the final expected sample record.
5. unknown command type.
6. invalid CONFIG.

Invalid CONFIG must include at least:

- `power_step = 0`;
- `minimum_power_code > initial_power_code`;
- non-positive clipping threshold.

The agent may add additional validation if necessary, but all additional rules must be documented.

If multiple errors could apply to the same received word, stream-framing and TLAST errors take priority where necessary to avoid consuming words outside the malformed transaction.

On any fatal protocol or configuration error:

1. set `halted = 1`;
2. set the appropriate error code;
3. store the offending command's `tx_id` when one exists;
4. if an output burst has already started, terminate the last output word actually produced using TLAST;
5. return immediately;
6. do not drain or consume additional input.

The host is responsible for recovery after a fatal error.

A test performed after an error must use:

- fresh streams; and
- a new accelerator activation.

## 14. State behavior

The following state persists between PROCESS commands within one activation:

- active configuration;
- configuration-valid flag;
- current requested TX-power code.

The following state resets at the beginning of every PROCESS command:

- constellation-error accumulator;
- clipping count;
- received-sample count;
- any other per-window statistic.

The following resets at the beginning of every new accelerator activation:

- configuration-valid state;
- current TX-power state;
- status/error state;
- all per-window state.

No state may unintentionally carry between activations.

## 15. Required functional verification

Create an independent Python golden model before relying on the HLS implementation.

Verify the Python model using small hand-computable examples.

At minimum test the following behaviors.

### Test A: perfect constellation

Use several reference samples and make every measured sample identical to its corresponding reference sample.

Expected:

- `mean_error = 0`;
- `clip_count = 0`, assuming all measured samples are below the clipping threshold;
- PASS;
- current power unchanged.

### Test B: small acceptable constellation error

Use measured I/Q values slightly different from their references but below both configured quality limits.

Expected:

- nonzero mean error;
- PASS;
- current power unchanged.

Calculate the expected mean error independently by hand.

### Test C: excessive constellation error

Construct a small observation window where the measured samples differ enough from the reference samples to exceed `error_threshold`.

Expected:

- FAIL;
- requested TX power decreases by exactly one `power_step`.

Calculate the expected mean error independently by hand.

Example power behavior:

initial power:

`250 = 25.0 dBm`

step:

`5 = 0.5 dB`

after one failed window:

`245 = 24.5 dBm`

### Test D: clipping

Construct a window containing measured samples that meet or exceed the clipping threshold.

Verify:

- correct `clip_count`;
- FAIL when `clip_count > max_clip_count`;
- one configured power-step reduction.

Include at least one sample for which both measured I and measured Q exceed the clipping threshold.

Verify that this complex sample increments `clip_count` by only one.

### Test E: repeated PA backoff

Start with:

`initial_power_code = 250`

and:

`power_step = 5`

Send several failing PROCESS windows.

Expected TX-power sequence:

`25.0 dBm`

then:

`24.5 dBm`

then:

`24.0 dBm`

then:

`23.5 dBm`

assuming each corresponding observation window fails.

Then send a passing observation window.

Verify that the power code remains unchanged after the passing window.

### Test F: minimum-power saturation

Configure a known minimum requested TX power.

Create enough failing PROCESS windows that a further subtraction would fall below the configured minimum.

Verify that:

`current_power_code == minimum_power_code`

and never wraps around or goes below the configured minimum.

Example:

`minimum_power_code = 180`

represents:

`18.0 dBm`.

### Test G: reconfiguration

Send:

CONFIG  
-> PROCESS  
-> CONFIG  
-> PROCESS  
-> END

Verify that the second CONFIG:

- replaces all thresholds;
- replaces the minimum TX-power setting;
- replaces the backoff step;
- resets `current_power_code` to the new `initial_power_code`.

### Test H: boundary equality

Explicitly verify:

`mean_error == error_threshold`

is acceptable.

Also verify:

`clip_count == max_clip_count`

is acceptable.

### Test I: negative TX-power representation

Use a valid CONFIG with a negative minimum power setting, for example:

`minimum_power_code = -50`

which represents:

`-5.0 dBm`.

Verify that the signed power-code representation is handled correctly.

## 16. Required protocol/error verification

Also verify:

- PROCESS before CONFIG;
- zero-length PROCESS;
- early TLAST;
- missing TLAST;
- invalid CONFIG due to zero power step;
- invalid CONFIG due to minimum power above initial power;
- invalid CONFIG due to invalid clipping threshold;
- unknown command;
- successful fresh activation after each fatal error case.

## 17. Bit-exact verification

The Python golden model, Vitis C simulation, and RTL co-simulation must agree exactly.

Because the accelerator uses fixed-point arithmetic, compare integer/fixed-point representations directly.

For every normal PROCESS transaction compare at least:

- mean error;
- clip count;
- PASS/FAIL decision;
- resulting current power code.

For every error transaction compare:

- halted state;
- error code;
- offending transaction ID where applicable;
- output-stream termination behavior.

Do not use approximate floating-point equality for accelerator results.

## 18. HLS implementation requirements

Use the Waveflow MCP `stream_inband` example and the course command-response pattern as references.

Create:

- Waveflow command/response schemas;
- independent Python golden model;
- Waveflow host-activated module;
- hand-written Vitis HLS kernel body where required;
- C++ C-simulation testbench;
- RTL co-simulation tests;
- build scripts;
- Vitis HLS TCL scripts.

The final Vitis HLS source must be capable of being transferred to a restricted Vitis environment and built there without requiring Waveflow itself to be installed.

The restricted Vitis environment must not require:

- network access;
- Git access;
- package installation;
- additional Python libraries;
- Waveflow installation.

Any generated HLS source and headers required by Waveflow must be created before transfer into the restricted Vitis environment.

The Vitis-side project must be reproducible using ordinary source files and TCL scripts.

Use the course-required FPGA part and clock period.

If those values are unavailable from the assignment environment, report them as missing rather than silently selecting different hardware.

## 19. Recommended Vitis execution structure

Provide at least two reproducible TCL flows.

### C-simulation flow

A script that performs:

- project creation;
- source-file loading;
- testbench loading;
- solution creation;
- FPGA-part configuration;
- clock configuration;
- C simulation.

### Full verification flow

A script that performs:

- C simulation;
- HLS synthesis;
- RTL co-simulation.

The source files and TCL scripts should be portable into a restricted Vitis VM.

Do not require manually creating a Vitis or Vivado GUI project.

## 20. Synthesis and performance verification

After C simulation passes:

1. run HLS synthesis;
2. run RTL co-simulation;
3. verify RTL outputs against the same expected values used for Python and C simulation;
4. record measured latency;
5. record measured throughput;
6. record initiation interval where applicable;
7. record estimated clock performance;
8. record LUT usage;
9. record FF usage;
10. record DSP usage;
11. record BRAM usage.

Do not claim performance results that were not actually produced by the Vitis tools.

## 21. results.md

Produce `results.md` containing:

1. a short description of the accelerator;
2. the intended RF use case;
3. chosen command codes;
4. exact command layouts;
5. exact sample layouts;
6. exact response layouts;
7. exact fixed-point representations for all externally visible fields;
8. explanation that Q1.14 I/Q samples are normalized baseband amplitudes rather than RF power;
9. explanation that `power_code` uses signed 0.1 dBm units;
10. examples converting power codes to dBm;
11. internal widths used for:
    - subtraction;
    - squaring;
    - sample-error calculation;
    - accumulation;
    - division;
12. rounding rules;
13. saturation rules;
14. chosen error codes;
15. table of every functional verification case and PASS/FAIL;
16. table of every protocol/error case and PASS/FAIL;
17. confirmation of whether Python, C simulation, and RTL co-simulation agreed exactly;
18. measured latency;
19. measured throughput;
20. measured initiation interval where applicable;
21. estimated clock period;
22. LUT usage;
23. FF usage;
24. DSP usage;
25. BRAM usage;
26. every point where this prompt was ambiguous and the decision made by the implementation agent;
27. any Waveflow or Vitis problems encountered;
28. exact commands or scripts used to reproduce the tests.

Do not mark a test PASS unless it was actually executed successfully.
