Build a host-activated Vitis HLS streaming accelerator using the Waveflow `stream_inband` polynomial example as the reference design.

Follow the course command-response pattern wherever it differs from the Waveflow example. In particular, configuration must travel in-band on `in_stream`; AXI-Lite is only for `ap_start`, `ap_done`, and status registers.

## Operation

Implement scale and offset on 32-bit IEEE-754 floating-point samples:

`y[i] = a * x[i] + b`

The accelerator supports three commands:

- `CONFIG`
- `DATA`
- `END`

The agent may choose the numeric command codes, header layouts, response layouts, schema names, and file formats, but must document those choices and use them consistently.

## CONFIG

`CONFIG` supplies two float32 values:

- `a`
- `b`

A successful CONFIG replaces the current configuration.

A second CONFIG in the same activation replaces the old values with the new values. There is no other persistent arithmetic state to reset.

At the beginning of every activation there is no valid configuration. Configuration must never carry over between activations.

CONFIG has no sample burst.

## DATA

A DATA header contains at least:

- command type
- transaction ID `tx_id`
- sample count `nsamp`

It is followed by a sample burst containing exactly `nsamp` float32 samples.

Each result is computed using the most recent successful CONFIG:

`y = a*x + b`

Arithmetic must be float32 and must use this exact operation order:

1. round `a*x` to float32;
2. add `b`;
3. round the result to float32.

Do not use a fused multiply-add operation.

The DATA response must contain a response header that echoes `tx_id`, followed by the result burst.

TLAST terminates the response header burst and terminates the final word of the result burst.

`nsamp = 0` is legal. It produces the response header but no result-data burst.

## END

END terminates the activation normally.

END has no sample burst and produces no response data.

## Error handling

Expose status registers equivalent to:

- `halted`
- `error`
- offending `tx_id`

Clear all status registers at the beginning of each activation.

At minimum distinguish these errors:

1. DATA received before any CONFIG in the current activation.
2. Input sample TLAST arrives before all `nsamp` samples have been received.
3. TLAST is absent on the word containing the final expected sample.

Also treat an unknown command type as an error.

If more than one error could apply to the same received word, TLAST framing errors take priority over other validations that depend on later words.

On any error:

1. set `halted = 1`;
2. set the appropriate error code;
3. store the offending command's `tx_id`;
4. if an output data burst has already started, terminate the last word actually produced with TLAST;
5. return immediately;
6. do not read or drain any additional input.

The input stream is undefined after an error. Recovery is the host's responsibility. A subsequent test must use fresh streams and a new activation.

## Required implementation

Use the Waveflow MCP `stream_inband` example as the implementation reference.

Create:

- Waveflow schemas for command and response messages;
- a bit-exact Python golden model;
- the Waveflow host-activated module;
- the hand-written Vitis HLS kernel body;
- test scenarios;
- the C++ C-simulation testbench;
- the Waveflow build script necessary to run the Python checks and Vitis C simulation.

Do not manually pack protocol fields when Waveflow schema serialization can generate the packing.

## Verification

First test the arithmetic model using hand-computed cases, including at least:

1. `a=2`, `b=1`, `x=[0, 1, -2, 3.5]`
   expected `[1, 3, -3, 8]`.

2. `a=-0.5`, `b=2`, `x=[2, -4, 0]`
   expected `[1, 4, 2]`.

3. `a=1`, `b=0`, with several positive and negative float32 samples;
   output must be bit-identical to input.

Also test protocol scenarios:

- CONFIG → DATA → END;
- CONFIG → DATA → DATA → END;
- CONFIG → DATA → CONFIG → DATA → END, proving the second CONFIG takes effect;
- DATA before CONFIG;
- early TLAST in a DATA sample burst;
- missing TLAST on the final DATA sample;
- a fresh successful activation after each error scenario.

For every normal scenario, compare the C-simulation output against values independently produced by the Python golden model, bit for bit.

For every error scenario, verify:

- `halted`;
- error code;
- offending `tx_id`;
- correct output-burst termination;
- immediate termination without consuming later commands.

Run the Python model and checks, then Vitis C simulation.

Stop once C simulation passes. Do not run C synthesis, RTL co-simulation, implementation, or timing analysis for this in-class assignment.

## results.md

When finished, write `results.md` containing:

1. a short description of the implemented protocol;
2. the command codes and message layouts chosen;
3. the error codes chosen;
4. a table containing every verification scenario and PASS or FAIL;
5. confirmation of whether Python and C simulation agreed bit-for-bit;
6. the actual C-simulation result;
7. every place where this prompt was silent or ambiguous and what decision the agent made;
8. any problems encountered while using Waveflow or Vitis.

Do not report a test as PASS unless it was actually run successfully.
