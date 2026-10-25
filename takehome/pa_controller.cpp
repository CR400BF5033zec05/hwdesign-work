#include "pa_controller.hpp"


// ============================================================
// Helper: absolute value of signed Q1.14 raw sample
//
// Input is signed 16 bits, but abs(-32768) = 32768,
// so the result requires 17 unsigned bits.
// ============================================================

static ap_uint<17> abs_q14(ap_int<16> x)
{
#pragma HLS INLINE

    ap_int<17> wide = x;

    if (wide < 0) {
        return (ap_uint<17>)(-wide);
    }

    return (ap_uint<17>)wide;
}


// ============================================================
// Helper: terminate on error
// ============================================================

static void set_error(
    ap_uint<1>& halted,
    ap_uint<8>& error,
    ap_uint<16>& offending_tx_id,

    ap_uint<8> error_code,
    ap_uint<16> tx_id
)
{
#pragma HLS INLINE

    halted = 1;
    error = error_code;
    offending_tx_id = tx_id;
}


// ============================================================
// Helper: write PROCESS response
//
// Response word layout:
//
// [  7:  0]  response code = 0x82
// [ 23:  8]  tx_id
// [ 56: 24]  mean_error, unsigned Q5.28 (33 bits)
// [ 67: 57]  clip_count (11 bits)
// [ 68: 68]  decision: 0 = PASS, 1 = FAIL
// [ 84: 69]  current_power_code, signed 16-bit, 0.1 dBm
// [127: 85]  reserved = 0
//
// Response header is exactly one AXI beat, so TLAST = 1.
// ============================================================

static void write_process_response(
    hls::stream<axis_word_t>& out_stream,

    ap_uint<16> tx_id,
    ap_uint<33> mean_error,
    ap_uint<11> clip_count,
    ap_uint<1> failed,
    ap_int<16> current_power_code
)
{
    axis_word_t out_word;

    out_word.data = 0;

    out_word.data.range(7, 0) = RESP_PROCESS;
    out_word.data.range(23, 8) = tx_id;
    out_word.data.range(56, 24) = mean_error;
    out_word.data.range(67, 57) = clip_count;
    out_word.data[68] = failed;

    ap_uint<16> power_bits =
        current_power_code.range(15, 0);

    out_word.data.range(84, 69) = power_bits;

    out_word.keep = -1;
    out_word.strb = -1;

    // Single-word response header.
    out_word.last = 1;

    out_stream.write(out_word);
}


// ============================================================
// Top-level kernel
// ============================================================

void pa_controller(
    hls::stream<axis_word_t>& in_stream,
    hls::stream<axis_word_t>& out_stream,

    ap_uint<1>& halted,
    ap_uint<8>& error,
    ap_uint<16>& offending_tx_id
)
{
#pragma HLS INTERFACE axis port=in_stream
#pragma HLS INTERFACE axis port=out_stream

#pragma HLS INTERFACE s_axilite port=halted bundle=control
#pragma HLS INTERFACE s_axilite port=error bundle=control
#pragma HLS INTERFACE s_axilite port=offending_tx_id bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control


    // ========================================================
    // Activation state
    //
    // These are intentionally non-static.
    // Every new kernel activation starts clean.
    // ========================================================

    bool configured = false;

    ap_int<16> current_power_code = 0;
    ap_int<16> minimum_power_code = 0;

    ap_uint<16> power_step = 0;

    // Q5.28
    ap_uint<33> error_threshold = 0;

    // Positive signed Q1.14 threshold.
    ap_int<16> clipping_threshold = 0;

    ap_uint<11> max_clip_count = 0;


    // ========================================================
    // Status resets on every accelerator activation.
    // ========================================================

    halted = 0;
    error = ERR_NONE;
    offending_tx_id = 0;


    // ========================================================
    // Process commands until END or fatal error.
    // ========================================================

    while (true) {

        axis_word_t header = in_stream.read();

        ap_uint<8> command =
            header.data.range(7, 0);


        // ====================================================
        // CONFIG
        //
        // One 128-bit header word:
        //
        // [  7:  0] command = 0x01
        // [ 23:  8] initial_power_code, signed 16
        // [ 39: 24] minimum_power_code, signed 16
        // [ 55: 40] power_step, unsigned 16
        // [ 88: 56] error_threshold, unsigned Q5.28
        // [104: 89] clipping_threshold, signed Q1.14
        // [115:105] max_clip_count, unsigned 11
        // [127:116] reserved
        // ====================================================

        if (command == CMD_CONFIG) {

            ap_int<16> new_initial_power;
            ap_int<16> new_minimum_power;
            ap_uint<16> new_power_step;
            ap_uint<33> new_error_threshold;
            ap_int<16> new_clipping_threshold;
            ap_uint<11> new_max_clip_count;

            new_initial_power.range(15, 0) =
                header.data.range(23, 8);

            new_minimum_power.range(15, 0) =
                header.data.range(39, 24);

            new_power_step =
                header.data.range(55, 40);

            new_error_threshold =
                header.data.range(88, 56);

            new_clipping_threshold.range(15, 0) =
                header.data.range(104, 89);

            new_max_clip_count =
                header.data.range(115, 105);


            // --------------------------------------------
            // CONFIG validation
            // --------------------------------------------

            if (new_power_step == 0) {

                set_error(
                    halted,
                    error,
                    offending_tx_id,
                    ERR_INVALID_CONFIG_POWER_STEP,
                    0
                );

                return;
            }


            if (new_minimum_power > new_initial_power) {

                set_error(
                    halted,
                    error,
                    offending_tx_id,
                    ERR_INVALID_CONFIG_POWER_RANGE,
                    0
                );

                return;
            }


            if (new_clipping_threshold <= 0) {

                set_error(
                    halted,
                    error,
                    offending_tx_id,
                    ERR_INVALID_CONFIG_CLIP_THRESHOLD,
                    0
                );

                return;
            }


            if (new_max_clip_count > MAX_NSAMP) {

                set_error(
                    halted,
                    error,
                    offending_tx_id,
                    ERR_NSAMP_OUT_OF_RANGE,
                    0
                );

                return;
            }


            // --------------------------------------------
            // Successful CONFIG replaces previous state.
            // --------------------------------------------

            current_power_code =
                new_initial_power;

            minimum_power_code =
                new_minimum_power;

            power_step =
                new_power_step;

            error_threshold =
                new_error_threshold;

            clipping_threshold =
                new_clipping_threshold;

            max_clip_count =
                new_max_clip_count;

            configured = true;

            continue;
        }


        // ====================================================
        // PROCESS
        //
        // Header:
        //
        // [  7: 0] command = 0x02
        // [ 23: 8] tx_id
        // [39:24] nsamp
        // [127:40] reserved
        //
        // Header TLAST = 1.
        //
        // Then exactly nsamp sample beats.
        // ====================================================

        if (command == CMD_PROCESS) {

            ap_uint<16> tx_id =
                header.data.range(23, 8);

            ap_uint<16> nsamp =
                header.data.range(39, 24);


            // --------------------------------------------
            // PROCESS validation
            // --------------------------------------------

            if (!configured) {

                set_error(
                    halted,
                    error,
                    offending_tx_id,
                    ERR_PROCESS_BEFORE_CONFIG,
                    tx_id
                );

                return;
            }


            if (nsamp == 0) {

                set_error(
                    halted,
                    error,
                    offending_tx_id,
                    ERR_ZERO_LENGTH_PROCESS,
                    tx_id
                );

                return;
            }


            if (nsamp > MAX_NSAMP) {

                set_error(
                    halted,
                    error,
                    offending_tx_id,
                    ERR_NSAMP_OUT_OF_RANGE,
                    tx_id
                );

                return;
            }


            // =================================================
            // Per-window state
            //
            // Worst-case error_sample fits 33 bits.
            //
            // 1024 = 2^10 samples, so the accumulator needs:
            //
            // 33 + 10 = 43 bits.
            // =================================================

            ap_uint<43> error_sum = 0;
            ap_uint<11> clip_count = 0;


            // =================================================
            // Sample processing
            // =================================================

            for (
                ap_uint<16> i = 0;
                i < nsamp;
                ++i
            ) {
#pragma HLS PIPELINE II=1

                axis_word_t sample_word =
                    in_stream.read();


                // --------------------------------------------
                // Sample layout:
                //
                // [15: 0] I_ref
                // [31:16] Q_ref
                // [47:32] I_meas
                // [63:48] Q_meas
                // [127:64] reserved
                // --------------------------------------------

                ap_int<16> i_ref;
                ap_int<16> q_ref;
                ap_int<16> i_meas;
                ap_int<16> q_meas;

                i_ref.range(15, 0) =
                    sample_word.data.range(15, 0);

                q_ref.range(15, 0) =
                    sample_word.data.range(31, 16);

                i_meas.range(15, 0) =
                    sample_word.data.range(47, 32);

                q_meas.range(15, 0) =
                    sample_word.data.range(63, 48);


                bool expected_last =
                    (i == nsamp - 1);


                // --------------------------------------------
                // TLAST validation
                // --------------------------------------------

                if (
                    sample_word.last
                    &&
                    !expected_last
                ) {

                    set_error(
                        halted,
                        error,
                        offending_tx_id,
                        ERR_EARLY_TLAST,
                        tx_id
                    );

                    return;
                }


                if (
                    expected_last
                    &&
                    !sample_word.last
                ) {

                    set_error(
                        halted,
                        error,
                        offending_tx_id,
                        ERR_MISSING_TLAST,
                        tx_id
                    );

                    return;
                }


                // --------------------------------------------
                // Constellation error:
                //
                // eI = I_meas - I_ref
                // eQ = Q_meas - Q_ref
                //
                // 17 bits are needed after subtraction.
                // --------------------------------------------

                ap_int<17> e_i =
                    (ap_int<17>)i_meas
                    -
                    (ap_int<17>)i_ref;

                ap_int<17> e_q =
                    (ap_int<17>)q_meas
                    -
                    (ap_int<17>)q_ref;


                // 17x17 multiplication produces enough width
                // for every possible difference.
                ap_int<34> e_i_sq =
                    e_i * e_i;

                ap_int<34> e_q_sq =
                    e_q * e_q;


                ap_uint<33> sample_error =
                    (ap_uint<33>)e_i_sq
                    +
                    (ap_uint<33>)e_q_sq;


                error_sum +=
                    (ap_uint<43>)sample_error;


                // --------------------------------------------
                // Clipping detection
                // --------------------------------------------

                ap_uint<17> abs_i =
                    abs_q14(i_meas);

                ap_uint<17> abs_q =
                    abs_q14(q_meas);

                ap_uint<17> clip_limit =
                    (ap_uint<17>)
                    clipping_threshold;


                bool clipped =
                    (abs_i >= clip_limit)
                    ||
                    (abs_q >= clip_limit);


                // Count complex sample only once.
                if (clipped) {
                    clip_count++;
                }
            }


            // =================================================
            // Mean error
            //
            // All values are non-negative, therefore
            // round-nearest/ties-away-from-zero becomes:
            //
            // q = sum / N
            // r = sum % N
            //
            // if 2*r >= N:
            //     q++
            // =================================================

            ap_uint<43> quotient =
                error_sum / nsamp;

            ap_uint<16> remainder =
                error_sum % nsamp;


            if (
                ((ap_uint<17>)remainder << 1)
                >=
                nsamp
            ) {
                quotient++;
            }


            ap_uint<33> mean_error;

            const ap_uint<43> MAX_ERROR_VALUE =
                (((ap_uint<43>)1 << 33) - 1);


            if (quotient > MAX_ERROR_VALUE) {
                mean_error = -1;
            }
            else {
                mean_error =
                    quotient.range(32, 0);
            }


            // =================================================
            // PASS / FAIL decision
            // =================================================

            bool failed =
                (mean_error > error_threshold)
                ||
                (clip_count > max_clip_count);


            // =================================================
            // PA backoff
            // =================================================

            if (failed) {

                // 18 bits safely cover:
                //
                // signed16 - unsigned16
                //
                ap_int<18> candidate =
                    (ap_int<18>)current_power_code
                    -
                    (ap_int<18>)power_step;


                if (
                    candidate
                    <
                    (ap_int<18>)minimum_power_code
                ) {

                    current_power_code =
                        minimum_power_code;
                }
                else {

                    current_power_code =
                        (ap_int<16>)candidate;
                }
            }


            // =================================================
            // Response
            // =================================================

            write_process_response(
                out_stream,
                tx_id,
                mean_error,
                clip_count,
                failed ? 1 : 0,
                current_power_code
            );

            continue;
        }


        // ====================================================
        // END
        // ====================================================

        if (command == CMD_END) {
            return;
        }


        // ====================================================
        // Unknown command
        //
        // For diagnostic purposes, bits [23:8] are treated as
        // the offending transaction ID if supplied.
        // ====================================================

        ap_uint<16> unknown_tx_id =
            header.data.range(23, 8);

        set_error(
            halted,
            error,
            offending_tx_id,
            ERR_UNKNOWN_COMMAND,
            unknown_tx_id
        );

        return;
    }
}