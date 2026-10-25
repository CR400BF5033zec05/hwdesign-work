#include "pa_controller.hpp"

#include <iostream>
#include <vector>
#include <string>


// ============================================================
// Simple test bookkeeping
// ============================================================

static int tests_run = 0;
static int tests_failed = 0;


static void check(
    bool condition,
    const std::string& name
)
{
    tests_run++;

    if (condition) {
        std::cout
            << "[PASS] "
            << name
            << std::endl;
    }
    else {
        std::cout
            << "[FAIL] "
            << name
            << std::endl;

        tests_failed++;
    }
}


// ============================================================
// AXI-word constructors
// ============================================================

static axis_word_t blank_word()
{
    axis_word_t w;

    w.data = 0;
    w.keep = -1;
    w.strb = -1;
    w.last = 0;

    return w;
}


static axis_word_t make_config(
    int initial_power,
    int minimum_power,
    unsigned int power_step,
    unsigned long long error_threshold,
    int clipping_threshold,
    unsigned int max_clip_count
)
{
    axis_word_t w = blank_word();

    w.data.range(7, 0) =
        CMD_CONFIG;


    ap_int<16> initial =
        initial_power;

    ap_int<16> minimum =
        minimum_power;

    ap_int<16> clip =
        clipping_threshold;


    w.data.range(23, 8) =
        initial.range(15, 0);

    w.data.range(39, 24) =
        minimum.range(15, 0);

    w.data.range(55, 40) =
        power_step;

    w.data.range(88, 56) =
        error_threshold;

    w.data.range(104, 89) =
        clip.range(15, 0);

    w.data.range(115, 105) =
        max_clip_count;


    // Single-word command header.
    w.last = 1;

    return w;
}


static axis_word_t make_process(
    unsigned int tx_id,
    unsigned int nsamp
)
{
    axis_word_t w = blank_word();

    w.data.range(7, 0) =
        CMD_PROCESS;

    w.data.range(23, 8) =
        tx_id;

    w.data.range(39, 24) =
        nsamp;

    w.last = 1;

    return w;
}


static axis_word_t make_end()
{
    axis_word_t w = blank_word();

    w.data.range(7, 0) =
        CMD_END;

    w.last = 1;

    return w;
}


static axis_word_t make_unknown(
    unsigned int tx_id
)
{
    axis_word_t w = blank_word();

    w.data.range(7, 0) =
        0x7F;

    w.data.range(23, 8) =
        tx_id;

    w.last = 1;

    return w;
}


// ============================================================
// Q1.14 sample constructor
//
// Values are supplied as RAW Q1.14 integers.
// ============================================================

static axis_word_t make_sample_raw(
    int i_ref,
    int q_ref,
    int i_meas,
    int q_meas,
    bool last
)
{
    axis_word_t w = blank_word();

    ap_int<16> ir = i_ref;
    ap_int<16> qr = q_ref;
    ap_int<16> im = i_meas;
    ap_int<16> qm = q_meas;


    w.data.range(15, 0) =
        ir.range(15, 0);

    w.data.range(31, 16) =
        qr.range(15, 0);

    w.data.range(47, 32) =
        im.range(15, 0);

    w.data.range(63, 48) =
        qm.range(15, 0);


    w.last = last ? 1 : 0;

    return w;
}


// ============================================================
// Response decoder
// ============================================================

struct Response {
    unsigned int type;
    unsigned int tx_id;
    unsigned long long mean_error;
    unsigned int clip_count;
    bool failed;
    int power_code;
};


static Response decode_response(
    const axis_word_t& w
)
{
    Response r;

    r.type =
        w.data.range(7, 0).to_uint();

    r.tx_id =
        w.data.range(23, 8).to_uint();

    r.mean_error =
        w.data.range(56, 24).to_uint64();

    r.clip_count =
        w.data.range(67, 57).to_uint();

    r.failed =
        w.data[68];


    ap_int<16> power;

    power.range(15, 0) =
        w.data.range(84, 69);

    r.power_code =
        power.to_int();


    return r;
}


// ============================================================
// One accelerator activation
// ============================================================

struct ActivationResult {
    std::vector<Response> responses;

    bool halted;
    unsigned int error;
    unsigned int offending_tx_id;
};


static ActivationResult run_activation(
    const std::vector<axis_word_t>& input
)
{
    hls::stream<axis_word_t> in_stream;
    hls::stream<axis_word_t> out_stream;


    for (
        std::size_t i = 0;
        i < input.size();
        ++i
    ) {
        in_stream.write(
            input[i]
        );
    }


    ap_uint<1> halted;
    ap_uint<8> error;
    ap_uint<16> offending_tx_id;


    pa_controller(
        in_stream,
        out_stream,
        halted,
        error,
        offending_tx_id
    );


    ActivationResult result;

    result.halted =
        halted != 0;

    result.error =
        error.to_uint();

    result.offending_tx_id =
        offending_tx_id.to_uint();


    while (!out_stream.empty()) {

        axis_word_t w =
            out_stream.read();

        result.responses.push_back(
            decode_response(w)
        );
    }


    return result;
}


// ============================================================
// Verify that a fatal error can be followed by a successful
// fresh accelerator activation.
// ============================================================

static void check_fresh_recovery(
    const std::string& name
)
{
    // Q1.14:
    //
    // 0.25 = 4096
    // -0.25 = -4096

    std::vector<axis_word_t> v;

    v.push_back(
        make_config(
            250,
            180,
            5,

            // 0.01 in Q5.28
            2684355,

            // 1.5 in Q1.14
            24576,

            0
        )
    );

    v.push_back(
        make_process(
            999,
            1
        )
    );

    v.push_back(
        make_sample_raw(
            4096,
            -4096,
            4096,
            -4096,
            true
        )
    );

    v.push_back(
        make_end()
    );


    ActivationResult r =
        run_activation(v);


    bool ok =
        !r.halted
        &&
        r.error == ERR_NONE
        &&
        r.responses.size() == 1
        &&
        !r.responses[0].failed
        &&
        r.responses[0].power_code == 250;


    check(
        ok,
        name + " fresh activation recovery"
    );
}


// ============================================================
// Functional tests
// ============================================================

static void test_perfect_constellation()
{
    std::vector<axis_word_t> v;

    v.push_back(
        make_config(
            250,
            180,
            5,
            2684355,  // 0.01 Q5.28
            24576,    // 1.5 Q1.14
            0
        )
    );

    v.push_back(
        make_process(
            1,
            4
        )
    );


    v.push_back(
        make_sample_raw(
            4096,
            4096,
            4096,
            4096,
            false
        )
    );

    v.push_back(
        make_sample_raw(
            -12288,
            4096,
            -12288,
            4096,
            false
        )
    );

    v.push_back(
        make_sample_raw(
            12288,
            -12288,
            12288,
            -12288,
            false
        )
    );

    v.push_back(
        make_sample_raw(
            -4096,
            -4096,
            -4096,
            -4096,
            true
        )
    );

    v.push_back(
        make_end()
    );


    ActivationResult r =
        run_activation(v);


    bool ok =
        !r.halted
        &&
        r.responses.size() == 1
        &&
        r.responses[0].mean_error == 0
        &&
        r.responses[0].clip_count == 0
        &&
        !r.responses[0].failed
        &&
        r.responses[0].power_code == 250;


    check(
        ok,
        "A perfect constellation"
    );
}


static void test_small_acceptable_error()
{
    // eI = +1/16 = 1024 raw
    // eQ = -1/16 = -1024 raw
    //
    // error =
    // (1/16)^2 + (1/16)^2
    // = 1/128
    //
    // Q5.28 raw:
    // 2^28 / 128 = 2^21 = 2097152

    std::vector<axis_word_t> v;

    v.push_back(
        make_config(
            250,
            180,
            5,
            2684355,  // 0.01
            24576,
            0
        )
    );

    v.push_back(
        make_process(
            2,
            4
        )
    );


    v.push_back(
        make_sample_raw(
            4096,
            4096,
            5120,
            3072,
            false
        )
    );

    v.push_back(
        make_sample_raw(
            -12288,
            4096,
            -11264,
            3072,
            false
        )
    );

    v.push_back(
        make_sample_raw(
            12288,
            -12288,
            13312,
            -13312,
            false
        )
    );

    v.push_back(
        make_sample_raw(
            -4096,
            -4096,
            -3072,
            -5120,
            true
        )
    );

    v.push_back(
        make_end()
    );


    ActivationResult r =
        run_activation(v);


    bool ok =
        !r.halted
        &&
        r.responses.size() == 1
        &&
        r.responses[0].mean_error == 2097152ULL
        &&
        !r.responses[0].failed
        &&
        r.responses[0].power_code == 250;


    check(
        ok,
        "B acceptable hand-computed error"
    );
}


static void test_excessive_error()
{
    // eI = 0.25
    // eQ = 0
    //
    // mean error = 1/16 = 0.0625
    //
    // Q5.28 raw = 2^24 = 16777216

    std::vector<axis_word_t> v;

    v.push_back(
        make_config(
            250,
            180,
            5,

            // 0.05 Q5.28
            13421773,

            24576,
            0
        )
    );

    v.push_back(
        make_process(
            3,
            4
        )
    );


    v.push_back(
        make_sample_raw(
            4096,
            4096,
            8192,
            4096,
            false
        )
    );

    v.push_back(
        make_sample_raw(
            -12288,
            4096,
            -8192,
            4096,
            false
        )
    );

    v.push_back(
        make_sample_raw(
            12288,
            -12288,
            16384,
            -12288,
            false
        )
    );

    v.push_back(
        make_sample_raw(
            -4096,
            -4096,
            0,
            -4096,
            true
        )
    );

    v.push_back(
        make_end()
    );


    ActivationResult r =
        run_activation(v);


    bool ok =
        !r.halted
        &&
        r.responses.size() == 1
        &&
        r.responses[0].mean_error == 16777216ULL
        &&
        r.responses[0].failed
        &&
        r.responses[0].power_code == 245;


    check(
        ok,
        "C excessive constellation error"
    );
}


static void test_clipping()
{
    std::vector<axis_word_t> v;

    v.push_back(
        make_config(
            250,
            180,
            5,

            // 10.0 Q5.28
            2684354560ULL,

            // 1.0 Q1.14
            16384,

            // one clipped sample allowed
            1
        )
    );

    v.push_back(
        make_process(
            4,
            3
        )
    );


    // Both I and Q clip.
    // Must count only once.
    v.push_back(
        make_sample_raw(
            0,
            0,
            16384,
            20480,
            false
        )
    );


    // Second clipped complex sample.
    v.push_back(
        make_sample_raw(
            0,
            0,
            -18432,
            0,
            false
        )
    );


    // Not clipped.
    v.push_back(
        make_sample_raw(
            0,
            0,
            4096,
            4096,
            true
        )
    );


    v.push_back(
        make_end()
    );


    ActivationResult r =
        run_activation(v);


    bool ok =
        !r.halted
        &&
        r.responses.size() == 1
        &&
        r.responses[0].clip_count == 2
        &&
        r.responses[0].failed
        &&
        r.responses[0].power_code == 245;


    check(
        ok,
        "D clipping count and PA backoff"
    );
}


static void test_repeated_backoff()
{
    std::vector<axis_word_t> v;

    v.push_back(
        make_config(
            250,
            180,
            5,
            2684355,
            24576,
            0
        )
    );


    // Three failing windows.
    for (
        unsigned int tx = 10;
        tx <= 12;
        ++tx
    ) {

        v.push_back(
            make_process(
                tx,
                1
            )
        );

        v.push_back(
            make_sample_raw(
                0,
                0,
                4096,
                0,
                true
            )
        );
    }


    // One passing window.
    v.push_back(
        make_process(
            13,
            1
        )
    );

    v.push_back(
        make_sample_raw(
            4096,
            -4096,
            4096,
            -4096,
            true
        )
    );

    v.push_back(
        make_end()
    );


    ActivationResult r =
        run_activation(v);


    bool ok =
        !r.halted
        &&
        r.responses.size() == 4
        &&
        r.responses[0].power_code == 245
        &&
        r.responses[1].power_code == 240
        &&
        r.responses[2].power_code == 235
        &&
        r.responses[3].power_code == 235
        &&
        !r.responses[3].failed;


    check(
        ok,
        "E repeated PA backoff then pass"
    );
}


static void test_minimum_saturation()
{
    std::vector<axis_word_t> v;

    v.push_back(
        make_config(
            190,
            180,
            7,

            // zero threshold:
            // any nonzero error fails
            0,

            24576,
            0
        )
    );


    for (
        unsigned int tx = 20;
        tx <= 22;
        ++tx
    ) {

        v.push_back(
            make_process(
                tx,
                1
            )
        );

        v.push_back(
            make_sample_raw(
                0,
                0,
                4096,
                0,
                true
            )
        );
    }


    v.push_back(
        make_end()
    );


    ActivationResult r =
        run_activation(v);


    bool ok =
        !r.halted
        &&
        r.responses.size() == 3
        &&
        r.responses[0].power_code == 183
        &&
        r.responses[1].power_code == 180
        &&
        r.responses[2].power_code == 180;


    check(
        ok,
        "F minimum power saturation"
    );
}


static void test_reconfiguration()
{
    std::vector<axis_word_t> v;


    v.push_back(
        make_config(
            250,
            180,
            5,
            2684355,
            24576,
            0
        )
    );


    v.push_back(
        make_process(
            30,
            1
        )
    );

    v.push_back(
        make_sample_raw(
            0,
            0,
            4096,
            0,
            true
        )
    );


    // New config:
    //
    // 23.0 dBm initial
    // 15.0 dBm minimum
    // 1.0 dB step
    // 0.10 error threshold
    v.push_back(
        make_config(
            230,
            150,
            10,
            26843546,
            24576,
            0
        )
    );


    v.push_back(
        make_process(
            31,
            1
        )
    );

    v.push_back(
        make_sample_raw(
            0,
            0,
            4096,
            0,
            true
        )
    );


    v.push_back(
        make_end()
    );


    ActivationResult r =
        run_activation(v);


    bool ok =
        !r.halted
        &&
        r.responses.size() == 2
        &&
        r.responses[0].power_code == 245
        &&
        !r.responses[1].failed
        &&
        r.responses[1].power_code == 230;


    check(
        ok,
        "G CONFIG replaces state and resets power"
    );
}


static void test_equality_boundaries()
{
    // --------------------------------------------
    // mean_error == threshold
    // --------------------------------------------

    std::vector<axis_word_t> a;

    a.push_back(
        make_config(
            250,
            180,
            5,

            // exactly 1/16 in Q5.28
            16777216,

            24576,
            0
        )
    );

    a.push_back(
        make_process(
            40,
            1
        )
    );

    a.push_back(
        make_sample_raw(
            0,
            0,
            4096,
            0,
            true
        )
    );

    a.push_back(
        make_end()
    );


    ActivationResult ra =
        run_activation(a);


    bool error_equal_ok =
        !ra.halted
        &&
        ra.responses.size() == 1
        &&
        ra.responses[0].mean_error == 16777216ULL
        &&
        !ra.responses[0].failed;


    // --------------------------------------------
    // clip_count == max_clip_count
    // --------------------------------------------

    std::vector<axis_word_t> b;

    b.push_back(
        make_config(
            250,
            180,
            5,
            2684354560ULL,
            16384,
            1
        )
    );

    b.push_back(
        make_process(
            41,
            2
        )
    );

    b.push_back(
        make_sample_raw(
            0,
            0,
            16384,
            0,
            false
        )
    );

    b.push_back(
        make_sample_raw(
            0,
            0,
            4096,
            0,
            true
        )
    );

    b.push_back(
        make_end()
    );


    ActivationResult rb =
        run_activation(b);


    bool clip_equal_ok =
        !rb.halted
        &&
        rb.responses.size() == 1
        &&
        rb.responses[0].clip_count == 1
        &&
        !rb.responses[0].failed;


    check(
        error_equal_ok
        &&
        clip_equal_ok,
        "H equality boundaries PASS"
    );
}


static void test_negative_power()
{
    std::vector<axis_word_t> v;

    v.push_back(
        make_config(
            0,
            -50,
            30,
            0,
            24576,
            0
        )
    );


    for (
        unsigned int tx = 50;
        tx <= 52;
        ++tx
    ) {

        v.push_back(
            make_process(
                tx,
                1
            )
        );

        v.push_back(
            make_sample_raw(
                0,
                0,
                4096,
                0,
                true
            )
        );
    }


    v.push_back(
        make_end()
    );


    ActivationResult r =
        run_activation(v);


    bool ok =
        !r.halted
        &&
        r.responses.size() == 3
        &&
        r.responses[0].power_code == -30
        &&
        r.responses[1].power_code == -50
        &&
        r.responses[2].power_code == -50;


    check(
        ok,
        "I signed negative TX-power code"
    );
}


// ============================================================
// Error tests
// ============================================================

static void test_process_before_config()
{
    std::vector<axis_word_t> v;

    v.push_back(
        make_process(
            70,
            1
        )
    );


    ActivationResult r =
        run_activation(v);


    check(
        r.halted
        &&
        r.error == ERR_PROCESS_BEFORE_CONFIG
        &&
        r.offending_tx_id == 70,
        "PROCESS before CONFIG"
    );

    check_fresh_recovery(
        "PROCESS-before-CONFIG"
    );
}


static void test_zero_length()
{
    std::vector<axis_word_t> v;

    v.push_back(
        make_config(
            250,
            180,
            5,
            2684355,
            24576,
            0
        )
    );

    v.push_back(
        make_process(
            71,
            0
        )
    );


    ActivationResult r =
        run_activation(v);


    check(
        r.halted
        &&
        r.error == ERR_ZERO_LENGTH_PROCESS
        &&
        r.offending_tx_id == 71,
        "zero-length PROCESS"
    );

    check_fresh_recovery(
        "zero-length PROCESS"
    );
}


static void test_early_tlast()
{
    std::vector<axis_word_t> v;

    v.push_back(
        make_config(
            250,
            180,
            5,
            2684355,
            24576,
            0
        )
    );

    v.push_back(
        make_process(
            72,
            4
        )
    );


    // TLAST arrives on second sample instead of fourth.
    v.push_back(
        make_sample_raw(
            0, 0, 0, 0,
            false
        )
    );

    v.push_back(
        make_sample_raw(
            0, 0, 0, 0,
            true
        )
    );


    ActivationResult r =
        run_activation(v);


    check(
        r.halted
        &&
        r.error == ERR_EARLY_TLAST
        &&
        r.offending_tx_id == 72,
        "early TLAST"
    );

    check_fresh_recovery(
        "early TLAST"
    );
}


static void test_missing_tlast()
{
    std::vector<axis_word_t> v;

    v.push_back(
        make_config(
            250,
            180,
            5,
            2684355,
            24576,
            0
        )
    );

    v.push_back(
        make_process(
            73,
            2
        )
    );

    v.push_back(
        make_sample_raw(
            0, 0, 0, 0,
            false
        )
    );

    // Final expected sample deliberately has TLAST = 0.
    v.push_back(
        make_sample_raw(
            0, 0, 0, 0,
            false
        )
    );


    ActivationResult r =
        run_activation(v);


    check(
        r.halted
        &&
        r.error == ERR_MISSING_TLAST
        &&
        r.offending_tx_id == 73,
        "missing final TLAST"
    );

    check_fresh_recovery(
        "missing TLAST"
    );
}


static void test_bad_configs()
{
    // --------------------------------------------
    // power_step = 0
    // --------------------------------------------

    {
        std::vector<axis_word_t> v;

        v.push_back(
            make_config(
                250,
                180,
                0,
                2684355,
                24576,
                0
            )
        );

        ActivationResult r =
            run_activation(v);

        check(
            r.halted
            &&
            r.error
                ==
                ERR_INVALID_CONFIG_POWER_STEP,
            "invalid CONFIG: zero power step"
        );

        check_fresh_recovery(
            "zero-power-step CONFIG"
        );
    }


    // --------------------------------------------
    // minimum > initial
    // --------------------------------------------

    {
        std::vector<axis_word_t> v;

        v.push_back(
            make_config(
                200,
                210,
                5,
                2684355,
                24576,
                0
            )
        );

        ActivationResult r =
            run_activation(v);

        check(
            r.halted
            &&
            r.error
                ==
                ERR_INVALID_CONFIG_POWER_RANGE,
            "invalid CONFIG: minimum > initial"
        );

        check_fresh_recovery(
            "power-range CONFIG"
        );
    }


    // --------------------------------------------
    // clipping threshold <= 0
    // --------------------------------------------

    {
        std::vector<axis_word_t> v;

        v.push_back(
            make_config(
                250,
                180,
                5,
                2684355,

                // invalid
                0,

                0
            )
        );

        ActivationResult r =
            run_activation(v);

        check(
            r.halted
            &&
            r.error
                ==
                ERR_INVALID_CONFIG_CLIP_THRESHOLD,
            "invalid CONFIG: clipping threshold"
        );

        check_fresh_recovery(
            "clip-threshold CONFIG"
        );
    }
}


static void test_unknown_command()
{
    std::vector<axis_word_t> v;

    v.push_back(
        make_config(
            250,
            180,
            5,
            2684355,
            24576,
            0
        )
    );

    v.push_back(
        make_unknown(
            74
        )
    );


    ActivationResult r =
        run_activation(v);


    check(
        r.halted
        &&
        r.error == ERR_UNKNOWN_COMMAND
        &&
        r.offending_tx_id == 74,
        "unknown command"
    );

    check_fresh_recovery(
        "unknown command"
    );
}


static void test_window_limit()
{
    // ========================================================
    // Legal nsamp = 1024
    // ========================================================

    {
        std::vector<axis_word_t> v;

        v.push_back(
            make_config(
                250,
                180,
                5,
                2684355,
                24576,
                0
            )
        );

        v.push_back(
            make_process(
                100,
                MAX_NSAMP
            )
        );


        for (
            int i = 0;
            i < MAX_NSAMP;
            ++i
        ) {

            v.push_back(
                make_sample_raw(
                    0,
                    0,
                    0,
                    0,
                    i == MAX_NSAMP - 1
                )
            );
        }


        v.push_back(
            make_end()
        );


        ActivationResult r =
            run_activation(v);


        check(
            !r.halted
            &&
            r.responses.size() == 1
            &&
            r.responses[0].mean_error == 0
            &&
            !r.responses[0].failed,
            "maximum legal nsamp = 1024"
        );
    }


    // ========================================================
    // Illegal nsamp = 1025
    // ========================================================

    {
        std::vector<axis_word_t> v;

        v.push_back(
            make_config(
                250,
                180,
                5,
                2684355,
                24576,
                0
            )
        );

        v.push_back(
            make_process(
                101,
                MAX_NSAMP + 1
            )
        );


        ActivationResult r =
            run_activation(v);


        check(
            r.halted
            &&
            r.error == ERR_NSAMP_OUT_OF_RANGE
            &&
            r.offending_tx_id == 101,
            "nsamp > 1024 rejected"
        );

        check_fresh_recovery(
            "out-of-range nsamp"
        );
    }
}


// ============================================================
// Main
// ============================================================

int main()
{
    std::cout
        << "\n"
        << "Adaptive QAM PA Power-Backoff Controller\n"
        << "C-simulation / co-simulation testbench\n"
        << "========================================\n\n";


    test_perfect_constellation();

    test_small_acceptable_error();

    test_excessive_error();

    test_clipping();

    test_repeated_backoff();

    test_minimum_saturation();

    test_reconfiguration();

    test_equality_boundaries();

    test_negative_power();


    std::cout
        << "\nProtocol and error tests\n"
        << "------------------------\n";


    test_process_before_config();

    test_zero_length();

    test_early_tlast();

    test_missing_tlast();

    test_bad_configs();

    test_unknown_command();

    test_window_limit();


    std::cout
        << "\n========================================\n";

    std::cout
        << "Checks run: "
        << tests_run
        << "\n";

    std::cout
        << "Checks failed: "
        << tests_failed
        << "\n";


    if (tests_failed == 0) {

        std::cout
            << "\nALL C++ TESTS PASS\n";

        return 0;
    }


    std::cout
        << "\nC++ TESTS FAILED\n";

    return 1;
}