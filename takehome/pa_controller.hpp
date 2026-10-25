#ifndef PA_CONTROLLER_HPP
#define PA_CONTROLLER_HPP

#include <ap_int.h>
#include <ap_axi_sdata.h>
#include <hls_stream.h>


// ============================================================
// Stream interface
// ============================================================

// 128-bit AXI4-Stream.
//
// One PROCESS sample contains:
//
//   I_ref   : 16 bits
//   Q_ref   : 16 bits
//   I_meas  : 16 bits
//   Q_meas  : 16 bits
//
// so one complete complex sample fits in a single AXI beat.
static const int AXIS_WIDTH = 128;

typedef ap_axiu<AXIS_WIDTH, 0, 0, 0> axis_word_t;


// ============================================================
// Command codes
// ============================================================

enum CommandCode {
    CMD_CONFIG  = 0x01,
    CMD_PROCESS = 0x02,
    CMD_END     = 0x03
};


// ============================================================
// Response codes
// ============================================================

enum ResponseCode {
    RESP_PROCESS = 0x82
};


// ============================================================
// Error codes
//
// These deliberately match golden_model.py.
// ============================================================

enum ErrorCode {
    ERR_NONE                          = 0,
    ERR_PROCESS_BEFORE_CONFIG         = 1,
    ERR_ZERO_LENGTH_PROCESS           = 2,
    ERR_EARLY_TLAST                   = 3,
    ERR_MISSING_TLAST                 = 4,
    ERR_UNKNOWN_COMMAND               = 5,
    ERR_INVALID_CONFIG_POWER_STEP     = 6,
    ERR_INVALID_CONFIG_POWER_RANGE    = 7,
    ERR_INVALID_CONFIG_CLIP_THRESHOLD = 8,
    ERR_NSAMP_OUT_OF_RANGE            = 9
};


// ============================================================
// Design limits
// ============================================================

static const int MAX_NSAMP = 1024;


// ============================================================
// Top-level HLS kernel
// ============================================================

void pa_controller(
    hls::stream<axis_word_t>& in_stream,
    hls::stream<axis_word_t>& out_stream,

    ap_uint<1>& halted,
    ap_uint<8>& error,
    ap_uint<16>& offending_tx_id
);


#endif