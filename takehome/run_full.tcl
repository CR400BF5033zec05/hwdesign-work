# ============================================================
# Adaptive QAM PA Power-Backoff Controller
# Full Vitis HLS verification flow
#
#   1. C simulation
#   2. HLS synthesis
#   3. RTL co-simulation
# ============================================================

open_project -reset pa_controller_prj

set_top pa_controller

add_files pa_controller.cpp
add_files pa_controller.hpp

add_files -tb testbench.cpp


open_solution -reset solution1

# Course-required FPGA
set_part {xc7z020clg400-1}

# Course-required clock
create_clock -period 10 -name default


# ============================================================
# Step 1: C simulation
# ============================================================

csim_design


# ============================================================
# Step 2: C/C++ -> RTL synthesis
# ============================================================

csynth_design


# ============================================================
# Step 3: RTL co-simulation using same C++ testbench
# ============================================================

cosim_design -rtl verilog


exit