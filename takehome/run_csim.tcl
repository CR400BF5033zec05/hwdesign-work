# ============================================================
# Adaptive QAM PA Power-Backoff Controller
# Vitis HLS C-simulation flow
# ============================================================

open_project -reset pa_controller_prj

set_top pa_controller

add_files pa_controller.cpp
add_files pa_controller.hpp

add_files -tb testbench.cpp


open_solution -reset solution1

# Course-required FPGA
set_part {xc7z020clg400-1}

# Course-required 10 ns clock = 100 MHz
create_clock -period 10 -name default


# ============================================================
# C simulation only
# ============================================================

csim_design


exit