#=========================================================================
# Constraints file
#-------------------------------------------------------------------------
#
# This file contains various constraints for your chip including the
# target clock period, fanout, transition time and any
# input/output delay constraints.

set_units \
    -capacitance                        ff \
    -time                               ps

# set clock period [ps]
echo "Creating clocks"
create_clock \
    -period                             10000 \
    -name                               master_clk \
    clk

set_clock_uncertainty                   0.02 \
    -hold \
    [all_clocks]

set_clock_uncertainty                   0.02 \
    -setup \
    [all_clocks]

# set input and output delay [ns]
echo "Setting input and output delay for system clk"
set input_ports                         [all_inputs]
set input_ports                         [remove_from_collection ${input_ports} [get_ports clk]]
set input_ports                         [remove_from_collection ${input_ports} [get_ports reset]]
set_input_delay   -max                      0.01 \
    -clock                              master_clk \
    ${input_ports}
set_input_delay   -min                      0.01 \
    -clock                              master_clk \
    ${input_ports}
set output_ports                        [get_ports data_out]
#set output_ports                        [get_ports result]
set_output_delay                            0.01 \
    -clock                              master_clk \
    ${output_ports}

# echo "Setting false paths"

set_load 0.5                            [all_outputs] 

