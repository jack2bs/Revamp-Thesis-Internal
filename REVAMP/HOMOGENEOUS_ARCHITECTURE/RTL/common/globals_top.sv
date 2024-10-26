
`timescale 1ns/1ps

`define STRINGIFY(x)            `"x`"

package TopPkg;

// Mux type
typedef enum logic [1:0]
{
    MUX_BINARY,
    MUX_ONEHOT,
    MUX_ONEHOT_TRI
} MuxType;

// Arbiter type
typedef enum logic
{
    ARB_MATRIX,
    ARB_FIXED_PRIORITY
} ArbiterType;

// Scan chain
typedef struct packed
{
    logic                       clk_master;
    logic                       clk_slave;
    logic                       reset;
    logic                       enable;
    logic                       update;
} ScanControl;

endpackage

