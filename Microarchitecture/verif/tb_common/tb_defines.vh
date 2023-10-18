`ifndef _tb_defines_vh_
`define _tb_defines_vh_

//TB defines
`define TB_DRIVER       tb_top.tb_driver

//Clock settings (ns)
`define SYS_CLK_PERIOD 10

//Register/signal defines
`define SOC_TOP    tb_top.soc_pad
`define HYC_TOP    tb_top.soc_pad.flex0

// Architecture -specific constants
`define ADDR_WIDTH 19
`define DATA_WIDTH 16

`define TB_NAME                 tb_top
`define TB_NUM_SIM_CYCLES       2000


`endif //_tb_defines_vh_

