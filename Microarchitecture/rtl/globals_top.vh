`ifndef GLOBAL_TOP
`define GLOBAL_TOP

`define STRINGIFY(x)            `"x`"

// Enable tracking package latency and some other timing stats
// `define ON_CHIP_LATENCY_TRACKER_ENABLE
// ASK `define BUFFER_AFTER_ST_ENABLE

// Enable support for flexible flit
//`define FLEXIBLE_FLIT_ENABLE

// Optimization for flexible flit
`ifdef FLEXIBLE_FLIT_ENABLE
`define LOW_LOAD_BYPASS_ENABLE
`define DEST_BYPASS_ENABLE
`define LOOKAHEAD_ROUTING_ENABLE
`endif

`ifndef SYNTHESIS
`define LINK_DELAY                      #(0.002)
`endif

//Flit size
`define RT_FLIT_SIZE 17

//Number of tiles (X,Y)
`define NUM_TILES_X 4
`define NUM_TILES_Y 4

//Hardcoded QSPI related widths
`define QSPI_ID_WIDTH 12
`define QSPI_CMD_WIDTH 4
`define QSPI_ADDR_INCR 2
`define QSPI_ADDR_WIDTH 20
`define QSPI_SIZE_WIDTH 8
`define QSPI_DATA_WIDTH 16
`define QSPI_PACE_ID 12'h13E

package SMARTPkg;

localparam NUM_TILES_X                  = `NUM_TILES_X;
localparam NUM_TILES_Y                  = `NUM_TILES_Y;
localparam NUM_TILES                    = NUM_TILES_X * NUM_TILES_Y;

typedef logic [$clog2(NUM_TILES)-1:0]   TileId;

typedef struct packed
{
    logic [$clog2(NUM_TILES_Y)-1:0]     y;
    logic [$clog2(NUM_TILES_X)-1:0]     x;
} TileXYId;

// LOCAL should always be the last one
localparam NUM_DIRECTIONS               = 5;
typedef enum logic [2:0]
{
    EAST    = 0,
    SOUTH   = 1,
    WEST    = 2,
    NORTH   = 3,
    ALU_T   = 4,
    TREG    = 5,
    LOCAL   = 6,  //ASKmanupa
    INVALID_DIRECTION   = 7   //ASKmanupa
} Direction;

typedef enum logic [5:0]                //ASKmanupa
{
    EAST_ONEHOT     = 6'b000001,
    SOUTH_ONEHOT    = 6'b000010,
    WEST_ONEHOT     = 6'b000100,
    NORTH_ONEHOT    = 6'b001000,
    LOCAL_ONEHOT    = 6'b010000,
    TILE_ONEHOT     = 6'b100000,      //ASKmanupa
    INVALID_DIRECTION_ONEHOT    = 6'b000000
} DirectionOneHot;

typedef enum logic [1:0]
{
    TURN_STRAIGHT   = 0,
    TURN_LEFT       = 1,
    TURN_RIGHT      = 2,
    TURN_LOCAL      = 3
} DirectionTurn;

typedef logic [39:0]                    Time;

//------------------------------------------------------------------------------
// SMART-path Setup Request
//------------------------------------------------------------------------------
`ifdef NUM_HOPS_PER_CYCLE
localparam NUM_HOPS_PER_CYCLE           = `NUM_HOPS_PER_CYCLE;
`else
localparam NUM_HOPS_PER_CYCLE           = 1;
`endif

typedef logic [$clog2(NUM_HOPS_PER_CYCLE+1)-1:0]
                                        SMARTHop;

typedef struct packed
{
    `ifdef DEST_BYPASS_ENABLE
    logic                               is_dest;
    `endif
    SMARTHop                            num_hops;
} SMARTSetupRequest;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Flit
//------------------------------------------------------------------------------

// User-specified width

// Multiple lane enabled
localparam NUM_LANES                    = 1;
localparam FLIT_FIXED_PARITY_WIDTH      = 1;
localparam FLIT_FIXED_PHYSICAL_WIDTH    = 9;

localparam FLIT_FIXED_MAX_NUM_TURNS     = 3;
localparam FLIT_DATA_PARITY_WIDTH       = FLIT_FIXED_PARITY_WIDTH;

`ifdef FLEXIBLE_FLIT_ENABLE
localparam FLIT_FIXED_TO_FLEXIBLE_MULTIPLIER    = 2;
localparam FLIT_FLEXIBLE_PARITY_WIDTH   = FLIT_FIXED_PARITY_WIDTH * FLIT_FIXED_TO_FLEXIBLE_MULTIPLIER;
localparam FLIT_FLEXIBLE_PHYSICAL_WIDTH = FLIT_FIXED_PHYSICAL_WIDTH * FLIT_FIXED_TO_FLEXIBLE_MULTIPLIER;
`endif

// Lane controls
typedef logic [NUM_LANES-1:0]           LaneMask;
typedef logic [$clog2(NUM_LANES)-1:0]   LaneId;

typedef enum logic
{
    FIXED       = 0,
    FLEXIBLE    = 1
} LaneMode;

typedef struct packed
{
    LaneId                              master_id;
    logic                               is_master;
    LaneMode                            mode;
    logic                               enable;
} LaneConfig;

// Basic fields
typedef enum logic
{
    FT_INVALID  = 0,
    FT_VALID    = 1
} FlitType;

typedef logic [5:0]                     FlitFixedTime;
typedef logic [11:0]                    FlitFlexibleTime;
typedef logic [19:0]                    FlitId;
typedef logic [`RT_FLIT_SIZE-1:0]       FlitFixedData;

`ifdef DEBUG_ENABLE
typedef struct packed
{
    TileXYId                            src_xy_id;
    TileXYId                            dest_xy_id;
    LaneId                              lane_id;
    FlitId                              flit_id;
    `ifndef ON_CHIP_LATENCY_TRACKER_ENABLE
    Time                                gen_time;
    `endif
} FlitDebug;
localparam FLIT_DEBUG_WIDTH = $bits(FlitDebug);
`else
localparam FLIT_DEBUG_WIDTH = 0;
`endif

// FlitFixed
typedef struct packed
{
    DirectionTurn [FLIT_FIXED_MAX_NUM_TURNS-1:0]
                                        turn;
    FlitType                            flit_type;
} FlitFixedHeader;

localparam FLIT_FIXED_WIDTH             = FLIT_FIXED_PHYSICAL_WIDTH + FLIT_DEBUG_WIDTH;
localparam FLIT_FIXED_CONTENT_WIDTH     = FLIT_FIXED_WIDTH - $bits(FlitFixedHeader) - FLIT_FIXED_PARITY_WIDTH;
`ifdef ON_CHIP_LATENCY_TRACKER_ENABLE
localparam FLIT_FIXED_REMAINDER_WIDTH   = FLIT_FIXED_CONTENT_WIDTH - $bits(FlitFixedTime) 
                                        - FLIT_DEBUG_WIDTH;
`else
localparam FLIT_FIXED_REMAINDER_WIDTH   = FLIT_FIXED_CONTENT_WIDTH - FLIT_DEBUG_WIDTH;
`endif

typedef struct packed
{
    `ifdef DEBUG_ENABLE
    FlitDebug                           debug;
    `endif

    logic [FLIT_FIXED_REMAINDER_WIDTH-1:0]
                                        remainder;
    `ifdef ON_CHIP_LATENCY_TRACKER_ENABLE
    FlitFixedTime                       gen_time;
    `endif
} FlitFixedContent;

typedef struct packed
{
    FlitFixedData                     data;
} FlitFixed;

// FlitData (no header)
localparam FLIT_DATA_WIDTH              = FLIT_FIXED_WIDTH;
localparam FLIT_DATA_CONTENT_WIDTH      = FLIT_DATA_WIDTH - FLIT_DATA_PARITY_WIDTH;
localparam FLIT_DATA_REMAINDER_WIDTH    = FLIT_DATA_CONTENT_WIDTH - FLIT_DEBUG_WIDTH;

typedef struct packed
{
    `ifdef DEBUG_ENABLE
    FlitDebug                           debug;
    `endif
    logic [FLIT_DATA_REMAINDER_WIDTH-1:0]
                                        remainder;
} FlitDataContent;

typedef struct packed
{
    logic [FLIT_DATA_PARITY_WIDTH-1:0]  parity;
    FlitDataContent                     content;
} FlitData;

`ifdef FLEXIBLE_FLIT_ENABLE
// FlitFlexible
typedef struct packed
{
    `ifdef LOOKAHEAD_ROUTING_ENABLE
    SMARTSetupRequest                   lookahead_ssr;
    DirectionOneHot                     lookahead_route;
    `endif
    FlitType                            flit_type;
} FlitFlexibleHeader;

localparam FLIT_FLEXIBLE_DEBUG_WIDTH    = FLIT_DEBUG_WIDTH * FLIT_FIXED_TO_FLEXIBLE_MULTIPLIER;
localparam FLIT_FLEXIBLE_WIDTH          = FLIT_FLEXIBLE_PHYSICAL_WIDTH + FLIT_FLEXIBLE_DEBUG_WIDTH;
localparam FLIT_FLEXIBLE_CONTENT_WIDTH  = FLIT_FLEXIBLE_WIDTH 
                                        - $bits(FlitFlexibleHeader) - FLIT_FLEXIBLE_PARITY_WIDTH;
`ifdef ON_CHIP_LATENCY_TRACKER_ENABLE
localparam FLIT_FLEXIBLE_REMAINDER_WIDTH    = FLIT_FLEXIBLE_CONTENT_WIDTH
                                            - $bits(TileXYId) - $bits(TileXYId) - $bits(FlitFlexibleTime) 
                                            - FLIT_FLEXIBLE_DEBUG_WIDTH;
`else
localparam FLIT_FLEXIBLE_REMAINDER_WIDTH    = FLIT_FLEXIBLE_CONTENT_WIDTH
                                            - $bits(TileXYId) - $bits(TileXYId) - FLIT_FLEXIBLE_DEBUG_WIDTH;
`endif

typedef struct packed
{
    `ifdef DEBUG_ENABLE
    // Multiple debug message to maintain the same flit width when debug is on
    FlitDebug [FLIT_FIXED_TO_FLEXIBLE_MULTIPLIER-1:0]
                                        debug;
    `endif

    logic [FLIT_FLEXIBLE_REMAINDER_WIDTH-1:0]
                                        remainder;
    `ifdef ON_CHIP_LATENCY_TRACKER_ENABLE
    FlitFlexibleTime                    gen_time;
    `endif
    TileXYId                            dest_xy_id;
    TileXYId                            src_xy_id;
} FlitFlexibleContent;

typedef struct packed
{
    logic [FLIT_FLEXIBLE_PARITY_WIDTH-1:0]
                                        parity;
    FlitFlexibleContent                 content;
    FlitFlexibleHeader                  header;
} FlitFlexible;
`endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Credit
//------------------------------------------------------------------------------
typedef logic                           Credit;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Traffic
//------------------------------------------------------------------------------
localparam MAX_NUM_FLITS                = 1 << 31;
localparam LOG_MAX_NUM_FLITS            = $clog2(MAX_NUM_FLITS);
localparam LFSR_SEND_WIDTH              = 20;
localparam LFSR_DEST_WIDTH              = 20;
localparam LFSR_DATA_WIDTH              = 15;
localparam SEND_PRECISION               = 10;

typedef enum logic [1:0]
{
    TRAFFIC_TYPE_UNIFORM_RANDOM,
    TRAFFIC_TYPE_TRANSPOSE,
    TRAFFIC_TYPE_BIT_COMPLEMENT,
    TRAFFIC_TYPE_FIXED_DEST
} TrafficType;

typedef struct packed
{
    Time                                                        max_gen_time;
    logic [NUM_LANES-1:0][SEND_PRECISION-1:0]                   send_threshold;
    logic [NUM_LANES-1:0][LOG_MAX_NUM_FLITS-1:0]                max_num_flits_sent;

    `ifdef FLEXIBLE_FLIT_ENABLE
    // For flexible lane
    TrafficType                                                 traffic_type;
    TileXYId                                                    fixed_dest_xy_id;
    `endif

    // For fixed lane
    DirectionTurn [NUM_LANES-1:0][FLIT_FIXED_MAX_NUM_TURNS-1:0] lane_turns;
    `ifdef DEBUG_ENABLE
    TileXYId [NUM_LANES-1:0]                                    lane_dest_xy_id;
    `endif
} TrafficConfig;

typedef struct packed
{
    logic [NUM_LANES-1:0][LFSR_SEND_WIDTH-1:0]  send_seed;
    logic [NUM_LANES-1:0][LFSR_DATA_WIDTH-1:0]  data_seed;
    logic [LFSR_DEST_WIDTH-1:0]                 dest_seed;         // For flexible lane
} LFSRConfig;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Useful functions
//------------------------------------------------------------------------------
`ifdef DEBUG_ENABLE
function TileId convert_2d_to_1d;
input TileXYId                          xy_id;

convert_2d_to_1d = xy_id.y * NUM_TILES_X + xy_id.x;
endfunction

function string flit_fixed_to_string;
input FlitFixed                         flit;
TileId                                  src_id, dest_id;
LaneId                                  lane_id;
FlitId                                  flit_id;

src_id  = convert_2d_to_1d(flit.content.debug.src_xy_id);
dest_id = convert_2d_to_1d(flit.content.debug.dest_xy_id);
lane_id = flit.content.debug.lane_id;
flit_id = flit.content.debug.flit_id;

$sformat(flit_fixed_to_string, "%0d --> %0d (%0d, %0d)", src_id, dest_id, lane_id, flit_id);
endfunction

function string flit_data_to_string;
input FlitData                          flit;
TileId                                  src_id, dest_id;
LaneId                                  lane_id;
FlitId                                  flit_id;

src_id  = convert_2d_to_1d(flit.content.debug.src_xy_id);
dest_id = convert_2d_to_1d(flit.content.debug.dest_xy_id);
lane_id = flit.content.debug.lane_id;
flit_id = flit.content.debug.flit_id;

$sformat(flit_data_to_string, "%0d --> %0d (%0d, %0d)", src_id, dest_id, lane_id, flit_id);
endfunction

`ifdef FLEXIBLE_FLIT_ENABLE
function string flit_flexible_to_string;
input FlitFlexible                      flit;
TileId                                  src_id, dest_id;
LaneId                                  lane_id;
FlitId                                  flit_id;

src_id  = convert_2d_to_1d(flit.content.debug[0].src_xy_id);
dest_id = convert_2d_to_1d(flit.content.debug[0].dest_xy_id);
lane_id = flit.content.debug[0].lane_id;
flit_id = flit.content.debug[0].flit_id;

$sformat(flit_flexible_to_string, "%0d --> %0d (%0d, %0d)", src_id, dest_id, lane_id, flit_id);
endfunction
`endif
`endif
//------------------------------------------------------------------------------

endpackage


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

`endif
