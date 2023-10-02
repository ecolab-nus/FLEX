##TODO
# Add parallel I/O addr and data sizes

# Data Memory (DM)
DM_NUM_BLOCKS = 4
DM_NUM_PORTS_PER_BLOCK = 2
DM_NUM_PORTS_TOTAL = DM_NUM_BLOCKS * DM_NUM_PORTS_PER_BLOCK
DM_BLOCK_DEPTH = 256

# Config Memory (CM)
CM_DEPTH = 8

# Tiles
TILES_NUM_ROWS = 4
TILES_NUM_COLS = 4
TILES_NUM_TOTAL = TILES_NUM_ROWS * TILES_NUM_COLS


###################################################
# APP CONFIGS - Configure before executing automate
clusters_cm_to_write = [0] # writes to CM of all tiles in cluster
dmems_to_write = [
    0 , 1, 2, 3,
    4, 5, 6, 7]
dmem_range_to_write_zeroes = [[0, DM_BLOCK_DEPTH*2-1]] # index [x][1] is not inclusive.
dmem_exec_end_line = "8190,0,1" # None is not to write. address 16382 -> row 8191, byte 0

NOP_INST = "0000000000000000000000000000000000000000000111111111111111111111"

WRITE_CM = True
WRITE_DM = True
WRITE_LUT = True
WRITE_CLUS_EN = True
###################################################

# Bit Widths
DATA_WIDTH = 16
DM_WIDTH = 16
CM_WIDTH = 64

CM_BYTE_SEL_BITS = 2 # 
CM_ROW_SEL_BITS = CM_DEPTH.bit_length()-1 # log2(CM_DEPTH)
CM_SEL_BITS = (TILES_NUM_TOTAL-1).bit_length() # log2(TILES_NUM_TOTAL)

DM_BYTE_SEL_BITS = 1
DM_ROW_SEL_BITS = (DM_BLOCK_DEPTH-1).bit_length() # log2(DM_BLOCK_DEPTH)
DM_SEL_BITS = (DM_NUM_PORTS_TOTAL-1).bit_length() # log2(DM_NUM_PORTS_TOTAL)

###################################################
# Addressing Indexes
# Edit ADDR_WIDTH and LSBs of the addressing segments.
ADDR_MEM_TYPE_ENCODING = {
    "CM": "0000",
    "DM": "0001",
    "TRIGGER": "0100",
    "LUT": "10",
    "CLUS_EN": "11",
    "LOCAL": "1110",
    "CLKENOP":"0101",
    "CLKENR":"1001"
}

ADDR_WIDTH = 19 # [18:0]
ADDR_MEM_TYPE_SEL = 15 # addr[18:17]
# assumed always 2 bits for ADDR_MEM_TYPE_SEL

ADDR_CM_BYTE_SEL = 1 # addr[2:1]
ADDR_CM_ROW_SEL = 3 # addr[7:3]
ADDR_CM_SEL = 8 # addr[13:8]

ADDR_DM_BYTE_SEL = 0 # addr[0]
ADDR_DM_ROW_SEL = 1 # addr[12:1]
ADDR_DM_SEL = 12 # addr[16:13]

# don't need to care about this below: just some indexing calculations
ADDR_MSB_ZEROES = (ADDR_WIDTH-ADDR_MEM_TYPE_SEL-4) * '0'

ADDR_CM_TOP_ZEROES = (ADDR_MEM_TYPE_SEL - (ADDR_CM_SEL+CM_SEL_BITS)) * '0'
ADDR_CM_MID1_ZEROES = (ADDR_CM_SEL - (ADDR_CM_ROW_SEL+CM_ROW_SEL_BITS)) * '0'
ADDR_CM_MID2_ZEROES = (ADDR_CM_ROW_SEL - (ADDR_CM_BYTE_SEL+CM_BYTE_SEL_BITS)) * '0'
ADDR_CM_LSB_ZEROES = ADDR_CM_BYTE_SEL * '0'

ADDR_DM_TOP_ZEROES = (ADDR_MEM_TYPE_SEL - (ADDR_DM_SEL+ DM_SEL_BITS)) * '0'
ADDR_DM_MID1_ZEROES = (ADDR_DM_SEL - (ADDR_DM_ROW_SEL+DM_ROW_SEL_BITS)) * '0'
ADDR_DM_MID2_ZEROES = (ADDR_DM_ROW_SEL - (ADDR_DM_BYTE_SEL+DM_BYTE_SEL_BITS)) * '0'
ADDR_DM_LSB_ZEROES = ADDR_DM_BYTE_SEL * '0'

ADDR_TRIGGER_TOP_ZEROS =  (ADDR_MEM_TYPE_SEL - CM_SEL_BITS) * '0'
# ADDR_MSB_ZEROES - ADDR_MEM_TYPE_SEL - ADDR_CM_TOP_ZEROES - ADDR_CM_SEL - ADDR_MID1_ZEROES - ADDR_CM_ROW_SEL - ADDR_MID2_ZEROES - ADDR_CM_BYTE_SEL - ADDR_CM_LSB_ZEROES
print('DM_SEL_BITS->',DM_SEL_BITS,'DM_ROW_SEL_BITS->',DM_ROW_SEL_BITS,'DM_SEL_BITS->',DM_SEL_BITS)

LOCAL_REG_DEPTH = 8
LOCAL_ROW_SEL_BITS = LOCAL_REG_DEPTH.bit_length()-1 
ADDR_LOCAL_TOP_ZEROES = (ADDR_MEM_TYPE_SEL - LOCAL_ROW_SEL_BITS - CM_SEL_BITS) * '0'
CLKENOP_INDEX_SEL_BITS = CM_DEPTH.bit_length()
