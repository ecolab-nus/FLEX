import re
import collections
import subprocess
import sys
import argparse

## PYTHON FILE with constants
import flex_config as FLEX

ins_inp  =  str(sys.argv[1])
data_inp =  str(sys.argv[2])
TIMEEXEC = int(sys.argv[3])
local_inp = str(sys.argv[4])
vec_size = str(sys.argv[5])


cm_addr_file = open("./mem_files/addr_ins.trc","w+")
cm_addr_trig_file = open("./mem_files/addr_trig.trc","w+")
cm_addr_clken_file = open("./mem_files/addr_clken.trc","w+")
local_addr_file = open("./mem_files/addr_local.trc","w+")

dm_addr_file = open("./mem_files/addr_data.trc","w+")
dm_data_file = open("./mem_files/data.trc","w+")
iter_data_file = open("./mem_files/iter_data.trc","w+")
loopstart_data_file = open("./mem_files/loopstart_data.trc","w+")
clken_data_file = open("./mem_files/clken_data.trc","w+")
clken_op_data_file = open("./mem_files/clken_op_data.trc","w+")
clken_r_data_file = open("./mem_files/clken_r_data.trc","w+")

results_expected_file = open("results_expected.trc", "w+")

dm_src_read_file = open(data_inp,"r")

######################################
# CM BINARY
######################################

###### step 1: split CM instructions by tiles
num_inst_per_tile = 0
for i in range(FLEX.TILES_NUM_ROWS):
    for j in range(FLEX.TILES_NUM_COLS):

        tile_num = (i*FLEX.TILES_NUM_ROWS) + j

        cmdtorun = "grep -i -A "+ str(TIMEEXEC) + " \"\[CONFIGURATION\] Y=" + str(i) + " " + "X=" + str(j) + "\" " + ins_inp + " | tail -n +3" + " >" + " ./mem_files/tile" + str(tile_num) + "_inst.trc"
        make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        (output, err) = make_process.communicate()

##### step 2: prune instructions, convert 64b to 16b
for i in range(FLEX.TILES_NUM_ROWS):
    for j in range(FLEX.TILES_NUM_COLS):
        tile_num = (i*FLEX.TILES_NUM_ROWS) + j  

        cmdtorun = "sed -i " + "\'s/Y=" + str(i) + " " + "X=" + str(j) + ",//\'" + " ./mem_files/tile" + str(tile_num) + "_inst.trc"
        make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        (output, err) = make_process.communicate()

        #### Append NOPs to fill until cubetime
        cmdtorun = "grep -c ^" +  " ./mem_files/tile" + str(tile_num) + "_inst.trc"
        make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        (output, err) = make_process.communicate()
        line_count = int(output)

        line_count_diff = (TIMEEXEC-1) - int(line_count)
        if(line_count_diff > 0):
            for count in range(0, line_count_diff):
                cmdtorun = "echo \\" + FLEX.NOP_INST + " >> " + " ./mem_files/tile" + str(tile_num) + "_inst.trc"
                make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
                (line_count, err) = make_process.communicate()

        split_iterations = int(FLEX.CM_WIDTH/FLEX.DATA_WIDTH)
        cmdtorun = "sed -i \'"
        for x in range(1, split_iterations):
           if(x!=1):
                cmdtorun = cmdtorun + ";"
            cmdtorun = cmdtorun + "s/./&\\n/" + str(x*FLEX.DATA_WIDTH + (x-1))
        cmdtorun = cmdtorun + "\' ./mem_files/tile" + str(tile_num) + "_inst.trc" 

        make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        (output, err) = make_process.communicate()

####### step 3: GENERATE CM binaries
cmdtorun = "cat "
for i in range(FLEX.TILES_NUM_ROWS):
    for j in range(FLEX.TILES_NUM_COLS):
        tile_num = (i*FLEX.TILES_NUM_ROWS) + j
        cmdtorun = cmdtorun + "./mem_files/tile" + str(tile_num) + "_inst.trc "
cmdtorun = cmdtorun + "> ./mem_files/ins.trc"

make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
(output, err) = make_process.communicate()

if(FLEX.WRITE_CM):
    addr_mem_type = FLEX.ADDR_MEM_TYPE_ENCODING["CM"]
    SRAMCELL = int(FLEX.CM_WIDTH/FLEX.DATA_WIDTH)

    addr_tile_sel_format = '0' + str(FLEX.CM_SEL_BITS) + 'b'
    addr_row_sel_format = '0' + str(FLEX.CM_ROW_SEL_BITS) + 'b'
    addr_byte_sel_format = '0' + str(FLEX.CM_BYTE_SEL_BITS) + 'b'

    for i in range(FLEX.TILES_NUM_ROWS):
        for j in range(FLEX.TILES_NUM_COLS):
            tile_num = (i*FLEX.TILES_NUM_ROWS) + j
            for k in range(0,TIMEEXEC-1):
                for l in range(SRAMCELL-1, -1, -1):
                    addr_tile_sel = format(tile_num, addr_tile_sel_format)
                    addr_row_sel = format(k,addr_row_sel_format)
                    addr_byte_sel = format(l,addr_byte_sel_format)

                    totaladdress = FLEX.ADDR_MSB_ZEROES + addr_mem_type + FLEX.ADDR_CM_TOP_ZEROES + addr_tile_sel + FLEX.ADDR_CM_MID1_ZEROES + addr_row_sel + FLEX.ADDR_CM_MID2_ZEROES + addr_byte_sel + FLEX.ADDR_CM_LSB_ZEROES
                    cm_addr_file.write(totaladdress + "\n")

cm_addr_file.close()

###### step 1: split CM instructions by tiles
num_inst_per_tile = 0
for i in range(FLEX.TILES_NUM_ROWS):
    for j in range(FLEX.TILES_NUM_COLS):

        tile_num = (i*FLEX.TILES_NUM_ROWS) + j

        cmdtorun = "grep -i -A "+ str(FLEX.LOCAL_REG_DEPTH) + " \"\[LOCAL_REG\] Y=" + str(i) + " " + "X=" + str(j) + "\" " + local_inp + " | tail -n +2" + " >" + " ./mem_files/tile" + str(tile_num) + "_local_inst.trc"
        make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        (output, err) = make_process.communicate()

cmdtorun = "cat "
for i in range(FLEX.TILES_NUM_ROWS):
    for j in range(FLEX.TILES_NUM_COLS):
        tile_num = (i*FLEX.TILES_NUM_ROWS) + j
        cmdtorun = cmdtorun + "./mem_files/tile" + str(tile_num) + "_local_inst.trc "
cmdtorun = cmdtorun + "> ./mem_files/local.trc"

make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
(output, err) = make_process.communicate()


if(FLEX.WRITE_CM):
    addr_mem_type = FLEX.ADDR_MEM_TYPE_ENCODING["LOCAL"]

    addr_tile_sel_format = '0' + str(FLEX.CM_SEL_BITS) + 'b'
    addr_row_sel_format = '0' + str(FLEX.LOCAL_ROW_SEL_BITS) + 'b'

    for i in range(FLEX.TILES_NUM_ROWS):
        # For each tile
        for j in range(FLEX.TILES_NUM_COLS):
            tile_num = (i*FLEX.TILES_NUM_ROWS) + j
            # For each row
            for k in range(0,FLEX.LOCAL_REG_DEPTH):
                 addr_tile_sel = format(tile_num, addr_tile_sel_format)
                 addr_row_sel = format(k,addr_row_sel_format)
     
                 totaladdress = addr_mem_type + FLEX.ADDR_LOCAL_TOP_ZEROES + addr_tile_sel + addr_row_sel
                 local_addr_file.write(totaladdress + "\n")

local_addr_file.close()

######################################
# DM BINARY
######################################
if(FLEX.WRITE_DM):
    addr_mem_type = FLEX.ADDR_MEM_TYPE_ENCODING["DM"]

    dataToWrite = {}
    dataVal = {}
    dataVal_Ordered = {}
    dataToWrite_backup = {}
    lines_n = []
    lines = dm_src_read_file.readlines()
    #print("automate.py [INFO]: DM details  size-> ",len(lines))
    for line in lines:
        allField = line.split(',')
        match = re.match(r"([0-9]+)", allField[0], re.I)
        if(match):
            allField_dm =  int((int(allField[0])//(FLEX.DATA_WIDTH/8)) // FLEX.DM_BLOCK_DEPTH) # DM row index // DM depth
            allField_RowandByte = int((int(allField[0])) % (FLEX.DM_BLOCK_DEPTH * (FLEX.DATA_WIDTH/8)))
            key1 = 2*allField_dm;
            key2 = 2*allField_dm+1;
            if key1 not in dataToWrite.keys():
                dataToWrite[key1] = []
                dataToWrite_backup[key1] = []

            if key2 not in dataToWrite.keys():
                dataToWrite[key2] = []
                dataToWrite_backup[key2]= []

            dataToWrite_backup[key1].append(allField_RowandByte)
            dataToWrite_backup[key2].append(allField_RowandByte)

            dataToWrite[key1].append([allField_RowandByte, allField[1], allField[2]])
            dataToWrite[key2].append([allField_RowandByte, allField[1], allField[2]])

 
    for dm_index in dataToWrite.keys():
        #print("automate.py [INFO]: DM details  dm_index-> ",dm_index) 
        dataVal[dm_index] = {}
        dataVal_Ordered[dm_index] = {}

        for custom_range in FLEX.dmem_range_to_write_zeroes:
            for i in range(custom_range[0], custom_range[1]):
                if i not in dataToWrite_backup[dm_index]:
                    dataToWrite[dm_index].append([str(i), "0", "0"])

    for dm in dataToWrite:
        for i in dataToWrite[dm]:
            byte = int(i[0])
            row = int(i[0])//2
            if row not in dataVal[dm].keys():
                dataVal[dm][row] = [0, 0, 0, 0] # [LSB pre-run, LSB post-run, MSB pre-run, MSB post-run]
            if(byte%2):
                dataVal[dm][row][2] = int(i[1]) # pre-run data (byte)
                dataVal[dm][row][3] = int(i[2]) # post-run data (byte)
            else:
                dataVal[dm][row][0] = int(i[1]) # pre-run data (byte)
                dataVal[dm][row][1] = int(i[2]) # post-run data (byte)

        dataVal_Ordered[dm] = collections.OrderedDict(sorted(dataVal[dm].items()))

    '''
    # After sorting, dataVal_Ordered will be as follows:
    # OrderedDict([(0, [pre-LSB, post-LSB, pre-MSB, post-MSB]), 
    #              (2, [pre-LSB, post-LSB, pre-MSB, post-MSB]),
    #               ...])
    # pre: pre-run data
    # post: post-run data
    '''
    addr_dm_sel_format = '0' + str(FLEX.DM_SEL_BITS) + 'b'
    addr_row_sel_format = '0' + str(FLEX.DM_ROW_SEL_BITS) + 'b'
    addr_byte_sel_format = '0' + str(FLEX.DM_BYTE_SEL_BITS) + 'b'
    for dm in dataToWrite:
        if ((dm %2)==0): 
            addr_dm_sel = format(dm, addr_dm_sel_format)
            for i in dataVal_Ordered[dm].keys():
                addr_row_sel = format(i, addr_row_sel_format)
                addr_byte_sel = format(0, addr_byte_sel_format)
                totaladdress = FLEX.ADDR_MSB_ZEROES + addr_mem_type + FLEX.ADDR_DM_TOP_ZEROES + addr_dm_sel + FLEX.ADDR_DM_MID1_ZEROES + addr_row_sel + FLEX.ADDR_DM_MID2_ZEROES + addr_byte_sel + FLEX.ADDR_DM_LSB_ZEROES

                dm_addr_file.write(totaladdress + "\n")

                ValinKey = dataVal_Ordered[dm][i]

                pre_dataLSB = ValinKey[0]
                post_dataLSB = ValinKey[1]
                pre_dataMSB = ValinKey[2]
                post_dataMSB = ValinKey[3]

                pre_data2file = format(pre_dataMSB,'08b') + format(pre_dataLSB,'08b')
                post_data2file = format(post_dataMSB,'08b') + format(post_dataLSB,'08b')
                dm_data_file.write(pre_data2file + "\n")
                results_expected_file.write(post_data2file + "\n")
dm_addr_file.close()
dm_data_file.close()
results_expected_file.close()

######################################
# TRIGGER
######################################

###### step 1: split CM instructions by tiles
num_inst_per_tile = 0
for i in range(FLEX.TILES_NUM_ROWS):
    for j in range(FLEX.TILES_NUM_COLS):

        tile_num = (i*FLEX.TILES_NUM_ROWS) + j  
        # write CM files   
        cmdtorun = "grep -i \"\[TRIGGER\] Y=" + str(i) + " " + "X=" + str(j) + "\" " + ins_inp + " | grep -o \'[^:]\+$\'" + " >" + " ./mem_files/tile" + str(tile_num) + "_trigger.trc"
        make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        (output, err) = make_process.communicate()

####### step 3: GENERATE CM binaries
cmdtorun = "cat "
for i in  range(FLEX.TILES_NUM_ROWS):
    for j in range(FLEX.TILES_NUM_COLS):
        tile_num = (i*FLEX.TILES_NUM_ROWS) + j
        cmdtorun = cmdtorun + "./mem_files/tile" + str(tile_num) + "_trigger.trc "
cmdtorun = cmdtorun + "> ./mem_files/trigger.trc"

make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
(output, err) = make_process.communicate()

if(FLEX.WRITE_CM):
    addr_mem_type = FLEX.ADDR_MEM_TYPE_ENCODING["TRIGGER"]

    addr_tile_sel_format = '0' + str(FLEX.CM_SEL_BITS) + 'b'

    for i in range(FLEX.TILES_NUM_ROWS):
        # For each tile
        for j in range(FLEX.TILES_NUM_COLS):
            tile_num = (i*FLEX.TILES_NUM_ROWS) + j
            addr_tile_sel = format(tile_num, addr_tile_sel_format)
            totaladdress = addr_mem_type + FLEX.ADDR_TRIGGER_TOP_ZEROS + addr_tile_sel
            cm_addr_trig_file.write(totaladdress + "\n")

cm_addr_trig_file.close()

######################################
# CLK_EN
######################################

###### step 1: split CM instructions by tiles
num_inst_per_tile = 0
for i in range (TIMEEXEC):
    # write CM files   
    cmdtorun = "grep -i \"\[CLKENCMEM\] T=" + str(i) + "\" " + ins_inp + " | grep -o \'[^:]\+$\'" + " >" + " ./mem_files/clkenop_" + str(i) + ".trc"
    cmdtorun1 = "grep -i \"\[CMEMROUTEEN\] T=" + str(i) + "\" " + ins_inp + " | grep -o \'[^:]\+$\'" + " >" + " ./mem_files/clkenr_" + str(i) + ".trc"
    make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
    (output, err) = make_process.communicate()
    make_process = subprocess.Popen(cmdtorun1,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
    (output, err) = make_process.communicate()

####### step 3: GENERATE CM binaries
cmdtorun = "cat "
cmdtorun1 = "cat "
for i in range (TIMEEXEC-1):
    cmdtorun = cmdtorun + " ./mem_files/clkenop_" + str(i+1) + ".trc"
    cmdtorun1 = cmdtorun1 + " ./mem_files/clkenr_" + str(i+1) + ".trc"
cmdtorun = cmdtorun + "> ./mem_files/clkenop.trc"
cmdtorun1 = cmdtorun1 + "> ./mem_files/clkenr.trc"
make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
(output, err) = make_process.communicate()
make_process = subprocess.Popen(cmdtorun1,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
(output, err) = make_process.communicate()

addr_index_sel_format = '0' + str(FLEX.CLKENOP_INDEX_SEL_BITS) + 'b'
addr_mem_type = FLEX.ADDR_MEM_TYPE_ENCODING["CLKENOP"]

for j in range (TIMEEXEC-1):
    addr_tile_sel = format(j, addr_index_sel_format)
    totaladdress = addr_mem_type + "000000000" + addr_tile_sel + "00"
    cm_addr_clken_file.write(totaladdress + "\n")

addr_mem_type = FLEX.ADDR_MEM_TYPE_ENCODING["CLKENR"]

for j in range (TIMEEXEC-1):
    addr_tile_sel = format(j, addr_index_sel_format)
    totaladdress = addr_mem_type + "000000000" + addr_tile_sel + "00"
    cm_addr_clken_file.write(totaladdress + "\n")

cm_addr_clken_file.close()


######################################
# Collate for TOTALDATA, TOTALADDR
######################################
cmdtorun = "cat ./mem_files/ins.trc ./mem_files/data.trc ./mem_files/local.trc ./mem_files/trigger.trc ./mem_files/clkenop.trc ./mem_files/clkenr.trc > totaldata.trc"
make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
(output, err) = make_process.communicate()

cmdtorun = "cat ./mem_files/addr_ins.trc ./mem_files/addr_data.trc ./mem_files/addr_local.trc ./mem_files/addr_trig.trc ./mem_files/addr_clken.trc> totaladdr.trc"
make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
(output, err) = make_process.communicate()

######################################
#loop start and loop end
######################################
cmdtorun = "grep " + "\"LOOP_START_END\"" + " " + ins_inp+ " > ./mem_files/loopstart_data.trc\nsed -i \'s/LOOP_START_END=//\' ./mem_files/loopstart_data.trc"

make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
(output, err) = make_process.communicate()
loopstart_file = open("./mem_files/loopstart_data.trc", "r")
data = loopstart_file.read()
loopstart_file.close()
totaldata = open("totaldata.trc","a+")
totaladdr = open("totaladdr.trc","a+")

totaldata.write(data)

totaladdr.write("1100000000000000000\n")
totaladdr.close()
totaldata.close()

######################################
#iter counter binary
######################################
cmdtorun = "grep " + "\"ITER\"" + " " + ins_inp+ " > ./mem_files/iter_data.trc\nsed -i \'s/ITER=//\' ./mem_files/iter_data.trc"

make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
(output, err) = make_process.communicate()
iter_file = open("./mem_files/iter_data.trc", "r")
data = iter_file.read()
iter_file.close()
totaldata = open("totaldata.trc","a+")
totaladdr = open("totaladdr.trc","a+")

totaldata.write(data[-17:]+"00000000000000"+data[:-17]+"\n")

totaladdr.write("1111000000000000000\n1111000000000000001\n")

######################################
#clken binary
######################################
cmdtorun = "grep " + "\"TILEEN\"" + " " + ins_inp+ " > ./mem_files/clken_data.trc\nsed -i \'s/TILEEN=//\' ./mem_files/clken_data.trc"

make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
(output, err) = make_process.communicate()
clken_data_file = open("./mem_files/clken_data.trc", "r")
data = clken_data_file.read()
#print(len(data));
clken_data_file.close()
totaldata = open("totaldata.trc","a+")
totaladdr = open("totaladdr.trc","a+")
totaldata.write(data)

totaladdr.write("1101000000000000000\n")
######################################
#clken_op binary
######################################
cmdtorun = "grep " + "\"CMEMOPEN\"" + " " + ins_inp+ " > ./mem_files/clken_op_data.trc\nsed -i \'s/CMEMOPEN=//\' ./mem_files/clken_op_data.trc"

make_process = subprocess.Popen(cmdtorun,shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
(output, err) = make_process.communicate()
clken_data_file = open("./mem_files/clken_op_data.trc", "r")
data = clken_data_file.read()
#print(len(data));
clken_data_file.close()
totaldata = open("totaldata.trc","a+")
totaladdr = open("totaladdr.trc","a+")
totaldata.write(data)

totaladdr.write("0111000000000000000\n")


######################################
#Adding the vec_size
######################################
totaldata = open("totaldata.trc","a+")
totaladdr = open("totaladdr.trc","a+")
totaldata.write(vec_size)
totaldata.write("\n")
totaladdr.write("0110000000000000000\n")


totaladdr.close()
totaldata.close()

