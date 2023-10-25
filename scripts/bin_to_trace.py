import re
import collections
import subprocess
import sys
import argparse
import os
import os.path

## PYTHON FILE with constants
import scripts.flex_config as FLEX

def generate_cm_data_addr(ins_inp, TIMEEXEC):
    ###### step 1: split CM instructions by tiles
    num_inst_per_tile = 0
    for i in range(FLEX.TILES_NUM_ROWS):
        for j in range(FLEX.TILES_NUM_COLS):

            tile_num = (i*FLEX.TILES_NUM_ROWS) + j
            os.system("grep -i -A "+ str(TIMEEXEC) + " \"\[CONFIGURATION\] Y=" + str(i) + " " + "X=" + str(j) + "\" " + ins_inp + " | tail -n +3" + " >" + " ./mem_files/tile" + str(tile_num) + "_inst.trc")

    ##### step 2: prune instructions, convert 64b to 16b
    for i in range(FLEX.TILES_NUM_ROWS):
        for j in range(FLEX.TILES_NUM_COLS):
            tile_num = (i*FLEX.TILES_NUM_ROWS) + j  

            os.system("sed -i " + "\'s/Y=" + str(i) + " " + "X=" + str(j) + ",//\'" + " ./mem_files/tile" + str(tile_num) + "_inst.trc")

            split_iterations = int(FLEX.CM_WIDTH/FLEX.DATA_WIDTH)
            cmdtorun = "sed -i \'"
            for x in range(1, split_iterations):
                if(x!=1):
                    cmdtorun = cmdtorun + ";"
                cmdtorun = cmdtorun + "s/./&\\n/" + str(x*FLEX.DATA_WIDTH + (x-1))
            cmdtorun = cmdtorun + "\' ./mem_files/tile" + str(tile_num) + "_inst.trc" 
            os.system(cmdtorun)

    ####### step 3: GENERATE CM binaries
    cmdtorun = "cat "
    for i in range(FLEX.TILES_NUM_ROWS):
        for j in range(FLEX.TILES_NUM_COLS):
            tile_num = (i*FLEX.TILES_NUM_ROWS) + j
            cmdtorun = cmdtorun + "./mem_files/tile" + str(tile_num) + "_inst.trc "
    cmdtorun = cmdtorun + "> ./mem_files/ins.trc"
    os.system(cmdtorun)
    cm_addr_file = open("./mem_files/addr_ins.trc","w+")

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
    
def generate_local_data_addr(local_inp, TIMEEXEC):
    num_inst_per_tile = 0
    for i in range(FLEX.TILES_NUM_ROWS):
        for j in range(FLEX.TILES_NUM_COLS):
            tile_num = (i*FLEX.TILES_NUM_ROWS) + j

            os.system("grep -i -A "+ str(FLEX.LOCAL_REG_DEPTH) + " \"\[LOCAL_REG\] Y=" + str(i) + " " + "X=" + str(j) + "\" " + local_inp + " | tail -n +2" + " >" + " ./mem_files/tile" + str(tile_num) + "_local_inst.trc")

    cmdtorun = "cat "
    for i in range(FLEX.TILES_NUM_ROWS):
        for j in range(FLEX.TILES_NUM_COLS):
            tile_num = (i*FLEX.TILES_NUM_ROWS) + j
            cmdtorun = cmdtorun + "./mem_files/tile" + str(tile_num) + "_local_inst.trc "
    cmdtorun = cmdtorun + "> ./mem_files/local.trc"
    os.system(cmdtorun)

    local_addr_file = open("./mem_files/addr_local.trc","w+")

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

def generate_dm_data_addr(data_inp, TIMEEXEC):
    dm_addr_file = open("./mem_files/addr_data.trc","w+")
    dm_data_file = open("./mem_files/data.trc","w+")
    results_expected_file = open("results_expected.trc", "w+")
    dm_src_read_file = open(data_inp,"r")

    if(FLEX.WRITE_DM):
        addr_mem_type = FLEX.ADDR_MEM_TYPE_ENCODING["DM"]

        dataToWrite = {}
        dataVal = {}
        dataVal_Ordered = {}
        dataToWrite_backup = {}
        lines_n = []
        lines = dm_src_read_file.readlines()

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

def generate_trigger_data_addr(ins_inp, TIMEEXEC):
    cm_addr_trig_file = open("./mem_files/addr_trig.trc","w+")
    ###### step 1: split CM instructions by tiles
    num_inst_per_tile = 0
    for i in range(FLEX.TILES_NUM_ROWS):
        for j in range(FLEX.TILES_NUM_COLS):
            tile_num = (i*FLEX.TILES_NUM_ROWS) + j    
            os.system("grep -i \"\[TRIGGER\] Y=" + str(i) + " " + "X=" + str(j) + "\" " + ins_inp + " | grep -o \'[^:]\+$\'" + " >" + " ./mem_files/tile" + str(tile_num) + "_trigger.trc")

    ####### step 3: GENERATE CM binaries
    cmdtorun = "cat "
    for i in  range(FLEX.TILES_NUM_ROWS):
        for j in range(FLEX.TILES_NUM_COLS):
            tile_num = (i*FLEX.TILES_NUM_ROWS) + j
            cmdtorun = cmdtorun + "./mem_files/tile" + str(tile_num) + "_trigger.trc "
    cmdtorun = cmdtorun + "> ./mem_files/trigger.trc"
    os.system(cmdtorun)

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

def generate_clken_data_addr(ins_inp, TIMEEXEC):
    ###### step 1: split CM instructions by tiles
    num_inst_per_tile = 0
    for i in range (TIMEEXEC):
        # write CM files   
        os.system("grep -i \"\[CLKENCMEM\] T=" + str(i) + "\" " + ins_inp + " | grep -o \'[^:]\+$\'" + " >" + " ./mem_files/clkenop_" + str(i) + ".trc")
        os.system("grep -i \"\[CMEMROUTEEN\] T=" + str(i) + "\" " + ins_inp + " | grep -o \'[^:]\+$\'" + " >" + " ./mem_files/clkenr_" + str(i) + ".trc")

    ####### step 3: GENERATE CM binaries
    cmdtorun = "cat "
    cmdtorun1 = "cat "
    cm_addr_clken_file = open("./mem_files/addr_clken.trc","w+")

    for i in range (TIMEEXEC-1):
        cmdtorun = cmdtorun + " ./mem_files/clkenop_" + str(i+1) + ".trc"
        cmdtorun1 = cmdtorun1 + " ./mem_files/clkenr_" + str(i+1) + ".trc"
    cmdtorun = cmdtorun + "> ./mem_files/clkenop.trc"
    cmdtorun1 = cmdtorun1 + "> ./mem_files/clkenr.trc"

    os.system(cmdtorun)
    os.system(cmdtorun1)

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

def generate__data_addr(vec_size, ins_inp, TIMEEXEC):
    os.system("grep " + "\"LOOP_START_END\"" + " " + ins_inp+ " > ./mem_files/loopstart_data.trc\nsed -i \'s/LOOP_START_END=//\' ./mem_files/loopstart_data.trc")
    os.system("grep " + "\"ITER\"" + " " + ins_inp+ " > ./mem_files/iter_data.trc\nsed -i \'s/ITER=//\' ./mem_files/iter_data.trc")
    os.system("grep " + "\"TILEEN\"" + " " + ins_inp+ " > ./mem_files/clken_data.trc\nsed -i \'s/TILEEN=//\' ./mem_files/clken_data.trc")
    os.system("grep " + "\"CMEMOPEN\"" + " " + ins_inp+ " > ./mem_files/clken_op_data.trc\nsed -i \'s/CMEMOPEN=//\' ./mem_files/clken_op_data.trc")

    loopstart_file = open("./mem_files/loopstart_data.trc", "r")
    loopstart_data = loopstart_file.read()
    loopstart_file.close()

    iter_file = open("./mem_files/iter_data.trc", "r")
    iter_data = iter_file.read()
    iter_file.close()

    clken_data_file = open("./mem_files/clken_data.trc", "r")
    clken_data = clken_data_file.read()
    clken_data_file.close()

    clken_op_data_file = open("./mem_files/clken_op_data.trc", "r")
    clken_op_data = clken_op_data_file.read()
    clken_op_data_file.close()

    totaldata = open("totaldata.trc","a+")
    totaladdr = open("totaladdr.trc","a+")

    totaldata.write(loopstart_data)
    totaldata.write(iter_data[-17:])
    totaldata.write("00000000000000"+iter_data[:-17]+'\n')
    totaldata.write(clken_data)
    totaldata.write(clken_op_data)
    totaldata.write('{0:016b}'.format(vec_size))
    totaldata.write("\n")

    totaladdr.write("1100000000000000000\n")
    totaladdr.write("1111000000000000000\n1111000000000000001\n")    
    totaladdr.write("1101000000000000000\n")
    totaladdr.write("0111000000000000000\n")
    totaladdr.write("0110000000000000000\n")

    totaladdr.close()
    totaldata.close()


def dump_trace_full(vec_size, ins_inp,data_inp,local_inp,TIMEEXEC):
    generate_cm_data_addr(ins_inp, TIMEEXEC)
    generate_dm_data_addr(data_inp, TIMEEXEC)
    generate_local_data_addr(local_inp, TIMEEXEC)
    generate_clken_data_addr(ins_inp, TIMEEXEC)

    if(vec_size > 1):
        generate_trigger_data_addr(ins_inp, TIMEEXEC)

        os.system("cat ./mem_files/ins.trc ./mem_files/data.trc ./mem_files/local.trc ./mem_files/trigger.trc ./mem_files/clkenop.trc ./mem_files/clkenr.trc > totaldata.trc")
        os.system("cat ./mem_files/addr_ins.trc ./mem_files/addr_data.trc ./mem_files/addr_local.trc ./mem_files/addr_trig.trc ./mem_files/addr_clken.trc> totaladdr.trc")
    else:
        os.system("cat ./mem_files/ins.trc ./mem_files/data.trc ./mem_files/local.trc ./mem_files/clkenop.trc ./mem_files/clkenr.trc > totaldata.trc")
        os.system("cat ./mem_files/addr_ins.trc ./mem_files/addr_data.trc ./mem_files/addr_local.trc ./mem_files/addr_clken.trc> totaladdr.trc")


    generate__data_addr(vec_size, ins_inp, TIMEEXEC)

def dump_include_with_trigger(vec_size, dfgname,ins_inp,totaladdr,mappedII, CM_WIDTH_BYTES, DM_WIDTH_BYTES, DM_DEPTH, NUM_DM, PE_SIZE):
    with open(totaladdr, 'r') as fp:
        lines = len(fp.readlines())

    cm_inst = CM_WIDTH_BYTES*mappedII*PE_SIZE
    dm_inst = DM_WIDTH_BYTES*DM_DEPTH*NUM_DM

    trig_arr=[]
    for i in range(FLEX.TILES_NUM_ROWS):
        trig_arr_row=[]
        for j in range(FLEX.TILES_NUM_COLS):
            tile_num = (i*FLEX.TILES_NUM_ROWS) + j
            os.system("grep -i \"\[OPERATION TRIGGER\] Y=" + str(i) + " " + "X=" + str(j) + "\" " + ins_inp + " | cut -d ':' -f 2" + " >" + " ./mem_files/trig_" + str(tile_num) + ".trc")

    trig_arr = []
    for v in range(1,vec_size+1):
        trig_row = []
        for i in range(FLEX.TILES_NUM_ROWS):
            trig_col=""
            for j in range(FLEX.TILES_NUM_COLS):
                tile_num = (i*FLEX.TILES_NUM_ROWS) + j

                with open("./mem_files/trig_" + str(tile_num) + ".trc", 'r') as f1:
                    trig = f1.readlines()

                if int(trig[0], 2) <= v-1:
                    trig_col= '1'+trig_col
                else:
                    trig_col= '0'+trig_col
            trig_row.append(trig_col)
        trig_arr.append(trig_row)

    filename = dfgname+'_'+str(vec_size)+"_def.svh"

    f =open(filename, "a")
    f.write("`define TB_NUM_INST "+str(lines)+'\n')
    f.write("`define TB_NUM_DMEM_INST "+str(dm_inst)+'\n')
    f.write("`define TB_NUM_CMEM_INST "+str(cm_inst)+'\n')
    
    f.write("\n")
    f.close()

    filename = dfgname+'_'+str(vec_size)+"_trig.svh"

    f =open(filename, "a")

    f.write("task enable_trig;\n")

    for v in range(1,vec_size+1):
        f.write("@(posedge `HYC_TOP.clk);\n")        
        for i in range(FLEX.TILES_NUM_ROWS):
            f.write("trigger["+str(i)+"]="+str(FLEX.TILES_NUM_ROWS)+"'b"+trig_arr[v-1][i]+";\n") 
   
    f.write("\nendtask\n")
            
    f.close()

def dump_include(dfgname,totaladdr,mappedII, CM_WIDTH_BYTES, DM_WIDTH_BYTES, DM_DEPTH, NUM_DM, PE_SIZE):
    with open(totaladdr, 'r') as fp:
        lines = len(fp.readlines())

    cm_inst = CM_WIDTH_BYTES*mappedII*PE_SIZE
    dm_inst = DM_WIDTH_BYTES*DM_DEPTH*NUM_DM

    filename = dfgname+'_1_def.svh'

    f =open(filename, "a")
    f.write("`define TB_NUM_INST "+str(lines)+'\n')
    f.write("`define TB_NUM_DMEM_INST "+str(dm_inst)+'\n')
    f.write("`define TB_NUM_CMEM_INST "+str(cm_inst)+'\n')

    f.write("\n")
    f.close()

    filename = dfgname+'_1_trig.svh'

    f =open(filename, "a")
    
    f.write("task enable_trig;\n")    
    f.write("@(posedge `HYC_TOP.clk);")
    f.write("\nendtask\n")
    
    f.close()

def generate_mem(size):
    data_dm_file = open("data_dm.txt","w+")

    for i in range(size):
        data_val = str(i)+","+str(i%10)+","+str(i%10)+"\n"
        data_dm_file.write(data_val)

    data_dm_file.close()
    
