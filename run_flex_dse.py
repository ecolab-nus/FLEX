#!/usr/bin/env python
import sys
import os
import os.path
import shutil
import re

import scripts.bin_to_trace as BT
FLEX_HOME = os. getcwd() 
MAPPER_HOME = FLEX_HOME + '/Blockwise_Morpher_Mapper'
MICROARCHITECTURE_HOME = FLEX_HOME + '/Microarchitecture'
KERNEL_HOME = FLEX_HOME + '/Kernels'

def run_dse_mapper(dfg_name,v,initII,maxII):
    os.chdir(KERNEL_HOME+'/'+dfg_name)
    print('Mapping for vector size='+str(v))
    os.system(MAPPER_HOME+'/build/src/cgra_xml_mapper -d *.xml -j flex_4x4.json -i '+str(initII)+' -v '+str(v)+' --max_II '+str(maxII)+' >/dev/null')
    currII=maxII+v
    for file in os.listdir("./"):
        if file.endswith(".bin"):
            currII=int(file.split('II=')[-1].split(' ')[0].split('_V')[0].split(' ')[0])
    os.system('cp *.bin binary/')
    os.system('cp *.bin '+MICROARCHITECTURE_HOME+'/verif/')
    os.system('rm *.txt *.csv *.log *.info *.bin')
    os.chdir(MICROARCHITECTURE_HOME+'/verif')
    os.system('rm -rf '+dfg_name+'_'+str(v))
    os.system('mkdir '+dfg_name+'_'+str(v))
    os.system('mv *.bin binary.bin')
    print('Mapped with II='+str(currII))
    return currII


def run_dse_power(dfg_name,v,currII):
    os.chdir(MICROARCHITECTURE_HOME+'/verif/'+dfg_name+'_'+str(v))
    BT.generate_mem(1024)
    BT.dump_trace_full("binary.bin","data_dm.txt",currII)
    os.system('cp ../tb.sv '+dfg_name+'_'+str(v)+".sv")

    if v==1:
        BT.dump_include(dfg_name,"totaladdr.trc",currII,8,2,256,4,16)
    else:
        BT.dump_include_with_trigger(v,dfg_name,"binary.bin","totaladdr.trc",currII,8,2,256,4,16)

    os.chdir(MICROARCHITECTURE_HOME)
    os.system('make sim UNIT=flex TBTOP='+dfg_name+'_'+str(v))
    os.system('make power UNIT=flex TBTOP='+dfg_name+'_'+str(v))

        
def main():
    print(r"""
    ###------  FLEX DESIGN SPACE EXPLORATION ------###
    """)    
    dfg_name = str(sys.argv[1])
    vc = int(sys.argv[2])
    vh = int(sys.argv[3])

    performance = {}

    if vc < vh:
        vmax = vc
    else:
        vmax = vh

    os.chdir(KERNEL_HOME+'/'+dfg_name)
    os.system('rm -rf binary')
    os.system('mkdir binary')

    for v in range(vmax):
        if v == 1:
            stII = run_dse_mapper(dfg_name,v,1,8)
            performance[v] = stII
        elif v > 1:
            stvII = run_dse_mapper(dfg_name,v,stII,(stII+4)*v) #maxII is the performance margin set
            performance[v] = stvII



if __name__ == '__main__':
  main()
