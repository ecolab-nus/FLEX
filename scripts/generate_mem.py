# python3 automate.py --cubeins {file_from_mapper}.bin --cubedata {data_file}.txt --cubetime {II+1} --cubeclus {no_clusters}
import sys
MEM_SIZE = int(sys.argv[1])

data_dm_file = open("data_dm_zero.txt","w+")

for i in range(MEM_SIZE):
    #data_val = str(i)+","+str(i%10)+","+str(i%10)+"\n"
    data_val = str(i)+","+str(0)+","+str(0)+"\n"
    data_dm_file.write(data_val)

data_dm_file.close()

