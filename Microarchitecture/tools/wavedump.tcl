#Generates shm waveforms during simulation
#To generate VCD waveform format, just replace -shm with -vcd and remove -memories
#Specify WAVE=off to disable waveform dump and speed up simulation
if { $::env(WAVE) != "off" } {
    database -open $::env(WAVE) -shm -compress -event -default
    probe -create tb_top -shm -all -dynamic -memories -depth all
}
run
exit

