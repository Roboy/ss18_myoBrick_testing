# Testbench Control
This project contains the necessary code to control the testbench. 

It is mostly based on preexisting modules of roboy, whose setup can be found in the respective readme. 

To setup control of the test bench control, first clone this repo. 
Then enter the folder "src/" and type the commands 
```
git submodule init
git submodule update
```
Afterwards, copy the files 
```
./main.cp
```
into
the folder 
```
./src/roboy_plexus/src/
```
and 
```
./testbench_sensor_status/ 
```
into 
```
./src/roboy_rqt_plugins/
```
Now run the command "catkin_make" in the root directory of the repository.
From here on out one can put the compiled file onto the FPGA (following the setup here: https://devanthro.atlassian.net/wiki/spaces/SS18/pages/267747332/How+to+connect+to+Motor+FPGA) 
or run rqt plugin. 


# PS: 

the current version of roboy_plexus, being loaded onto the FPGA only enables reading out sensor data. Motor control has been disabled, mostly due to limited working pins on the available FPGA. If wanted, the control parts can be commented in (or rather copied from the latest version of roboy_plexus in the respective submodule). 