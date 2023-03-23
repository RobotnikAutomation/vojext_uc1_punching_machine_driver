# punching_driver

ROS driver for UC1 punching PEMU machine 

It is needed to install the pyModbusTCP library as follows (this version is required):

sudo pip install pyModbusTCP==v0.1.10

## MODBUS-TCP address

test punching machine
PLC1      IP addrress is: 192.168.17.195 (testing)
PLC2      IP addrress is: 192.168.17.196

### Only Read address

        punched_pillow = 16384


### Write address

        pillow_ready = 16385
