# Autolabor hardware configuration files
* [autolabor-devices.rules](autolabor-devices.rules) - Define USB setting
* [setRule.sh](setRule.sh) - A script files to put "autolabor-devies.rules" and ".conf" to system

## Find USB device serial devpath
```
udevadm info -a -n /dev/ttyUSB1 | grep '{devpath}'
udevadm info -a -n /dev/ttyUSB1 | grep '{serial}'
```

## Udev Setting    
Setting the device rule. 
```
cd ~/autolabor_ws/src/autolabor/autolabor_hardware_rule/
sh setRule.sh 
```
