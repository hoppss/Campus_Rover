source ~/campusrover2019_ws/devel/setup.bash
export ROS_IP=192.168.3.21
export ROS_MASTER_URI=http://192.168.3.10:11311/
alias ssh_nano="ssh nvidia-nano@192.168.3.10"
alias cr_demo="roslaunch campusrover_demo demo.launch"
alias cr_sensor="roslaunch campusrover_sensors start_sensor.launch"
# alias cr_move_eeoffice="rostopic pub /cr_trip campusrover_msgs/LocationRoom 'building': 'itc', floor: '2', room: 'r11'}'"