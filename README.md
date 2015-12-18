# cepheus_space_robot
A ROS package that include all the source code that runs on Space robot's PC.

#REQUIRMENTS
- RTD DM95820 card for acquisition of encoder and PWM's 
- 2 or 3 mouse's for localization

# Instructions
## Load DM95820 Drivers

- '''source devel/setup.bash'''

- Copy kernel module to the drivers directory.
'''sudo cp $(rospack find cepheus_robot)/DM7820_Linux_*/driver/rtd-dm7820.ko /lib/modules/$(uname -r)/kernel/drivers/'''

- Add the name of themodule to the file /etc/modules. You can edit the file or just append to it as shown here.
'''echo 'rtd_dm7820.ko' | sudo tee -a /etc/modules'''

- Update the list of module dependencies.
'''sudo depmod'''

- Reboot the computer and voila, it worked.

## In every boot run the following commands: 
- sudo chmod a+rw /dev/input/by-path/"your mouse path"         
tip:run this for all mouses
- cd ~/catkin_ws
- soure devel/setup.bash
- export ROS_MASTER_URI=http://192.168.1.231:11311
- export ROS_HOSTNAME=192.168.1.231
- export ROS_IP=192.168.1.231