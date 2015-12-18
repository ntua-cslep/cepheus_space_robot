# DESCREPTION
A ROS package that include all the source code that runs on Space robot's PC.

#REQUIRMENTS
- RTD DM95820 card for acquisition of encoder and PWM's 
- 2 or 3 mouse's for localization

# Instructions
## Load DM95820 Drivers
- Source your workspace to find the package

```source devel/setup.bash```

- Copy kernel module to the drivers directory.

```sudo cp $(rospack find cepheus_robot)/DM7820_Linux_*/driver/rtd-dm7820.ko /lib/modules/$(uname -r)/kernel/drivers/```

- Add the name of themodule to the file /etc/modules. You can edit the file or just append to it as shown here.

```echo 'rtd_dm7820.ko' | sudo tee -a /etc/modules```

- Update the list of module dependencies.

```sudo depmod```

- Reboot the computer and voila, it worked.

##Mouse Instalation 
- In the launch file of the package modify the parameters of the mouse_odom node to correspond to the correct mouses like that:

_/dev/input/by-path/"correct mouse name"_


## In every boot run the following commands before launch:     
- ```cd ~/catkin_ws```
- ```su```
- password root
- ```soure devel/setup.bash```
- ```export ROS_MASTER_URI=http://cepheus.local:11311```
- ```export ROS_IP=192.168.1.165```