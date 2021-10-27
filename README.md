# PX4-Controllers

## Description
This repos contains external PX4 modules designed to allow for easy implementation of custom controllers. The `mc_pos_control_pid` module is an implementation of the default pid controller.

Currently only position controllers are supported. Future support will be added for attitude controllers in another module.

## Installation

1. Clone this repository to a directory outside of the PX4-Autonomy firmware
```git clone https://github.com/amirza495/px4-controllers.git```

2. Remove the position controller from the MODULES list in `boards/px4/sitl/default.cmake`. The default module creates conflicts with some required libraries for this module.

2. Remove the line `mc_pos_control start` from the end of `ROMFS/px4fmu_common/init.d/rc.mc_apps`. Optionally, the `mc_pos_control` module can be changed to your controller module. Otherwise you will need to start your position control module manually.

2. Build your desired target with the location of this git repo as the EXTERNAL_MODULES_LOCATION variable. For example, this is for creating a jmavsim SITL simulation with this repository installed at `/home/adam`
```make px4_sitl jmavsim EXTERNAL_MODULES_LOCATION=/home/adam/px4-controllers/``

3. In the `pxh>` prompt, enter `mc_pos_control_pid start` or insert the name of your module, to start the position control module.