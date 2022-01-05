# UAV augumented reality GUI 

Scripts for implementing graphical user interface as part of HMI (human machine interface) for UAV operator and UAV. 

# Required dependencies (drawGUI):
* numpy = 1.13.3
* pillow = 5.3.0 

Could be newer probably 

# Required dependencies (simulation): 
* rotors_gazebo = `2c990b01ec653983531047534bfc923f2cf8974d` (larics)  
* gazebo = 9.19


## To run the "draw GUI" node:
1. In one terminal run simulation with following command: `roslaunch rotors_gazebo mav_hovering_example_with_vi_sensor.launch`
2. In other terminal change the current working directory to: .../probniPack/launch and run the command: `roslaunch launchodm.launch`
3. In third terminal run the command: `rosrun rviz rviz` in order to see drone GUI

## ROS network setup 

Current idea is to use this package to draw GUI on smart glasses
of an operator. 

To be able to do so, I'm using 2 PCs. One PC is server PC 
and it runs simulation + HPE and roscore. 

That PC has following configuration: 
```
ROS_MASTER_URI=http://192.168.0.20:11311
ROS_IP=192.168.0.20
```

While raspberryPi which runs GUI draw has: 
```
ROS_MASTER_URI=http://192.168.0.177:11311
ROS_IP=192.168.0.20
```

Both of those are added to corresponding `~/.bashrc`


## TODO: 

- [x] Change package name
- [x] test on raspberryPi
- [x] use compressed_image
- [x] minor refactoring
- [ ] Test with python3 
- [ ] Test with official larics simulation stack 
- [ ] Add sweep plotter on UI 
- [ ] Add visualization of human pose estimation for HMI control 
- [ ] Update documentation instructions (a) sim (b) real robot
- [ ] clean catkin text
