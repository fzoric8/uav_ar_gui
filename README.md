# UAV augumented reality GUI 

Scripts for implementing graphical user interface as part of HMI (human machine interface) for UAV operator and UAV. 

# Required dependencies (drawGUI):
* numpy
* pillow

# Required dependencies (simulation): 
* rotors_gazebo   
* gazebo 


## To run the "draw GUI" node:
1. In one terminal run simulation with following command: `roslaunch rotors_gazebo mav_hovering_example_with_vi_sensor.launch`
2. In other terminal change the current working directory to: .../probniPack/launch and run the command: `roslaunch launchodm.launch`
3. In third terminal run the command: `rosrun rviz rviz` in order to see drone GUI

## TODO: 

- [ ] Test with python3 
- [ ] Test with official larics simulation stack 
- [ ] Add sweep plotter on UI
- [ ] Add visualization of human pose estimation for HMI control 
- [ ] Change package name (still probniPack)
- [ ] Test on Raspberry PI
- [ ] Update documentation instructions (a) sim (b) real robot
- [ ] clean catkin text
