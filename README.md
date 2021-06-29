# probniPack

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

- [ ] Change package name 
- [x] [PEP8 Code Styling](https://www.python.org/dev/peps/pep-0008/)   
- [ ] Check different layouts / inspect different python plotting libraries (Compass, different artifical horizon positioning, decoupled roll, pitch)
- [ ] [Explore UX/UI design for FPV](https://www.rightware.com/blog/the-future-of-automotive-ux-from-the-designers-perspective)     	
