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

## Current bandwith status

```
subscribed to [/stickman_compressed_ctl_area]
average: 763.17KB/s
	mean: 97.05KB min: 92.18KB max: 104.10KB window: 7
average: 628.11KB/s
	mean: 99.00KB min: 92.18KB max: 105.35KB window: 12
average: 648.76KB/s
	mean: 98.77KB min: 92.18KB max: 105.91KB window: 19
```

```
rostopic bw /uav/rod_camera/image_raw/compressed
subscribed to [/uav/rod_camera/image_raw/compressed]
average: 234.52KB/s
	mean: 13.04KB min: 12.85KB max: 13.58KB window: 11
average: 217.45KB/s
	mean: 12.99KB min: 12.81KB max: 13.58KB window: 27
average: 223.44KB/s
	mean: 12.98KB min: 12.81KB max: 13.58KB window: 45
average: 218.94KB/s
```

Client PC: 
```
average rate: 6.384
	min: 0.067s max: 0.496s std dev: 0.05048s window: 446
average rate: 6.377
	min: 0.067s max: 0.496s std dev: 0.05036s window: 451
average rate: 6.370
	min: 0.067s max: 0.496s std dev: 0.05022s window: 458
average rate: 6.347
	min: 0.067s max: 0.496s std dev: 0.05029s window: 463
``` 

Server PC has about 15 Hz. 


## rPi 

Credentials are: 
```
username: rPi 
pass: raspberry
```

## How to show it? 

For showing AR GUI currently i plan to use [rosboard](https://github.com/dheera/rosboard). 
Reasons are following: 
 * Easily configurable for image topics 
 * web-server which is accessible from any browser on local host 
 * better than rviz for current use-case 

## How to install grpcio tools. 

Follow this instructions to install grpcio-tools: 

```
sudo apt-get install python3-pip 
```

After that installation of `grpcio-tools` you can follow this [link](https://stackoverflow.com/questions/56357794/unable-to-install-grpcio-using-pip-install-grpcio)

## Pillow installation procedure

Started using [Pillow-SIMD](https://github.com/uploadcare/pillow-simd) which is few times faster than classical Pillow, 

You can install it as follows: 

```
sudo apt-get install libfreetype6-dev
pip install --upgrade pip 
pip install pillow-simd==5.3.0.post0
```



## TODO: 

- [x] Change package name
- [x] test on raspberryPi
- [x] use compressed_image
- [x] minor refactoring
- [x] Test with python3 
- [ ] Test with official larics simulation stack --> POSTPONED  
- [ ] Add sweep plotter on UI --> Cool stuff but I need laser for that (TBD) 
- [x] Add visualization of human pose estimation for HMI control --> Added it to rosboard/quite easy 
- [x] Use only compressed image 
- [x] Add chrony sinchronization 
- [ ] Add image health (check timestamp vs current PC time) 
- [ ] Update documentation instructions (a) sim (b) real robot 
- [ ] clean catkin text
