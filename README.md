# Lafayette_OpenDog_gazebo
-This repository stores Lafayette_openDog yaml, urdf and launch files for Gazebo simulation. 
-Package name is 'openDog_description'

# Directory / File Explanation
1. __Config__ stores controller gains. Adjust those if necessary
2. **Launch** stores openDog Gazebo/gait pattern launch files.

	-To spawn the openDog in Gazebo world, type in 
		```roslaunch openDog_description opendog_gazebo_control.launch``` for 2D model or 
		```openDog_description opendog_gazebo_control_3D.launch``` for 3D model

	-To launch gait patterns, use gazebo_node.launch (2D) or gazebo_node_3D.launch (3D)
		-The syntax will be 
			```roslaunch openDog_description gazebo_node.launch```
			or ```roslaunch openDog_description gazebo_node_3D.launch```

3. 
