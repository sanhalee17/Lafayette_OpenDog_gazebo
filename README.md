# Lafayette_OpenDog_gazebo
-This repository stores Lafayette_openDog yaml, urdf and launch files for Gazebo simulation. 
-Package name is 'openDog_description'

# Directory / File / Command Explanation
1. __Config__ stores controller gains. Adjust those if necessary
2. **Launch** stores openDog Gazebo/gait pattern launch files.

	-To spawn the openDog in Gazebo world, type in 
		```roslaunch openDog_description opendog_gazebo_control.launch``` for 2D model or 
		```openDog_description opendog_gazebo_control_3D.launch``` for 3D model

	-To launch gait patterns, use gazebo_node.launch (2D) or gazebo_node_3D.launch (3D)
		-The syntax will be 
			```roslaunch openDog_description gazebo_node.launch```
			or ```roslaunch openDog_description gazebo_node_3D.launch```

	-To place foot in the default, steady position, use gazebo_home.launch. ```roslaunch openDog_description gazebo_home.launch```

	-To change the environment, modify opendog.world file.

	-Change parameters if necessary.

3. **node** stores necessary nodes for running the gait pattern.
	-new_foot_path.py generates perodic foot path wave
	
	-2D_inverse_kinematics.py calculates angle
	
	-gazebo_joint.py sends the angle to the Gazebo
	
	-For 3D gait, 3D tags are added in file names.
	
	-marker.py is for debugging footpath. visualize this in rviz. ```rviz```

4. **urdf** stores geometry files of openDog. d
	-```openDog_full.urdf``` is 2D model
	
	-```openDog_full_whip.urdf``` is 3D model
	
	-visualize those by using launch file ```urdf_visualize.launch```
	
	-change corresponding file name within the launch file. 