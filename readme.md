# Image-Based GPS Interface for Rover Navigation
In autonomous navigation, precise input of GPS coordinates is critical for the successful movement and control of robotic rovers. Traditionally, GPS coordinates are manually entered, which can be time-consuming and prone to user errors. To streamline this process, this repository introduces an image-based GPS input interface designed to simplify the navigation setup for rovers. By enabling users to select GPS points directly from an interactive map, the repository offers an intuitive and efficient way to provide navigation inputs.

This repository integrates image-based interaction with GPS data processing and is built as a [ROS (Robot Operating System)](http://wiki.ros.org/noetic/Installation/Ubuntu) package to ensure seamless integration with robotic platforms. Users can click on a map image, which is an [OpenStreet Map tile](https://www.openstreetmap.org/), the selected pixels are converted into real-time GPS data and fed to the rover through the ROS communication framework. This enables smooth interaction between the user and the rover’s navigation system. The interface is particularly beneficial for field applications where quick route adjustments and intuitive inputs are essential.



	
	
## Environment setting
* Create the [Ros Workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and inside workspcace, create an empty directory src.
* Inside `src` folder, download the repository and build the package.
* The package has one c++ client `get_gps_from_map_client.cpp` in the folder `src`, and has corresponding server is defined in the script `get_gps_from_map_server.py` located in `script` folder of this package.



## Workflow of the package
* First server need to be run by running the following commands
```
rosrun get_gps_from_map get_gps_from_map_server.py
```
A blank window will appear with title `tile_map`.

* In second terminal, client need to be run with followinhg commands
```
rosrun get_gps_from_map get_gps_from_map_client
```

* In third terminal, Run rviz and load the rviz config file `rviz_config.rviz`

		
* Once you up the client, you will see some messages in the client terminal. Client will ask you to either update the Reference GPS location or get the transformed GPS or exit the loop. Once you update the reference GPS location, you will see the satellite map in image as well in RVIZ. Now on image, you can select the pixels using left click and once you are done, right click on the image. The slected pixels are shown in the image as well as tranformed points are shown in RVIZ as well. One of the example is shown below:
 
 
 
Window `tile_map` for getting user Input
![Alt text](.tile_map.png?raw=true "User Interface")




Tranformed GPS signals visualized in RVIZ
![Alt text](.Rviz.png?raw=true "RVIZ")



## Issues
* In rviz, sometimes AerialMap display is not visible, this is because the correponding RVIZ plugins are not installed. Hence a temporary workaround is build the package [`rviz_satellite`](https://github.com/nobleo/rviz_satellite/tree/ros1) in your workspace and lunch the rviz with following command
```
roslaunch rviz_satellite demo.launch
``` 
