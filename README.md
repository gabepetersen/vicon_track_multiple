# vicon_track_multiple
Package to track multiple vicon objects and detect if they are colliding through the seperation of axes theorem

------------------------------------------------------------------------
# Implementation 
------------------------------------------------------------------------

### Instructions on how to run:

1. In vicon_track_multiple/launch/vicon_track.launch,
   Specify the amount of duckiebots that will be added to the duckietown by controlling how many versions of nodes of car.cpp there will be. From there, you can specify the parameters of each duckiebot's name, size, and number For example:
   ```
      <launch>
	      <node name="car1" pkg="vicon_track_multiple" type="car" output="screen">
		      <!-- tells the node what car it is based on int -->
		      <param name="number" type="int" value="1"/>

		      <!-- tells the node what the corresponding vicon object's name is -->
		      <param name="obj_name" type="string" value="car1"/>

		      <!-- Threshold for collision detection -->
		      <!-- i.e. What distance the collision box edge should be from the vicon obj origin -->
          <!--- x_thresh is the width and y_thresh is the width of the duckiebot's collision boundary box -->
		      <param name="x_thresh" type="double" value="0.18"/>
		      <param name="y_thresh" type="double" value="0.13" />
	      </node>
	
	      <node name="car2" pkg="vicon_track_multiple" type="car" output="screen">
		      <param name="number" type="int" value="2"/> 
		      <param name="obj_name" type="string" value="car2"/> 
		      <param name="x_thresh" type="double" value="0.18"/> 
		      <param name="y_thresh" type="double" value="0.13" /> 
	      </node>
      </launch>
   ```
   This creates a system with two duckiebots: one with int value 1 and another with 2. Their names are car1 and car2 respectively, and it is important to name them car followed by the number on vicon for the nodes to work. In this case, both cars have a boundary box with width of 0.18 and and height of 0.13. 

2. ```catkin build```

3. ```roslaunch vicon_track_multiple qtalker```

4. Move the duckiebots around and wait for collisions!


------------------------------------------------------------------------
# Things that most likely need changing
------------------------------------------------------------------------
1. For now each car does collision detection on its own, examining other cars in the surrounding vicon area. This should probably be centralized so that only one computer is calculating this. In this case, only one instance of car.cpp is ran, so the code would have to be altered a bit
2. With the current code, it can only handle three cars. If in need of more, edit the default constructor: ViconTrack() in car.cpp as well as add more subscriber callbacks to handle more vicon objects.
3. When collisions happen, the only thing that happens is a message is printed out. The code should be altered to advertise messages to a topic when there is a collision between duckiebots.


------------------------------------------------------------------------
