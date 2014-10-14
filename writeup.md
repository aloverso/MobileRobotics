#Robot Marco Polo
Kaitlin Gallagher
Anne LoVerso

##Description:
Make a three-robot modified marco polo game. One robot, Marco, is tasked with chasing other two, the Polos, without receiving its own position information and with only sporadic access to Polos’ information. Polos are restricted to a field and have access to their own location as well as knowing where Marco is. 

##Basic Goals:
Each robot can:
- Publish its location as a tf frame
- Receive location of other players
- Identify when a tag has happened
- Switch roles when a tag occurs
“Marco” chasing robot can:
- Request opponents’ relative locations at random intervals
- Chase closest robot
“Polo” robot can:
- Run away from Marco
- Stay within specified boundary

##Stretch Goals:
- Random bursts of invisibility granted to Polos
- More advanced chasing algorithms
- More advanced running algorithms


##Introduction

In this project, we sought to create a Marco Polo game with three robots, one “Marco” and two “Polos” that played a form of tag with each other. 

We solved this problem primarily through the use of tf transforms. These transforms are coordinate transformations between frames, allowing the robots to identify another robot’s position within a larger world frame. This method relies entirely on dead reckoning between base_link and odom for each robot.

We used tf to broadcast a transform between any given robot and a “world” frame, which was centered at the start of Marco’s odometry. By consistently starting the robots from the same positions relative to Marco, we were able to define a static transformation between their odometry and the world frame.

Figure 1 shows the tf frames view of the coordinate frame structure. Each robot is connected to the overall world frame. Because of this, we can use tf in our code to request transforms between, the base_links of two robots, and tf is able to calculate that for us.

![Figure 1](frames.png)
Fig 1. Tree of tf frames produced by running rosrun tf view_frames.

##Design Decision

We chose to communicate robot position using dead reckoning and and a static transform between robot odometries to connect them to the overall world frame. There are some drawbacks to this approach, but we decided it would be best for this project. Like the static transform between base_link and base_laser_link, this is based on a measured distance between starting points. This means that unless we start our robots in precisely the same locations, the coordinate transforms will be offset from the actual robot positions and that we have to expect some drift over the course of the game.

Although other options, such as SLAM, would have provided more accurate positioning and allowed for more advanced following/chasing techniques, we wanted to focus on game logic and getting the game to work. Broadcasting a static transform and relying on the built-in dead reckoning is a lot less difficult that attempting to incorporate SLAM and mapping into our code, which left us more time to work harder on the logic of Marco and Polo movements. 

##Code Structure

The code is structured in an object-oriented framework, with two classes “Marco” and “Polo” both inheriting from a common “Robot” class. The robot class has all the specifications that both Marcos and Polos need, such as a robotname that identifies the ROS namespace of that robot and a tf listener. They have a common “run” method that broadcasts the static transform to the world frame each timestep and publishes a stamped pose within the world frame that can be used to track the robots in rviz. All robots also have a “get_transform” method that is called often within the code to find the coordinate transform of a “from_frame” within the coordinate system of the “to_frame”. This method is vital to the system, because it allows the robots to find each other and make decisions based off those results.

##Challenges

We faced several challenges along the way on this project.

###Polo movement

We wanted the Polos to be able to run away from Marco, but also stay within a set boundary. The eventual solution was creating two force vectors that repulsed Polo, one for Marco and one for the boundary. The result was that Polo had a tendency to stay towards the origin, as moving towards the boundary increased the repulsive force. But being close to Marco was also a priority, so the closer Polo is to Marco, the more it should focus on running away. In order to prevent the robots from leaving the boundary at all costs, the boundary force at the boundary is about twice as strong as the maximum Marco force, but drops off quickly as a fourth-order polynomial equation. 

###Marco Following

In a real game of Marco Polo, Marco is supposed to call out to the Polos every so often, gaining a single location for a given point in time, and moving towards that point until it calls out again, and updates the goal point for the closest Polo.

We wrote the logic for this behavior, but had difficulty implementing it because of its periodic behavior. We started by having Marco call out every five seconds, and updating a goal point within his odometry to look for. We then had him calculate his own transform between his base_link and odometry each timestep. Because we had a single point as the goal that was only periodically updated, we could not find a function in the tf API that would allow us to convert between a given point in Marco’s odometry to Marco’s base_link. We did these calculations by hand using trigonometry, and implemented them. We tested them by driving Marco around a stationary Polo, and watching the desired angle it should drive in change as it moved relative to Polo.

However, we were struggling with implementing this in the actual game. Translating an angle towards a Polo into a angular velocity only sometimes works in implementation. We had some trouble figuring out where the zero degree started, and whether the degrees increased clockwise or counter-clockwise.

We believe the trigonometry is correct, but because we were adding angles between transforms by hand instead of using tf, we think this is part of why it didn’t work entirely. Not only that, but because we chose to entirely use tf and coordinate transforms instead of Lidar, we are relying completely on dead reckoning. Any slipping of the motors or collisions will confuse the coordinate frames compared to actual robot positions.

We decided to focus on other parts of the game logic by simplifying the calculations and allowing tf to do the math for us. We removed the logic that had Marco call out every five seconds and update a goal point, instead having him search for the closest Polo every single timestep. This change makes the game much more of a tag logical behavior than Marco Polo, but it was a choice we made in the interest of time spent on the project.

##Future Improvements

Given the minimum deliverables list we constructed at the start of this project, there are definitely some improvements we could make about behaviors we didn’t have time to implement or were unable to do so in this project.

We did write the code that would allow for robots to switch roles when a Polo got “tagged” by a Marco. However, because the entire game was not working without it to a degree with which we were pleased, we did not ever use the code in running our game. If we had more time, we would get the game working better so we could add this additional behavior.

In addition, both Marco’s and Polo’s runtime behaviors are somewhat erratic and could be improved. In theory, given the vector-force setup to direct Polo’s movement, both Polos should begin by performing symmetric actions. In a perfect world, the game would be precisely the same every time, because Marco’s runtime is determined by Polo position, and it all boils down to the same initial starting position, and therefore, in theory, the same exact gameplay. Of course, the real world is far from perfect and this didn’t happen much. One of the more baffling results of this was that the Polos are running precisely the same code at the beginning, on either side of Marco, yet at the beginning, one consistently goes to the northwest of Marco, and the other turns in place and goes south or southeast.

As mentioned earlier, we believe a significant part of the troubles with the game were in dead reckoning and even some connectivity issues with the robots. If we had more time, it might be worthwhile to implement an additional method of finding robots using Lidar, so that we could verify that the dead reckoning transforms are correct. Because as the game goes on, Marco often ends up getting worse and worse at following the Polos around, we suspect that the errors from dead reckoning are slowly accumulating and making it harder for him to actually find the Polos. In addition, it might be important to include Polo logic that would not only keep track of Marco’s position and avoid him, but also work on avoiding other Polos. At the moment, the Polos know nothing of each others’ positions and theoretically crash.

We were also thinking of separating the logic of the game into separate nodes. We would have one node that broadcasted and listened to the tf transforms, and one that simply implemented game logic. If we did this, then we would be able to debug and test one of those behaviors without having to depend on the other.

One more thing we could do would be to work with the launch file further and construct our node within that. Therefore, instead of launching our three robots separately, we could do it all at once. Building off of that, our code depends on there being three robots, named consecutively “robot1”, “robot2”, and “robot3”, and that robot2 must be the one on Marco’s right. A definitely improvement could be made here, such that the launch file passed the robotname parameters into the python code, so that it’s less rigid and more universally usable.

Along a similar vein, to work on making our code more universally usable, we could go back to our original design decision and work on incorporating SLAM so that we don’t need the static transform between frames, and the robots don’t need a designated starting position.

##Learning

We learned a lot from this project. Knowing how to use tf transforms can be incredibly useful, as well as the new deeper understanding of coordinate transforms and how they work using dead reckoning to figure out where the robot is.

We also had to learn how to work with launch files to edit the multi-agent launch to make sure our rostopics were being published correctly with the many robotnames. 

In terms of teamwork, we spent most of this project partner coding, sometimes breaking apart specific bits of logic to focus on individually. In general, a combination of the two approaches seems to be one of the best methods. It allows us to both understand fully all parts of the code, but also be able to work on the project outside of meeting times. 
