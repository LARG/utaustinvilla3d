# utaustinvilla3d
---
UT Austin Villa RoboCup 3D simulation team base code release

### About: 
This release is based off the UT Austin Villa RoboCup 3D simulation league team.  

#### What it includes:
* Omnidirectional walk engine based on a double inverted pendulum model 
* A skill description language for specifying parameterized skills/behaviors
* Getup behaviors for all agent types
* A couple basic skills for kicking one of which uses inverse kinematics
* Sample demo dribble and kick behaviors for scoring a goal
* World model and particle filter for localization
* Kalman filter for tracking objects
* All necessary parsing code for sending/receiving messages from/to the server
* Code for drawing objects in the roboviz monitor
* Communication system previously provided for use in drop-in player challenges
* Example behaviors/tasks for optimizing a kick and forward walk
* Support for Gazebo RoboCup 3D simulation plugin (https://bitbucket.org/osrf/robocup3ds)

#### What is not included: 
* The team's complete set of skills such as long kicks and goalie dives
* Optimized parameters for behaviors such as the team's fastest walks (slow and stable walk engine parameters are included, as well as optimized walk engine parameters for positioning/dribbling and approaching the ball to kick)
* High level strategy including formations and role assignment


### Requirements:
* simspark and rcssserver3d
* Boost library
* Threads library

Instructions for installing simspark and rcssserver3d:
http://simspark.sourceforge.net/wiki/index.php/Installation_on_Linux

It's optional (recommended) to install the roboviz monitor:
https://github.com/magmaOffenburg/RoboViz


### To build:
```bash
cmake . 
```
 (If cmake can't find RCSSNET3D set the SPARK_DIR environmental variable to the path where you installed the server and then rerun cmake.  Also, if you installed rcssserver3d from a package instead of building it from source, you might need to install the rcssserver3d-dev package.)
 
```bash
make
```

### Instructions for running agent:
Run full team:
```bash
./start.sh <host>
```
Run penalty kick shooter:
```bash
./start_penalty_kicker.sh <host>
```
Run penalty kick goalie:
```bash
./start_penalty_goalie.sh <host>
```
Run agent for Gazebo RoboCup 3D simulation plugin:
```bash
./start_gazebo.sh <host>
```
Kill team:
```bash
./kill.sh
```
List command line options:
```bash
./agentspark --help
```

### Documentation:
See *DOCUMENTATION* for some high level documentation about the codebase.


###Demo behaviors:
See the methods in `selectSkill()` in *behaviors/strategy.cc* for demo behaviors.


###Optimization task examples:
See the *optimization* directory.


### UT Austin Villa 3D simulation team homepage:
(http://www.cs.utexas.edu/~AustinVilla/sim/3dsimulation/)


### More information (team publications):
(http://www.cs.utexas.edu/~AustinVilla/sim/3dsimulation/publications.html)

If you use this code for research purposes, please consider citing one or more research papers listed at the above link which includes the following topics and papers:
##### Code Release
Patrick MacAlpine and Peter Stone. 
UT Austin Villa RoboCup 3D Simulation Base Code Release. 
In Sven Behnke, Daniel D. Lee, Sanem Sariel, and Raymond Sheh, editors, RoboCup 2016: Robot Soccer World Cup XX, Lecture Notes in Artificial Intelligence, Springer Verlag, Berlin, 2016.
(http://www.cs.utexas.edu/~pstone/Papers/bib2html/b2hd-LNAI16-MacAlpine2.html)

##### Walk Engine 
Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone. 
Design and Optimization of an Omnidirectional Humanoid Walk: A Winning Approach at the RoboCup 2011 3D Simulation Competition. 
In Proceedings of the Twenty-Sixth AAAI Conference on Artificial Intelligence (AAAI), July 2012.
(http://www.cs.utexas.edu/~pstone/Papers/bib2html/b2hd-AAAI12-MacAlpine.html)

##### Optimization 
Patrick MacAlpine, Mike Depinet, and Peter Stone. 
UT Austin Villa 2014: RoboCup 3D Simulation League Champion via Overlapping Layered Learning. 
In Proceedings of the Twenty-Ninth AAAI Conference on Artificial Intelligence (AAAI), January 2015.
(http://www.cs.utexas.edu/~pstone/Papers/bib2html/b2hd-AAAI15-MacAlpine2.html)

##### Winning team paper
Patrick MacAlpine, Josiah Hanna, Jason Liang, and Peter Stone. 
UT Austin Villa: RoboCup 2015 3D Simulation League Competition and Technical Challenges Champions. 
In Luis Almeida, Jianmin Ji, Gerald Steinbauer, and Sean Luke, editors, RoboCup-2015: Robot Soccer World Cup XIX, Lecture Notes in Artificial Intelligence, Springer Verlag, Berlin, 2016.
(http://www.cs.utexas.edu/~pstone/Papers/bib2html/b2hd-LNAI15-MacAlpine.html)


### UT Austin Villa team contacts:

Patrick MacAlpine (patmac@cs.utexas.edu)

Peter Stone (pstone@cs.utexas.edu)