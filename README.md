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
* An example behavior/task for optimizing a kick

#### What is not included: 
* The team's complete set of skills such as long kicks and goalie dives
* Optimized parameters for behaviors such as walking (slow and stable walk engine parameters are included)
* High level strategy including formations and role assignment


### Requirements:
* simspark and rcssserver3d
* Boost library
* Threads library


### To build:
```bash
cmake . 
```
 (If cmake can't find RCSSNET3D set the SPARK_DIR environmental variable to the path where you installed the server and then rerun cmake)
 
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
Kill team:
```bash
./kill.sh
```
List command line options:
```bash
./agentspark --help
```

### Documentation:
See DOCUMENTATION for some high level documentation about the codebase.


###Demo behaviors:
See the methods in selectSkill() in behaviors/strategy.cc for demo behaviors.


###Optimization task example:
See the optimization directory.


### UT Austin Villa 3D simulation team homepage:
(http://www.cs.utexas.edu/~AustinVilla/sim/3dsimulation/)


### More information (team publications):
(http://www.cs.utexas.edu/~AustinVilla/sim/3dsimulation/publications.html)

If you use this code for research purposes, please consider citing one or more research papers listed at the above link which includes the following topics and papers:
##### Walk Engine 
Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone. 
Design and Optimization of an Omnidirectional Humanoid Walk:A Winning Approach at the RoboCup 2011 3D Simulation Competition. 
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