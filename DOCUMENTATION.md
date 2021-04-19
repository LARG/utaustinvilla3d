# Documentation
---
The following is some high level documentation about this codebase.  We hope to add more documentation in the future.

### General Process Flow 
An agent receives information from the server which it processes in `NaoBehavior::Think()` after parsing the information in [parser/parser.cc](parser/parser.cc).  Next an agent decides what to do in `NaoBehavior::act()` which typically involves selecting a skill (such as walking or a kick) to execute in `NaoBehavior::selectSkill()`.  During the execution of a skill an agent determines new positions to move its joints to which are controlled by sending velocity commands (computed by PID controllers) to the server in [servercomm/primitives.cc](servercomm/primitives.cc). 


### World Model
Agents build a model of the world based on their observations in the worldmodel directory files.  This includes localizing themselves through the use of a particle filter (files in the [particlefilter](particlefilter) directory) and using Kalman filters (files in the [kalman](kalman) directory) to track the position of the ball and opponent agents.  The world model allows for easy conversion of positions from/to local (relative to the agent) and global coordinate systems through the `worldModel->l2g()` and `worldModel->g2l()` methods.  Information about different objects in the world are stored in `WorldObjects` ([worldmodel/WorldObject.h](worldmodel/WorldObject.h)).


### Drawings in Roboviz 
Agents can draw objects in the roboviz monitor (https://github.com/magmaOffenburg/RoboViz) by sending commands to roboviz's drawing port.  See the files in the [rvdraw](rvdraw) directory for commands to draw objects in roboviz, as well as example drawing commands in [behaviors/strategy.cc](behaviors/strategy.cc) and [behaviors/naobehavior.cc](behaviors/naobehavior.cc).  By default the agent tries to connect to a version of roboviz running on the localhost, however it can connect to a remotely running instance of roboviz by using the start script command line option ```-mh <roboviz_host>``` to connect to an instance of roboviz running on a remote machine.


### Skills 
Agents have skills they can execute, such as getting up after falling and kicking the ball.  Skills typically consist of a series of fixed poses that an agent moves through.  Skills for standing up after falling are contained in [behaviors/checkfall.cc](behaviors/checkfall.cc).  Other skills, such as those for kicking, use a skill description language to define them.  Documentation about the skill description language, as well as some example kicking skills using the skill description language, are contained are in the [skills](skills) directory. 


### Parameter Files 
Values for skills and other configurable variables can be read in and loaded at runtime from parameter files.  See the [paramfiles](paramfiles) directory for more information.


### Walk Engine 
Agents use a double inverted pendulum omnidirectional walk engine to move.  Code for the walk engine can be found in the [utwalk](utwalk) directory.  To move to a target location on the field agents can use `NaoBehavior::goToTarget()` or, to specify an exact direction, rotation, and speed to the walk engine,  `NaoBehavior::getWalk()` can be used.  Parameters that control the walk engine are contained in parameter files, and a default set of walk engine parameters that provide a slow and stable walk, as well as optimized walk engine parameters for positioning/dribbling and approaching the ball to kick, are included in this release.  More information about the walk engine, as well as the process UT Austin Villa used to optimize parameters for the walk engine, can be found in the following paper:

Patrick MacAlpine, Samuel Barrett, Daniel Urieli, Victor Vu, and Peter Stone.  
Design and Optimization of an Omnidirectional Humanoid Walk: A Winning Approach at the RoboCup 2011 3D Simulation Competition.  
In Proceedings of the Twenty-Sixth AAAI Conference on Artificial Intelligence (AAAI), July 2012.
(http://www.cs.utexas.edu/~AustinVilla/sim/3dsimulation/AustinVilla3DSimulationFiles/2011/html/walk.html)


### Kicking 
Two example kicking skills are included in this code release: one basic kicking skill and the other using inverse kinematics.  To kick the ball use the `kickBall()` method which takes in both a kick type (can be either a kick or dribble) and a position to kick the ball toward.  Code for approaching the ball and executing a kick is in [behaviors/kicking.cc](behaviors/kicking.cc).  More information about what the UT Austin Villa team has done to develop and optimize kicks can be found in the following publications:

Mike Depinet, Patrick MacAlpine, and Peter Stone.  
Keyframe Sampling, Optimization, and Behavior Integration: Towards Long-Distance Kicking in the RoboCup 3D Simulation League.  
In Reinaldo A. C. Bianchi, H. Levent Akin, Subramanian Ramamoorthy, and Komei Sugiura, editors, RoboCup-2014: Robot Soccer World Cup XVIII, Lecture Notes in Artificial Intelligence, Springer Verlag, Berlin, 2015.
(http://www.cs.utexas.edu/~pstone/Papers/bib2html/b2hd-LNAI14-Depinet.html)

Patrick MacAlpine, Daniel Urieli, Samuel Barrett, Shivaram Kalyanakrishnan, Francisco Barrera, Adrian Lopez-Mobilia, Nicolae Stiurua, Victor Vu, and Peter Stone.  
UT Austin Villa 2011: A Champion Agent in the RoboCup 3D Soccer Simulation Competition.  
In Proc. of 11th Int. Conf. on Autonomous Agents and Multiagent Systems (AAMAS), June 2012.
(http://www.cs.utexas.edu/~AustinVilla/sim/3dsimulation/AustinVilla3DSimulationFiles/2011/html/kick.html)


### Communication 
See the files in the [audio](audio) directory which implement a communication system previously provided for use in drop-in player challenges.  This system, which can easily be extended, communicates basic information such as an agent's position and the position of the ball to its teammates.


### Strategy
Most strategy and high level behavior has been removed from the code release, however a few example demo behaviors are provided in [behaviors/strategy.cc](behaviors/strategy.cc).  Additionally an example simple soccer behavior with a team attempting to kick the ball toward the opponent's goal using a basic formation and dynamic greedy role assignment is provided in [behaviors/simplesoccer.cc](behaviors/simplesoccer.cc). The UT Austin Villa team has published papers about some of its high level strategy including role assignment to coordinate the movement of agents.  More information about  role assignment, and released code for performing role assignment, can be found in the following papers and locations: 

Patrick MacAlpine, Eric Price, and Peter Stone.  
SCRAM: Scalable Collision-avoiding Role Assignment with Minimal-makespan for Formational Positioning.  
In the Proceedings of the Twenty-Ninth AAAI Conference on Artificial Intelligence (AAAI-15) in Austin, Texas, USA, January 2015.
(http://www.cs.utexas.edu/~AustinVilla/sim/3dsimulation/AustinVilla3DSimulationFiles/2013/html/scram.html)

Patrick MacAlpine, Francisco Barrera, and Peter Stone.  
Positioning to Win: A Dynamic Role Assignment and Formation Positioning System.  
In Xiaoping Chen, Peter Stone, Luis Enrique Sucar, and Tijn Van der Zant, editors, RoboCup-2012: Robot Soccer World Cup XVI, Lecture Notes in Artificial Intelligence, Springer Verlag, Berlin, 2013.
(http://www.cs.utexas.edu/~AustinVilla/sim/3dsimulation/AustinVilla3DSimulationFiles/2011/html/positioning.html)


### Penalty Kick Behaviors 
Behaviors executed during penalty kicks are in [behaviors/pkbehaviors.&ast;](behaviors).  Penalty kick kickers and goalies can be started with the [start_penalty_kicker.sh](start_penalty_kicker.sh) and [start_penalty_goalie.sh](start_penalty_goalie.sh) start scripts.


### Optimization 
A considerable amount of the UT Austin Villa team's efforts in preparing for RoboCup competitions has been in the area of skill optimization and optimizing parameters for walks and kicks.  Example agents for optimizing a kick and forward walk are provided in the [optimization](optimization) directory.  Optimization agents perform some task (such as kicking a ball) and then determine how well they did at the task (such as how far they kicked the ball) with the `updateFitness()` method.  When performing an optimization task agents are able to change the world as needed (such as move themselves and the ball around) by sending training command parser commands to the server on the monitor port through the `setMonMessage()` method.  More information about optimizations performed by the team can be found in the following paper:

Patrick MacAlpine and Peter Stone.  
Overlapping Layered Learning.  
Artificial Intelligence (AIJ), 254:21-43, Elsevier, January 2018.
(http://www.cs.utexas.edu/~pstone/Papers/bib2html/b2hd-AIJ18-MacAlpine.html)


### Collecting Game Statistics/Data 
Scripts and code for collecting game statistics/data are provided in the [stats](stats) directory.


### Useful Scripts 
Some useful scripts for copying shared objects to the [libs](libs) directory to be loaded at runtime, and copying files needed to run the binary to a new directory, are provided in the [scripts](scripts) directory.


### More Documentation 
More information about the UT Austin Villa agent, including a fairly comprehensive description of its behavior, can be found in the following paper:

Patrick MacAlpine, Daniel Urieli, Samuel Barrett, Shivaram Kalyanakrishnan, Francisco Barrera, Adrian Lopez-Mobilia, Nicolae Stiurua, Victor Vu, and Peter Stone.   
UT Austin Villa 2011 3D Simulation Team Report.  
Technical Report AI11-10, The University of Texas at Austin, Department of Computer Science, AI Laboratory, 2011.
(http://www.cs.utexas.edu/~pstone/Papers/bib2html/b2hd-AI1110-macalpine.html)
