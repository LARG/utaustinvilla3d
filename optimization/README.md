# Optimization
---

A couple example agents for executing a task and recording how well the agent did on the task (its fitness).  The [sample_start-optimization.sh](sample_start-optimization.sh) script runs rcssserver3d and an optimization behavior agent (fixedKickAgent or walkForwardAgent), with a provided file of parameters to test, that either attempts 10 kicks in a row (fixedKickAgent) or measures how far it can walk in 10 seconds (walkForwardAgent).  To choose between the kick or walk optimization task set the `task` variable at the beginning of the [sample_start-optimization.sh](sample_start-optimization.sh) script to either `"kick"` or `"walk"`.  Once an optimization task is completed the agent writes a fitness score to an output file.  As soon as the script detects that the output file is written it then kills the agent and server.

##### Example usage:
```bash
rm <output_file>;
./sample_start-optimization.sh <agent_body_type> <parameter_file> <output_file>;
cat <output_file>
```

Optimization behaviors use the `updateFitness()` method, which is called every simulation cycle, to monitor the progress of the agent and evaluate how well the agent is doing at the given task it is attempting.  Agents can control the state of the world (such as the playmode and positions of agents and the ball) by sending commands to the training command parser through the `setMonMessage()` method.

Remember to turn on ground truth information when running optimizations for accurate measurements and correct values for the `worldModel->getMyPositionGroundTruth()`, `worldModel->getMyAngDegGroundTruth()`, and `worldModel->getBallGroundTruth()` methods.  To do this you need to edit the *&lt;server_install_dir&gt;/share/rcssserver3d/rsg/agent/nao/naoneckhead.rsg* file and change the `setSenseMyPos`, `setSenseMyOrien`, and `setSenseBallPos` values to `true`.  You might want to call `worldModel->setUseGroundTruthDataForLocalization(true)` if the agent needs to always know exactly where it is on the field (such as might be the case when optimizing a walk and needing the agent to purposely walk to a specific target point on the field). 

Also a good idea is to turn off real-time mode and turn on sync mode for faster runs.  To do this set `$agentSyncMode` to `true` in *~/.simspark/spark.rb* and set `$enableRealTimeMode` to `false` in *&lt;server_install_dir&gt;/share/rcssserver3d/rcssserver3d.rb*.  Additionally you might want to turn off beam noise if the position of a beamed agent is being checked (as is done in the example optimization tasks).  To turn off beam noise set `BeamNoiseXY` and `BeamNoiseAngle` to `0` in *&lt;server_install_dir&gt;/share/rcssserver3d/naosoccersim.rb*.
