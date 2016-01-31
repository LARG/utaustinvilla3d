# Optimization
---

An example agent for executing a task and recording how well it did on the task (its fitness).  The sample_start-optimization script runs rcssserver3d and an optimization behavior agent (fixedKickAgent), with a provided file of parameters to test, that attempts 10 kicks in a row.  Once the kicks are completed the agent writes a fitness score to an output file.  As soon as the script detects that the output file is written it then kills the agent and server.

##### Example usage:
```
rm <output_file>;
sample_start-optimization.sh <agent_body_type> <parameter_file> <output_file>;
cat <output_file>
```

Remember to turn on ground truth information when running optimizations for accurate measurements.  Also a good idea is to turn off real-time mode and turn on sync mode for faster runs.  Additionally you might want to turn off beam noise if the position of a beamed agent is being checked.