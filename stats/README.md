# Statistics
---

A couple scripts and code for starting a game and collecting statistics/data from the game (actually a single half).

### Running a game between our team and an opponent
The [start-game-me.sh](start-game-me.sh) script runs a game between our team and an opponent, and then the agents (started with the [start-stats.sh](start-stats.sh) start script) write out data about the game at the conclusion of the half using the `RecordStatsBehavior` class.

##### Example usage:
```bash
rm <stats_output_file>
start-game-me.sh <stats_output_file> <opponent_dir> <left | right>;
cat <stats_output_file>
```

The `stats_output_file` argument specifies the file that data is written to, the `opponent_dir` arguments specifies the directory containing the *start.sh* start script for the opponent team, and the `left | right` argument determines if our team starts on the left or right side of the field.

##### Output:
score = &lt;score_team&gt; &lt;score_opp&gt;  
average_ball_posX = &lt;avg_ball_posX&gt;  
kickoffs = &lt;num_scored&gt; &lt;num_missed&gt;  
opp_kickoffs = &lt;num_scored&gt; &lt;num_missed&gt;  
corners = &lt;num_scored&gt; &lt;num_missed&gt;  
opp_corners = &lt;num_scored&gt; &lt;num_missed&gt;  
indirect_kicks = &lt;num_scored&gt; &lt;num_missed&gt;  
opp_indirect_kicks = &lt;num_scored&gt; &lt;num_missed&gt;  
possession = &lt;percentage_of_ball_possession&gt;  
kicks&lt;unum&gt; = &lt;num_kicks&gt;  
\[missing_&lt;team | opp&gt; &lt;unum&gt;\]  
\[late_&lt;team | opp&gt; &lt;unum&gt;\]  
\[crash_&lt;team | opp&gt; &lt;unum&gt;\]  
\[all_crashed_opp\]  
\[host &lt;host&gt;\]

The score of the game is reported as the number of goals scored by our team followed by the number of goals scored by the opponent team.  The average position of the ball in the X direction is also recorded.

The number of times a teams does and does not score off of kickoffs, corner kicks, and indirect kicks for both our team and the opponent team are reported.  A team is considered to have scored after one of these kick events if they are able to score a goal within a certain amount of time after the event has occurred -- these time thresholds are defined in [recordstatsbehavior.h](recordstatsbehavior.h).

Possession refers to the percentage of time a player from our team is closest to the ball.

Each agent reports the number of times they perform a kick action/skill.

Agents' teams and uniforms numbers are reported if they are missing, late, or have crashed.  If all agents on the opponent team have crashed this is reported as well.  Additionally the name of the host machine is printed out for debugging purposes if any agents are missing, late, or have crashed.
* Missing = agent never seen
* Late = agent not present at start of game but shows up later
* Crash = agent seen but no longer present at end of game

### Running a game between any two teams
The [start-game-2teams.sh](start-game-2teams.sh) script runs a game between any two given teams, and then the script [gameMonitor.py](gameMonitor.py) listens to the game on the monitor port and, at the conclusion of the half, writes on the current score.

##### Example usage:
```bash
rm <stats_output_file>
start-game-2teams.sh <stats_output_file> <team1_dir> <team2_dir> <left | right>;
cat <stats_output_file>
```

The `stats_output_file` argument specifies the file that data is written to, the `team1_dir` and `team2_dir` arguments specify the team directories containing *start.sh* start scripts, and the `left | right` argument determines if team1 starts on the left or right side of the field.

##### Output:
score = &lt;score_left_team&gt; &lt;score_right_team&gt;  
\[missing_&lt;left | right&gt; &lt;unum&gt;\]  
\[late_&lt;left | right&gt; &lt;unum&gt;\]  
\[crash_&lt;left | right&gt; &lt;unum&gt;\]  
\[all_crashed_&lt;left|right&gt;\]  
\[host &lt;host&gt;\]

The score of the game is reported as the number of goals scored by the team starting on the left side of the field followed by the number of goals scored by the team starting on the right side of the field.

Agents' teams and uniforms numbers are reported if they are missing, late, or have crashed.  If all agents on a team have crashed this is reported as well.  Additionally the name of the host machine is printed out for debugging purposes if any agents are missing, late, or have crashed.
* Missing = agent never seen
* Late = agent not present at start of game but shows up later
* Crash = agent seen but no longer present at end of game
