# Skills
---

Files for defining skills represented as a series of fixed frame poses.  Skill files contain a series of commands and values that are executed by the `SkillParser`.  Values can be parameterized and loaded at runtime when entered as `$<parameter_name>`.

To add a new skill to an agent the skill's skill file should be loaded in by calling `readSkillsFromFile()` in `NaoBehavior::NaoBehavior()`.  The skill should also be added to `enum SkillType` in [headers/headers.h](../headers/headers.h) and to `EnumParser<SkillType>` in [headers/headers.cc](../headers/headers.cc). 

Skill files are in the following format:

##### Begin and name skill
`STARTSKILL <name_of_skill>`

##### Begin a frame
`STARTSTATE` 

##### Set the target joint angle in degrees
`settar <joint> <target_angle> [<joint> <target_angle>]* end`

##### Increase the target joint angle in degrees of joints
`inctar <joint> <amount_to_increase_angle> [<joint> <amount_to_increase_angle>]* end`

##### Set scale (proportional controller value for joints)
`setscale <joint> <scale> [<joint> <scale>]* end`

##### Reset joints to default positions
`reset <joint> [joint]* end`

##### Shift center of mass over given foot to ZMP value if possible
`stabilize <LEG_LEFT | LEG_RIGHT> <x_zmp_value> <y_zmp_value>`

##### Move the foot to a preset location w.r.t. the ball in meters and degrees
`setfoot <LEG_LEFT | LEG_RIGHT> <xoffset> <yoffset> <zoffset> <role> <pitch> <yaw> end`

##### Move the foot through a path of points relative to the ball marked by controlpoints 
`STARTCURVE <LEG_LEFT | LEG_RIGHT>
controlpoint <xoffset> <yoffset> <zoffset> <role> <pitch> <yaw> end
[controlpoint <xoffset> <yoffset> <zoffset> <role> <pitch> <yaw> end]*
ENDCURVE`

##### Amount of time to wait in seconds before moving to next frame
`wait <time> end`

##### End a frame
`ENDSTATE`

##### End a skill description
`ENDSKILL`

##### Reflect skill across left-right axis to create another skill
`REFLECTSKILL <name_of_skill_to_reflect> <reflected_skill_name>`
