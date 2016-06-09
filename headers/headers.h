#ifndef HEADERS_H
#define HEADERS_H

#include <map>
#include <string>
#include <vector>
#include <ostream>
#include <sstream>

using std::map;
using std::string;
using std::stringstream;

// TODO: is there a better way to map string to enum??????
// What is this class? see: http://stackoverflow.com/questions/726664/string-to-enum
template<typename T>
class EnumParser
{
    map<string, T> string2enum;
    map<T, string> enum2String;
    EnumParser();
    EnumParser(const EnumParser& that); // [nstiurca] disable copy constructor

    T
    _getEnumFromString(const string &value) const
    {
        if( string2enum.find( value ) != string2enum.end() ) {
            return string2enum.find( value )->second;
        } else {
            throw "string->enum mapping doesn't exist for " + value;
        }
    }

    string
    _getStringFromEnum(const T &value) const
    {
        if( enum2String.find( value ) != enum2String.end() ) {
            return enum2String.find( value )->second;
        } else {
            stringstream ss;
            ss << "enum->string mapping doesn't exist. ";
            ss << value;
            throw ss.str();
        }
    }

public:
    // [nstiurca] implement singleton pattern
    static const EnumParser<T> parser;
    static string getStringFromEnum(const T &value)
    {
        return parser._getStringFromEnum(value);
    }
    static T getEnumFromString(const string &value)
    {
        return parser._getEnumFromString(value);
    }
};

//Hinge Joints

#define HJ_H1  0
#define HJ_H2  1

#define HJ_LA1 2
#define HJ_LA2 3
#define HJ_LA3 4
#define HJ_LA4 5

#define HJ_RA1 6
#define HJ_RA2 7
#define HJ_RA3 8
#define HJ_RA4 9

#define HJ_LL1 10
#define HJ_LL2 11
#define HJ_LL3 12
#define HJ_LL4 13
#define HJ_LL5 14
#define HJ_LL6 15
#define HJ_LL7 16

#define HJ_RL1 17
#define HJ_RL2 18
#define HJ_RL3 19
#define HJ_RL4 20
#define HJ_RL5 21
#define HJ_RL6 22
#define HJ_RL7 23

#define HJ_NUM 24


//Effectors
enum Effectors {
    // NOTE!!! In any change to the enum please update the mappings in headers.cc
    EFF_H1,
    EFF_H2,

    EFF_LA1,
    EFF_LA2,
    EFF_LA3,
    EFF_LA4,

    EFF_RA1,
    EFF_RA2,
    EFF_RA3,
    EFF_RA4,

    EFF_LL1,
    EFF_LL2,
    EFF_LL3,
    EFF_LL4,
    EFF_LL5,
    EFF_LL6,
    EFF_LL7,

    EFF_RL1,
    EFF_RL2,
    EFF_RL3,
    EFF_RL4,
    EFF_RL5,
    EFF_RL6,
    EFF_RL7,

    EFF_NUM
};
/*
 * and the mapping from string to enum in headers.cc (TODO: better way?)
 */

//Flags
//#define FLAG_1_L  0
//#define FLAG_1_R  1
//#define FLAG_2_L  2
//#define FLAG_2_R  3
#define FLAG_NUM  4

//Goal Posts
//#define GOALPOST_1_L  0
//#define GOALPOST_1_R  1
//#define GOALPOST_2_L  2
//#define GOALPOST_2_R  3
#define GOALPOST_NUM  4

//Team Playing Side
#define SIDE_LEFT    0
#define SIDE_RIGHT   1

#define TEAMMATE_NUM 11
#define OPPONENT_NUM 11


//Body Components
#define COMP_TORSO     0

#define COMP_NECK      1
#define COMP_HEAD      2

#define COMP_LSHOULDER 3
#define COMP_LUPPERARM 4
#define COMP_LELBOW    5
#define COMP_LLOWERARM 6

#define COMP_RSHOULDER 7
#define COMP_RUPPERARM 8
#define COMP_RELBOW    9
#define COMP_RLOWERARM 10

#define COMP_LHIP1     11
#define COMP_LHIP2     12
#define COMP_LTHIGH    13
#define COMP_LSHANK    14
#define COMP_LANKLE    15
#define COMP_LFOOT     16

#define COMP_RHIP1     17
#define COMP_RHIP2     18
#define COMP_RTHIGH    19
#define COMP_RSHANK    20
#define COMP_RANKLE    21
#define COMP_RFOOT     22

#define COMP_NUM       23


//INDICES for body segments
enum BodyParts {
    // NOTE!!! In any change to the enum please update the mappings in headers.cc
    TORSO,
    HEAD,
    ARM_LEFT,
    ARM_RIGHT,
    LEG_LEFT,
    LEG_RIGHT,
    FOOT_LEFT,
    FOOT_RIGHT,
    TOE_LEFT,
    TOE_RIGHT
};

class bad_leg_index : public std::exception {
    std::string what_;
public:
    bad_leg_index(const int &legIndex) throw() {
        stringstream ss;
        ss << "Bad leg index: " << legIndex << ". Expected LEG_LEFT("
           << LEG_LEFT << ") or LEG_RIGHT(" << LEG_RIGHT << ").";
        what_ = ss.str();
    }

    virtual const char* what() const throw() {
        return what_.c_str();
    }

    virtual ~bad_leg_index() throw() {}
};
/*
 * and the mapping from string to enum in headers.cc (TODO: better way?)
 */

// Floating point comparison tolerance
//#define EPSILON 0.0001
const double EPSILON = 0.0001;
#define INF 1e16


//HCT Matrix Creation modes
#define HCT_IDENTITY            0
#define HCT_ROTATE_X            1
#define HCT_ROTATE_Y            2
#define HCT_ROTATE_Z            3
#define HCT_GENERALIZED_ROTATE  4
#define HCT_TRANSLATE           5

//Play Modes
#define PM_BEFORE_KICK_OFF 0
#define PM_KICK_OFF_LEFT   1
#define PM_KICK_OFF_RIGHT  2
#define PM_PLAY_ON         3
#define PM_KICK_IN_LEFT    4
#define PM_KICK_IN_RIGHT   5
#define PM_GOAL_LEFT       6
#define PM_GOAL_RIGHT      7
#define PM_GAME_OVER       8

//Extra added.
#define PM_CORNER_KICK_LEFT       9
#define PM_CORNER_KICK_RIGHT      10
#define PM_GOAL_KICK_LEFT         11
#define PM_GOAL_KICK_RIGHT        12
#define PM_OFFSIDE_LEFT           13
#define PM_OFFSIDE_RIGHT          14
#define PM_FREE_KICK_LEFT         15
#define PM_FREE_KICK_RIGHT        16
#define PM_DIRECT_FREE_KICK_LEFT  17
#define PM_DIRECT_FREE_KICK_RIGHT 18

//Directions
#define DIR_LEFT  0
#define DIR_RIGHT 1
#define DIR_BACK  2

//Skills
enum SkillType {
    SKILL_WALK_OMNI,

    SKILL_STAND,

    SKILL_KICK_LEFT_LEG,
    SKILL_KICK_RIGHT_LEG,

    SKILL_KICK_IK_0_LEFT_LEG,
    SKILL_KICK_IK_0_RIGHT_LEG,

    SKILL_NONE

};


//Kick Parameters
#define KICK_NONE -1
#define KICK_DRIBBLE 0
#define KICK_FORWARD 1
#define KICK_IK 2


// Number of agents on the team
#define NUM_AGENTS 11

#define GAZEBO_AGENT_TYPE -1


template<typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& t) {
    out << "[";
    for(typename std::vector<T>::const_iterator it = t.begin(); it != t.end(); ++it) {
        out << (*it) << ", ";
    }
    return out << "]";
}

#define LOG_BASE        clog << __FILE__ << ":" << __FUNCTION__ << "():" << __LINE__ << ":: "
#define LOG_STR(str)    LOG_BASE << str << endl
#define LOG(x)          LOG_BASE << (#x) << ": " << (x) << endl
#define LOG_MSG(msg, x) LOG_BASE << msg << ": " << x << endl
#define LOG_ST(var)     LOG_BASE << #var << ": " \
						<< EnumParser<SkillType>::getStringFromEnum(var) << endl



bool isKickSkill(SkillType skill);
bool isKickIKSkill(SkillType skill);


#endif // HEADERS_H

