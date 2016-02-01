#include "headers.h"

// This file contains mapping between enums to strings, to
// be used by the parser (mainly) when reading from file

template<>
EnumParser<BodyParts>::EnumParser()
{
    string2enum["TORSO"] = TORSO;
    enum2String[TORSO] = "TORSO";

    string2enum["HEAD"] = HEAD;
    enum2String[HEAD] = "HEAD";

    string2enum["ARM_LEFT"] = ARM_LEFT;
    enum2String[ARM_LEFT] = "ARM_LEFT";

    string2enum["ARM_RIGHT"] = ARM_RIGHT;
    enum2String[ARM_RIGHT] = "ARM_RIGHT";

    string2enum["LEG_LEFT"] = LEG_LEFT;
    enum2String[LEG_LEFT] = "LEG_LEFT";

    string2enum["LEG_RIGHT"] = LEG_RIGHT;
    enum2String[LEG_RIGHT] = "LEG_RIGHT";

    string2enum["FOOT_LEFT"] = FOOT_LEFT;
    enum2String[FOOT_LEFT] = "FOOT_LEFT";

    string2enum["FOOT_RIGHT"] = FOOT_RIGHT;
    enum2String[FOOT_RIGHT] = "FOOT_RIGHT";

    string2enum["TOE_LEFT"] = TOE_LEFT;
    enum2String[TOE_LEFT] = "TOE_LEFT";

    string2enum["TOE_RIGHT"] = TOE_RIGHT;
    enum2String[TOE_RIGHT] = "TOE_RIGHT";
}
template class EnumParser<BodyParts>;
template<>
const EnumParser<BodyParts> EnumParser<BodyParts>::parser = EnumParser<BodyParts>();

//EnumParser<BodyParts> a = EnumParser<BodyParts>::parser;

template<>
EnumParser<SkillType>::EnumParser()
{
    string2enum["SKILL_WALK_OMNI"] = SKILL_WALK_OMNI;
    enum2String[SKILL_WALK_OMNI] = "SKILL_WALK_OMNI";

    string2enum["SKILL_STAND"] = SKILL_STAND;
    enum2String[SKILL_STAND] = "SKILL_STAND";

    string2enum["SKILL_KICK_LEFT_LEG"] = SKILL_KICK_LEFT_LEG;
    enum2String[SKILL_KICK_LEFT_LEG] = "SKILL_KICK_LEFT_LEG";

    string2enum["SKILL_KICK_RIGHT_LEG"] = SKILL_KICK_RIGHT_LEG;
    enum2String[SKILL_KICK_RIGHT_LEG] = "SKILL_KICK_RIGHT_LEG";



// INVERSE KINEMATICS KICKS
    string2enum["SKILL_KICK_IK_0_LEFT_LEG"] = SKILL_KICK_IK_0_LEFT_LEG;
    enum2String[SKILL_KICK_IK_0_LEFT_LEG] = "SKILL_KICK_IK_0_LEFT_LEG";

    string2enum["SKILL_KICK_IK_0_RIGHT_LEG"] = SKILL_KICK_IK_0_RIGHT_LEG;
    enum2String[SKILL_KICK_IK_0_RIGHT_LEG] = "SKILL_KICK_IK_0_RIGHT_LEG";


// END INVERSE KINEMATICS KICKS


    string2enum["SKILL_NONE"] = SKILL_NONE;
    enum2String[SKILL_NONE] = "SKILL_NONE";

}
template class EnumParser<SkillType>;
template<>
const EnumParser<SkillType> EnumParser<SkillType>::parser = EnumParser<SkillType>();

template<>
EnumParser<Effectors>::EnumParser()
{
    string2enum["EFF_H1"] = EFF_H1;
    enum2String[EFF_H1] = "EFF_H1";

    string2enum["EFF_H2"] = EFF_H2;
    enum2String[EFF_H2] = "EFF_H2";

    string2enum["EFF_LA1"] = EFF_LA1;
    enum2String[EFF_LA1] = "EFF_LA1";

    string2enum["EFF_LA2"] = EFF_LA2;
    enum2String[EFF_LA2] = "EFF_LA2";

    string2enum["EFF_LA3"] = EFF_LA3;
    enum2String[EFF_LA3] = "EFF_LA3";

    string2enum["EFF_LA4"] = EFF_LA4;
    enum2String[EFF_LA4] = "EFF_LA4";

    string2enum["EFF_RA1"] = EFF_RA1;
    enum2String[EFF_RA1] = "EFF_RA1";

    string2enum["EFF_RA2"] = EFF_RA2;
    enum2String[EFF_RA2] = "EFF_RA2";

    string2enum["EFF_RA3"] = EFF_RA3;
    enum2String[EFF_RA3] = "EFF_RA3";

    string2enum["EFF_RA4"] = EFF_RA4;
    enum2String[EFF_RA4] = "EFF_RA4";

    string2enum["EFF_LL1"] = EFF_LL1;
    enum2String[EFF_LL1] = "EFF_LL1";

    string2enum["EFF_LL2"] = EFF_LL2;
    enum2String[EFF_LL2] = "EFF_LL2";

    string2enum["EFF_LL3"] = EFF_LL3;
    enum2String[EFF_LL3] = "EFF_LL3";

    string2enum["EFF_LL4"] = EFF_LL4;
    enum2String[EFF_LL4] = "EFF_LL4";

    string2enum["EFF_LL5"] = EFF_LL5;
    enum2String[EFF_LL5] = "EFF_LL5";

    string2enum["EFF_LL6"] = EFF_LL6;
    enum2String[EFF_LL6] = "EFF_LL6";

    string2enum["EFF_LL7"] = EFF_LL7;
    enum2String[EFF_LL7] = "EFF_LL7";

    string2enum["EFF_RL1"] = EFF_RL1;
    enum2String[EFF_RL1] = "EFF_RL1";

    string2enum["EFF_RL2"] = EFF_RL2;
    enum2String[EFF_RL2] = "EFF_RL2";

    string2enum["EFF_RL3"] = EFF_RL3;
    enum2String[EFF_RL3] = "EFF_RL3";

    string2enum["EFF_RL4"] = EFF_RL4;
    enum2String[EFF_RL4] = "EFF_RL4";

    string2enum["EFF_RL5"] = EFF_RL5;
    enum2String[EFF_RL5] = "EFF_RL5";

    string2enum["EFF_RL6"] = EFF_RL6;
    enum2String[EFF_RL6] = "EFF_RL6";

    string2enum["EFF_RL7"] = EFF_RL7;
    enum2String[EFF_RL7] = "EFF_RL7";

}
template class EnumParser<Effectors>;
template<>
const EnumParser<Effectors> EnumParser<Effectors>::parser = EnumParser<Effectors>();
bool isKickSkill(SkillType skill)
{
    string skillStr = EnumParser<SkillType>::getStringFromEnum( skill );
    return skillStr.find("KICK") != string::npos;
}

bool isKickIKSkill(SkillType skill)
{
    string skillStr = EnumParser<SkillType>::getStringFromEnum( skill );
    return skillStr.find("KICK_IK") != string::npos;
}

