#include "simplesoccer.h"

SimpleSoccerBehavior::SimpleSoccerBehavior( const std::string teamName,
        int uNum,
        const map<string, string>& namedParams_,
        const string& rsg_)
    : NaoBehavior( teamName,
                   uNum,
                   namedParams_,
                   rsg_) {
}

void SimpleSoccerBehavior::beam( double& beamX, double& beamY, double& beamAngle ) {
    VecPosition space = getSpaceForRole(static_cast<Role>(worldModel->getUNum()-1));
    beamX = space.getX();
    beamY = space.getY();
    beamAngle = 0;
}



SkillType SimpleSoccerBehavior::selectSkill() {
    std::map<int, Role> agentRoles = getAgentRoles();
    /*
    // Draw role assignments
    worldModel->getRVSender()->clear(); // erases drawings from previous cycle
    VecPosition space = getSpaceForRole(agentRoles[worldModel->getUNum()]);
    worldModel->getRVSender()->drawLine("me_to_space", me.getX(), me.getY(), space.getX(), space.getY());
    worldModel->getRVSender()->drawPoint("role_space", space.getX(), space.getY(), 10.0f);
    */

    if (agentRoles[worldModel->getUNum()] == ON_BALL) {
        if (me.getDistanceTo(ball) > 1) {
            // Walk to ball
            return goToTarget(ball);
        } else {
            // Kick ball toward opponent's goal
            return kickBall(KICK_FORWARD, VecPosition(HALF_FIELD_X, 0, 0));
        }
    }

    return goToSpace(getSpaceForRole(agentRoles[worldModel->getUNum()]));
}

int SimpleSoccerBehavior::getPlayerClosestToBall() {
    // Find closest player to ball
    int playerClosestToBall = -1;
    double closestDistanceToBall = 10000;
    for(int i = WO_TEAMMATE1; i < WO_TEAMMATE1+NUM_AGENTS; ++i) {
        VecPosition temp;
        int playerNum = i - WO_TEAMMATE1 + 1;
        if (worldModel->getUNum() == playerNum) {
            // This is us
            temp = worldModel->getMyPosition();
        } else {
            WorldObject* teammate = worldModel->getWorldObject( i );
            if (teammate->validPosition) {
                temp = teammate->pos;
            } else {
                continue;
            }
        }
        temp.setZ(0);

        double distanceToBall = temp.getDistanceTo(ball);
        if (distanceToBall < closestDistanceToBall) {
            playerClosestToBall = playerNum;
            closestDistanceToBall = distanceToBall;
        }
    }
    return playerClosestToBall;
}

map<int, Role> SimpleSoccerBehavior::getAgentRoles() {
    map<int, Role> agentRoles;
    set<Role> assignedRoles;
    agentRoles[getPlayerClosestToBall()] = ON_BALL;
    assignedRoles.insert(ON_BALL);
    if (!agentRoles.count(1)) {
        // Assign goalie the goalie role if goalie is not already assigned a role
        agentRoles[1] = GOALIE;
        assignedRoles.insert(GOALIE);
    }
    // Simple greedy role assignment of remaining unassigned roles
    typedef std::list<std::pair<double, std::pair<int, Role>>> dist_to_roles_list;
    dist_to_roles_list agentsDistancesToRoles;
    for (int r = 0; r < NUM_ROLES; r++) {
        Role role = static_cast<Role>(r);
        if (assignedRoles.count(role)) {
            continue;
        }
        for (int i = 1; i <= NUM_AGENTS; i++) {
            if (agentRoles.count(i)) {
                continue;
            }
            agentsDistancesToRoles.push_back(make_pair(getAgentDistanceToRole(i, role), make_pair(i, role)));
        }
    }

    agentsDistancesToRoles.sort();

    for (dist_to_roles_list::iterator it = agentsDistancesToRoles.begin(); it != agentsDistancesToRoles.end(); ++it) {
        pair<int, Role> assignment = it->second;
        int agent = assignment.first;
        Role role = assignment.second;
        if (agentRoles.count(agent) || assignedRoles.count(role)) {
            continue;
        }
        agentRoles[agent] = role;
        assignedRoles.insert(role);
        if (agentRoles.size() == NUM_AGENTS) {
            break;
        }
    }

    return agentRoles;
}

VecPosition SimpleSoccerBehavior::getSpaceForRole(Role role) {
    VecPosition ball = worldModel->getBall();
    if (beamablePlayMode()) {
        ball = VecPosition(0, 0, 0);
    }
    ball.setZ(0);

    // Keep ball position on the field
    ball.setX(max(min(ball.getX(), HALF_FIELD_X), -HALF_FIELD_X));
    ball.setY(max(min(ball.getY(), HALF_FIELD_Y), -HALF_FIELD_Y));

    VecPosition space = VecPosition(0,0,0);

    switch(role) {
    case ON_BALL:
        space = ball;
        break;
    case GOALIE:
        space = VecPosition(-HALF_FIELD_X+0.5, 0, 0);
        break;
    case SUPPORTER:
        space = ball + VecPosition(-2,0,0);
        break;
    case BACK_LEFT:
        space = ball + VecPosition(-10,3,0);
        break;
    case BACK_RIGHT:
        space = ball + VecPosition(-10,-3,0);
        break;
    case MID_LEFT:
        space = ball + VecPosition(-1,2,0);
        break;
    case MID_RIGHT:
        space = ball + VecPosition(-1,-2,0);
        break;
    case WING_LEFT:
        space = ball + VecPosition(0,7,0);
        break;
    case WING_RIGHT:
        space = ball + VecPosition(0,-7,0);
        break;
    case FORWARD_LEFT:
        space = ball + VecPosition(5,3,0);
        break;
    case FORWARD_RIGHT:
        space = ball + VecPosition(5,-3,0);
        break;
    default:
        cerr << "Unknown role: " << role << "\n";
    }

    space.setX(max(-HALF_FIELD_X, min(HALF_FIELD_X, space.getX())));
    space.setY(max(-HALF_FIELD_Y, min(HALF_FIELD_Y, space.getY())));

    if (beamablePlayMode()) {
        // Stay within your own half on kickoffs
        space.setX(min(-0.5, space.getX()));
    }

    return space;
}

double SimpleSoccerBehavior::getAgentDistanceToRole(int uNum, Role role) {
    VecPosition temp;
    if (worldModel->getUNum() == uNum) {
        // This is us
        temp = worldModel->getMyPosition();
    } else {
        WorldObject* teammate = worldModel->getWorldObject( WO_TEAMMATE1 + uNum - 1 );
        temp = teammate->pos;
    }
    temp.setZ(0);

    double distanceToRole = temp.getDistanceTo(getSpaceForRole(role));
    return distanceToRole;
}

SkillType SimpleSoccerBehavior::goToSpace(VecPosition space) {
    const double SPACE_THRESH = 0.5;
    if (me.getDistanceTo(space) < SPACE_THRESH ) {
        return watchPosition(ball);
    }

    // Adjust target to not be too close to teammates or the ball
    VecPosition target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, space, true/*keepDistance*/);
    return goToTarget(target);
}

SkillType SimpleSoccerBehavior::watchPosition(VecPosition pos) {
    const double POSITION_CENTER_THRESH = 10;
    VecPosition localPos = worldModel->g2l(pos);

    SIM::AngDeg localPosAngle = atan2Deg(localPos.getY(), localPos.getX());
    if (abs(localPosAngle) > POSITION_CENTER_THRESH) {
        return goToTargetRelative(VecPosition(), localPosAngle);
    }

    return SKILL_STAND;
}