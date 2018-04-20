#include "Agent.h"

Agent::Agent(float x_pos, float y_pos, float yaw, float z_pos)
{
    xy_vel[0] = 0;
    xy_vel[1] = 0;
    xy_pos[0] = x_pos;
    xy_pos[1] = y_pos;
    z_pos = z_pos;
    yaw = yaw;
};