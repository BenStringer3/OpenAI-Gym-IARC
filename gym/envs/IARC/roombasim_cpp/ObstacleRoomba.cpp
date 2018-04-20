#include "ObstacleRoomba.h"

ObstacleRoomba::ObstacleRoomba(float pos_x, float pos_y, float heading) : Roomba(pos_x, pos_y, heading){};

ObstacleRoomba::ObstacleRoomba(){}

ObstacleRoomba::~ObstacleRoomba(){}

void ObstacleRoomba::seed(int seed){};

void ObstacleRoomba::update(float delta, int elapsed) {
    float ang;
    if (collision_front) {
        collision_front = false;
    }
    else if (state == ROOMBA_STATE_FORWARD){
        pos[0] += ROOMBA_LINEAR_SPEED * cos(heading) * delta;
        pos[1] += ROOMBA_LINEAR_SPEED * sin(heading) * delta;
        ang =atan2(10 - pos[1], 10 - pos[0]);
        heading = ang + PI/2;
    }
};