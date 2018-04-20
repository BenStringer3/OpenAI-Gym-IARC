#include "Roomba.h"

Roomba::Roomba(float pos_x, float pos_y, float heading) {
    pos[0] = pos_x;
    pos[1] = pos_y;
    heading = heading;
    collision_front = false;
    collision_top = false;
    timer_noise = 0;
    timer_reverse = 0;
    timer_touch = 0;
    state = ROOMBA_STATE_IDLE;
    turn_target = 0;
    turn_clockwise = false;
};

void Roomba::reset() {
    collision_front = false;
    collision_top = false;
    timer_noise = 0;
    timer_reverse = 0;
    timer_touch = 0;
    state = ROOMBA_STATE_IDLE;
    turn_target = 0;
    turn_clockwise = false;
}

void Roomba::start() {
    state = ROOMBA_STATE_FORWARD;
};
void Roomba::stop() {
    state = ROOMBA_STATE_IDLE;
};

Roomba::~Roomba(){};

Roomba::Roomba(){};