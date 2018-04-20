#include "TargetRoomba.h"
//#include <random>

using namespace std;


TargetRoomba::TargetRoomba(float pos_x, float pos_y, float heading)
: Roomba(pos_x, pos_y, heading),
    distr(std::uniform_real_distribution<float>(-ROOMBA_HEADING_NOISE_MAX, ROOMBA_HEADING_NOISE_MAX)),
//    rand_dev(std::random_device()),
    generator( std::mt19937(rand_dev()))
{};

TargetRoomba::~TargetRoomba(){};

TargetRoomba::TargetRoomba(){};

void TargetRoomba::seed(int seed){
    generator.seed(seed);
//    cout << "seed: " << seed << endl;
}

void TargetRoomba::update(float delta, int elapsed) {
//    std::random_device                  rand_dev;
//    std::mt19937                        generator(rand_dev());
//    std::uniform_real_distribution<float>  distr(-ROOMBA_HEADING_NOISE_MAX, ROOMBA_HEADING_NOISE_MAX);
    if (state == ROOMBA_STATE_FORWARD) {
        if (collision_top) {
            state = ROOMBA_STATE_TOUCHED;
            collision_top = false;
            timer_touch = elapsed;
        }
        else if (elapsed - timer_reverse > ROOMBA_REVERSE_PERIOD) {
            state = ROOMBA_STATE_REVERSING;
            timer_reverse = elapsed;
        }
        else if (elapsed - timer_noise > ROOMBA_HEADING_NOISE_PERIOD) {
            state = ROOMBA_STATE_TURNING_NOISE;
            angular_noise_velocity = (distr(generator) / (ROOMBA_NOISE_DURATION / 1000.0));
            timer_noise = elapsed;
        }
        else if (collision_front) {
            collision_front = false;
            state = ROOMBA_STATE_REVERSING;
            timer_reverse = elapsed;
        }
        else {
            pos[0] += ROOMBA_LINEAR_SPEED * cos(heading) * delta;
            pos[1] += ROOMBA_LINEAR_SPEED * sin(heading) * delta;
        }
    }
    else if (state == ROOMBA_STATE_TOUCHED) {
        collision_top = false; //TODO: Is this right?
        turn_time = (PI / 4) / ROOMBA_ANGULAR_SPEED;
        if (elapsed - timer_touch >= turn_time * 1000) {
            state = ROOMBA_STATE_FORWARD;
        }
        else if ( collision_front) {
            state = ROOMBA_STATE_REVERSING;
            collision_front = false;
            timer_reverse = elapsed;
        }
        else {
            heading -= ROOMBA_ANGULAR_SPEED * delta;
        }
    }
    else if (state ==ROOMBA_STATE_REVERSING) {
        collision_front = false;// #TODO: Is this right?
        if (collision_top) {
            collision_top = false;
            state = ROOMBA_STATE_TOUCHED;
            timer_touch = elapsed;
        }
        else if ( elapsed - timer_reverse >= PI / ROOMBA_ANGULAR_SPEED * 1000) {
            state = ROOMBA_STATE_FORWARD;
        }
        else{
            heading -= ROOMBA_ANGULAR_SPEED * delta;
        }
        collision_front = false;
    }
    else if (state == ROOMBA_STATE_TURNING_NOISE) {
        if (collision_top){
            collision_top = false;
            state = ROOMBA_STATE_TOUCHED;
            timer_touch = elapsed;
        }
        else if (elapsed - timer_noise >= ROOMBA_NOISE_DURATION) {
            state = ROOMBA_STATE_FORWARD;
        }
        else if (collision_front) {
            collision_front = false;
            state = ROOMBA_STATE_REVERSING;
            timer_reverse = elapsed;
        }
        else{
            heading += angular_noise_velocity * delta;
            pos[0] += ROOMBA_LINEAR_SPEED * cos(heading) * delta;
            pos[1] += ROOMBA_LINEAR_SPEED * sin(heading) * delta;
        }
    }
    else if (state != ROOMBA_STATE_IDLE){
        cout << "roomba state error" << endl;
    }

    while (heading > 2*PI) {heading -= 2*PI;}
    while (heading < 0) {heading += 2*PI;}
};

