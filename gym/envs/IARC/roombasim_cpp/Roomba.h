#include <iostream>
#include <math.h>

#ifndef ROOMBA
#define ROOMBA

#define PI ((float)3.1415926535897)

//ROOMBA CONFIGURATION

//Speed of the roomba moving forwards
#define ROOMBA_LINEAR_SPEED ((float)0.33) // m/s

//Turning speed of the roomba
#define ROOMBA_ANGULAR_SPEED ((float)1.279) // rad/s

//Time until a full reverse (milliseconds)
#define ROOMBA_REVERSE_PERIOD 20000

//Time until random heading noise is applied (milliseconds)
#define ROOMBA_HEADING_NOISE_PERIOD 5000

//Maximum heading noise (applied in either direction) in radians
#define ROOMBA_HEADING_NOISE_MAX ((float)(20 * (PI / 180)))

//Time spent doing noisy turns (milliseconds)
#define ROOMBA_NOISE_DURATION 850

// Roomba's radius in meters
#define ROOMBA_RADIUS (float)(0.35 / 2)

#define DRONE_MAX_HORIZ_VELOCITY ((float)3)

#define ROOMBA_STATE_IDLE 0
#define ROOMBA_STATE_FORWARD 1
#define ROOMBA_STATE_TOUCHED 2
#define ROOMBA_STATE_REVERSING 3
#define ROOMBA_STATE_TURNING_NOISE 4

class Roomba {
    public:
        float pos[2];
        float heading;
        void start();//{ state = ROOMBA_STATE_FORWARD;}
        void stop();//{ state = ROOMBA_STATE_IDLE;}
        bool collision_front;
        bool collision_top;
        int timer_reverse;
        int timer_noise;
        int timer_touch;
        int state;
        virtual void update(float delta, int elapsed) =0;
        virtual void seed(int seed) = 0;
        Roomba(float pos_x, float pos_y, float heading);
        void reset();
        ~Roomba();
        Roomba();

    protected:
        float angular_noise_velocity;
        float turn_target;
        bool turn_clockwise;
        float turn_time;


};
#endif