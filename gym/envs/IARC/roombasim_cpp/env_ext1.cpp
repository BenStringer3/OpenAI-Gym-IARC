//#include "/usr/local/include/boost_1_66_0/boost/python.hpp"
//#include "/usr/local/include/boost_1_66_0/boost/python/numpy.hpp"
#include "env_ext1.h"
//#include "TargetRoomba.h"
//#include "Agent.h"
//#include "ObstacleRoomba.h"
//#include <string>
//#include <map>





#define TAU ((float)(2*PI))

//
// MISSION CONFIGURATION
//

#define NUM_RENDER_SECS 2

// number of target roombas to spawn
#define MISSION_NUM_TARGETS  10

// radius to spawn target roombas (centered at origin) in meters
#define MISSION_TARGET_SPAWN_RADIUS  1

// number of obstacle roombas to spawn
#define MISSION_NUM_OBSTACLES  4

// radius to spawn obstacle roombas in meters
#define MISSION_OBSTACLE_SPAWN_RADIUS  5



bool Env::check_rmba_collision(const Roomba * rmba1, const Roomba * rmba2){
     float dist = pow(rmba1->pos[0] - rmba2->pos[0], 2) + pow(rmba1->pos[1] - rmba2->pos[1], 2);
     return dist < (4*ROOMBA_RADIUS*ROOMBA_RADIUS);
}

bool Env::check_rmba_is_facing(const Roomba * rmba, const float * pos) {

    /*Returns true if roomba ra is facing the point pos.

    This works by determining the angle of the vector
    ra -> pos relative to the +x axis and checking if the
    heading of ra is within pi/2 radians of that result. */

    float ang = atan2(pos[1] - rmba->pos[1], pos[0] - rmba->pos[0]);
    return abs(fmod(((rmba->heading - ang) + PI), TAU) - PI) < PI / 2;
}

void Env::check_bounds(const Roomba * rmba, bool & has_left, int & rew){

    /*Check if a roomba has left the arena.

    Returns (has_left, reward):

    has_left - True if the roomba is outside the arena
    reward - 1 only if the roomba crossed the goal line,
        0 otherwise*/

    has_left = false;
    rew = -1000;

    if (rmba->pos[0] < -ROOMBA_RADIUS
        || rmba->pos[1] < -ROOMBA_RADIUS
        || rmba->pos[0] > 20 + ROOMBA_RADIUS
        || rmba->pos[1] > 20 + ROOMBA_RADIUS){
        has_left = true;
    }

    if (rmba->pos[0] > 20 + ROOMBA_RADIUS){
        rew = 2000;
    }
}

Env::Env() {
    float pos[2];
    float theta;
    last_rmba = 4;
    earlyTerminationTime_ms = 10*60*1000;
    Py_Initialize();
    np::initialize();

    agent = new Agent(13, 10, 0, 2);
    for (int i = 0; i < MISSION_NUM_TARGETS; i++) {
        theta = (TAU * (float)i) / (float)MISSION_NUM_TARGETS;
        pos[0] = cos(theta) * MISSION_TARGET_SPAWN_RADIUS + 10;
        pos[1] = sin(theta) * MISSION_TARGET_SPAWN_RADIUS + 10;
        roombas[i] = new TargetRoomba(pos[0], pos[1], theta);
        roombas[i]->start();
    }

    for (int i = MISSION_NUM_TARGETS; i < MISSION_NUM_OBSTACLES + MISSION_NUM_TARGETS; i++) {
        theta = (TAU * i) / MISSION_NUM_OBSTACLES;
        pos[0] = cos(theta) * MISSION_OBSTACLE_SPAWN_RADIUS + 10;
        pos[1] = sin(theta) * MISSION_OBSTACLE_SPAWN_RADIUS + 10;
        roombas[i] = new ObstacleRoomba(pos[0], pos[1], (float)(theta - PI/2));
        roombas[i]->start();
    }

//            rews_map.insert(make_pair("game_reward", 0));
//            rews_map.insert(make_pair("speed_reward", 0));
//            rews_map.insert(make_pair("direction_reward", 0));
//            rews_map.insert(make_pair("target_reward", 0));
//            rews_map.insert(make_pair("num_good_exits", 0));
//            rews_map.insert(make_pair("num_bad_exits", 0));

    reset();
}

void Env::reset(){
    float pos[2];
    float theta;
    num_good_exits = 0;
    num_bad_exits = 0;
    time_ms = 0;
//            time = 0;
    score = 0;

    agent->xy_vel[0] = 0;
    agent->xy_vel[1] = 0;
    agent->xy_pos[0] = 13;
    agent->xy_pos[1] = 10;
    agent->yaw = 0;
    agent->z_pos = 2;

//            rews_map["game_reward"] =  0;
//            rews_map["speed_reward"]= 0;
//            rews_map["direction_reward"]= 0;
//            rews_map["target_reward"]= 0;
//            rews_map["num_good_exits"]= 0;
//            rews_map["num_bad_exits"]= 0;
    rewards.last_num_bad_exits = 0;
    rewards.last_num_good_exits=0;
    rewards.last_game_score = 0;

    for (int i = 0; i < MISSION_NUM_TARGETS; i++) {
        theta = (TAU * i) / MISSION_NUM_TARGETS;
        pos[0] = cos(theta) * MISSION_TARGET_SPAWN_RADIUS + 10;
        pos[1] = sin(theta) * MISSION_TARGET_SPAWN_RADIUS + 10;
        roombas[i]->reset();
        roombas[i]->pos[0] = pos[0];
        roombas[i]->pos[1] = pos[1];
        roombas[i]->heading = theta;
        roombas[i]->start();
    }

    for (int i = MISSION_NUM_TARGETS; i < MISSION_NUM_OBSTACLES + MISSION_NUM_TARGETS; i++) {
        theta = (TAU * i) / MISSION_NUM_OBSTACLES;
        pos[0] = cos(theta) * MISSION_OBSTACLE_SPAWN_RADIUS + 10;
        pos[1] = sin(theta) * MISSION_OBSTACLE_SPAWN_RADIUS + 10;
        roombas[i]->reset();
        roombas[i]->pos[0] = pos[0];
        roombas[i]->pos[1] = pos[1];
        roombas[i]->heading = theta - PI/2;
        roombas[i]->start();
    }
}

void Env::set_earlyTerminationTime_ms(int earlyTerminationTime_ms) {
    earlyTerminationTime_ms = earlyTerminationTime_ms;
}

void Env::seed(np::ndarray seeds) {
    for (int i =0; i < MISSION_NUM_TARGETS; i++) {
        roombas[i]->seed(p::extract<int>(seeds[i]));
    }
}

p::dict Env::get_rews_dict() {
    p::dict rews;
    rews["game_reward"] = rewards.game_reward;
    rews["speed_reward"] = rewards.speed_reward;
    rews["direction_reward"] = rewards.direction_reward;
    rews["target_reward"] = rewards.target_reward;
    rews["num_good_exits"] = rewards.num_good_exits;
    rews["num_bad_exits"] = rewards.num_bad_exits;
    return rews;
}

void Env::masterUpdate(int rmba_index, bool top_or_front, bool engageTarget) {
    static float max_drone_speed = 3;
    float  aav_targ_pos[2];
    float dist2targ, ang;

    rewards.reset();

    for (int i = 0; i < NUM_RENDER_SECS*10; i++) {
        aav_targ_pos[0] = roombas[rmba_index]->pos[0];
        aav_targ_pos[1] = roombas[rmba_index]->pos[1];
        dist2targ = sqrt(pow(agent->xy_pos[0] - aav_targ_pos[0], 2) + pow(agent->xy_pos[1] - aav_targ_pos[1],2));

        if (dist2targ <= 0.35) {
            agent->xy_vel[0] = 0;
            agent->xy_vel[1] = 0;
            if (engageTarget) {
                if (top_or_front) {
                    roombas[rmba_index]->collision_top = true;
                }
                else {
                    roombas[rmba_index]->collision_front = true;
                }
            }
        }
        else {
            ang = atan2(aav_targ_pos[1] - agent->xy_pos[1], aav_targ_pos[0] - agent->xy_pos[0]);
            agent->xy_vel[0] = max_drone_speed*cos(ang);
            agent->xy_vel[1] = max_drone_speed*sin(ang);
        }
        time_ms += 100;
        update(0.1, time_ms);
    }
//            rews_map["target_reward"] = -2*NUM_RENDER_SECS*(roombas[rmba_index]->state == ROOMBA_STATE_IDLE);
//            rews_map["direction_reward"] = 0.3*get_direction_reward();
//            rews_map["game_reward"] = score/100 - rewards.last_game_score;
//            rewards.last_game_score = score/100;
//            rews_map["speed_reward"] = rewards.game_reward * 0.3 * (10 * 60 * 1000 - time_ms) / 1000 / 60 / 10;
//            rews_map["num_good_exits"] = num_good_exits - rewards.last_num_good_exits;
//            rews_map["num_bad_exits"] = num_bad_exits - rewards.last_num_bad_exits;
//            rewards.last_num_good_exits = num_good_exits;
//            rewards.last_num_bad_exits = num_bad_exits;
//            rews = p::dict(rews_map);
    rewards.target_reward = -2*NUM_RENDER_SECS*(roombas[rmba_index]->state == ROOMBA_STATE_IDLE);
    rewards.direction_reward = 0.3*get_direction_reward();
    rewards.game_reward = score/100 - rewards.last_game_score;
    rewards.last_game_score = score/100;
    rewards.speed_reward = rewards.game_reward * 0.3 * (10 * 60 * 1000 - time_ms) / 1000 / 60 / 10;
    rewards.num_good_exits = num_good_exits - rewards.last_num_good_exits;
    rewards.num_bad_exits = num_bad_exits - rewards.last_num_bad_exits;
    rewards.last_num_good_exits = num_good_exits;
    rewards.last_num_bad_exits = num_bad_exits;
    total_rew = rewards.direction_reward +  \
    rewards.target_reward + \
    rewards.game_reward + \
    rewards.speed_reward;
    if ((num_bad_exits + num_good_exits) >= MISSION_NUM_TARGETS){
        done = true;
    }
    else if(earlyTerminationTime_ms <= time_ms) {
        done = true;
    }
//            else if( time_ms >= 10 * 60 * 1000) {
//                done = true;
//            }
    else {
        done = false;
    }
}

void Env::update(float delta, int elapsed){
    int rew = 0;
    bool has_left = false;
    float tmp;
    time_ms = elapsed;
    for (int i = 0; i < MISSION_NUM_TARGETS + MISSION_NUM_OBSTACLES; i++) {
        if (roombas[i]->state == ROOMBA_STATE_IDLE){continue;}
        roombas[i]->update(delta, elapsed);

        for (int j = 0; j < MISSION_NUM_TARGETS + MISSION_NUM_OBSTACLES; j++) {
            if ( i == j || roombas[j]->state == ROOMBA_STATE_IDLE) {continue;}
            if (check_rmba_collision(roombas[i], roombas[j]) && check_rmba_is_facing(roombas[i], roombas[j]->pos)) {
                roombas[i]->collision_front = true;
            }
        }

        check_bounds(roombas[i], has_left, rew);
        if (has_left){
            if (rew > 0) {
                num_good_exits += 1;
            }
            else {
                num_bad_exits += 1;
            }
            score += rew;
            roombas[i]->stop();
        }
    }
//            tmp = sqrt(pow(agent->xy_vel[0],2) + pow(agent->xy_vel[1],2));
//            if (tmp > DRONE_MAX_HORIZ_VELOCITY) {
//                cout << tmp << endl;
//                agent->xy_vel[0] *= (DRONE_MAX_HORIZ_VELOCITY / tmp);
//                agent->xy_vel[1] *= (DRONE_MAX_HORIZ_VELOCITY / tmp);
//            }

    agent->xy_pos[0] += agent->xy_vel[0] * delta;
    agent->xy_pos[1] += agent->xy_vel[1] * delta;
}

np::ndarray Env::get_state() {
    static np::ndarray state = np::zeros(p::make_tuple(43), np::dtype::get_builtin<float>());

    for (int i = 0; i < MISSION_NUM_TARGETS; i++){

        state[i*4 + 0] = roombas[i]->pos[0]; //p::extract<float>(roombas[i]->pos[0]);
        state[i*4 + 1] = roombas[i]->pos[1]; //p::extract<float>(roombas[i]->pos[1]);
        state[i*4 + 2] = roombas[i]->heading;
        state[i*4 + 3] = (roombas[i]->state == ROOMBA_STATE_FORWARD);
    }

    state[40] = agent->xy_pos[0];
    state[41] = agent->xy_pos[1];
    state[42] = last_rmba;

    return state;
}

np::ndarray Env::get_screen() {
    const int img_size = 32;
    static np::ndarray arr = np::zeros(p::make_tuple(32, 32), np::dtype::get_builtin<int>());
    for (int i=0; i < MISSION_NUM_TARGETS; i++){
        if (roombas[i]->state != ROOMBA_STATE_IDLE) {
            arr[int(roombas[i]->pos[0] / 20.0 * (img_size - 1))][ int(roombas[i]->pos[1] / 20.0 * (img_size - 1))] = 255;
        }
    }
    return arr;
}

np::ndarray Env::get_target_pos(int rmba_index){
    return np::from_data(roombas[rmba_index]->pos, np::dtype::get_builtin<float>(),
                             p::make_tuple(2),
                             p::make_tuple(sizeof(float)),
                             p::object());
}

int Env::get_target_state(int rmba_index){
    return roombas[rmba_index]->state;
}

float Env::get_target_heading(int rmba_index){
    return roombas[rmba_index]->heading;
}

float Env::get_direction_reward() {
    float dir_rew = 0;
    for (int i=0; i < MISSION_NUM_TARGETS; i++){
        if (roombas[i]->state != ROOMBA_STATE_IDLE) {
            dir_rew += ((float)(roombas[i]->state == ROOMBA_STATE_FORWARD))*20* NUM_RENDER_SECS *  1/ PI/3 * (fabs(PI - roombas[i]->heading) - PI/2) / (MISSION_NUM_TARGETS -(num_good_exits+num_bad_exits));
        }
    }
    return dir_rew;
}

void Env::collision(int rmba_index, bool top_or_front) {
    if (top_or_front) {
        roombas[rmba_index]->collision_top= true;
    }
    else {
        roombas[rmba_index]->collision_front = true;
    }
}

np::ndarray Env::get_agent_pos() {
    return np::from_data(agent->xy_pos, np::dtype::get_builtin<float>(),
                             p::make_tuple(2),
                             p::make_tuple(sizeof(float)),
                             p::object());
}

np::ndarray Env::get_agent_vel() {
    return np::from_data(agent->xy_vel, np::dtype::get_builtin<float>(),
                             p::make_tuple(2),
                             p::make_tuple(sizeof(float)),
                             p::object());
}

float Env::get_agent_zpos() {
    return agent->z_pos;
}

float Env::get_agent_yaw() {
    return agent->yaw;
}

void Env::set_agent_vel(np::ndarray vel) {
    agent->xy_vel[0] = p::extract<float>(vel[0]);
    agent->xy_vel[1] = p::extract<float>(vel[1]);
}


BOOST_PYTHON_MODULE(env_ext1)

{

    using namespace boost::python;
    class_<Env>("Env")
	.def("update", &Env::update)
	.def("seed", &Env::seed)
	.def("reset", &Env::reset)
	.def_readonly("num_good_exits", &Env::num_good_exits)
	.def_readonly("num_bad_exits", &Env::num_bad_exits)
	.def_readonly("score", &Env::score)
	.def_readonly("time_ms", &Env::time_ms)
	.def("get_agent_pos", &Env::get_agent_pos)
	.def("get_agent_zpos", &Env::get_agent_zpos)
	.def("get_agent_yaw", &Env::get_agent_yaw)
	.def("set_agent_vel", &Env::set_agent_vel)
	.def("get_agent_vel", &Env::get_agent_vel)
	.def("get_state", &Env::get_state)
	.def("get_screen", &Env::get_screen)
	.def("get_target_pos", &Env::get_target_pos)
	.def("get_target_state", &Env::get_target_state)
	.def("get_target_heading", &Env::get_target_heading)
	.def("get_direction_reward", &Env::get_direction_reward)
	.def("collision", &Env::collision)
	.def("masterUpdate", &Env::masterUpdate)
	.def("set_earlyTerminationTime_ms", &Env::set_earlyTerminationTime_ms)
	.def_readonly("done", &Env::done)
	.def_readonly("total_rew", &Env::total_rew)
	.def("get_rews_dict", &Env::get_rews_dict)
//	.def("set_last_rmba", &Env::set_last_rmba)
//	.def_readwrite("roombas", &Env::roombas)
	;


}




//using namespace std;
//namespace np = boost::python::numpy;
//namespace p = boost::python;
//
//class Env{
//   private:
//
//        bool check_rmba_collision(const Roomba * rmba1, const Roomba * rmba2){
//             float dist = pow(rmba1->pos[0] - rmba2->pos[0], 2) + pow(rmba1->pos[1] - rmba2->pos[1], 2);
//             return dist < (4*ROOMBA_RADIUS*ROOMBA_RADIUS);
//        }
//        bool check_rmba_is_facing(const Roomba * rmba, const float * pos) {
//
//            /*Returns true if roomba ra is facing the point pos.
//
//            This works by determining the angle of the vector
//            ra -> pos relative to the +x axis and checking if the
//            heading of ra is within pi/2 radians of that result. */
//
//            float ang = atan2(pos[1] - rmba->pos[1], pos[0] - rmba->pos[0]);
//            return abs(fmod(((rmba->heading - ang) + PI), TAU) - PI) < PI / 2;
//        }
//        void check_bounds(const Roomba * rmba, bool & has_left, int & rew){
//
//            /*Check if a roomba has left the arena.
//
//            Returns (has_left, reward):
//
//            has_left - True if the roomba is outside the arena
//            reward - 1 only if the roomba crossed the goal line,
//                0 otherwise*/
//
//            has_left = false;
//            rew = -1000;
//
//            if (rmba->pos[0] < -ROOMBA_RADIUS
//                || rmba->pos[1] < -ROOMBA_RADIUS
//                || rmba->pos[0] > 20 + ROOMBA_RADIUS
//                || rmba->pos[1] > 20 + ROOMBA_RADIUS){
//                has_left = true;
//            }
//
//            if (rmba->pos[0] > 20 + ROOMBA_RADIUS){
//                rew = 2000;
//            }
//        }
//        Agent * agent;
//        int last_rmba;
//        int earlyTerminationTime_ms;
//    public:
//
//        Env() {
//            float pos[2];
//            float theta;
//            last_rmba = 4;
//            earlyTerminationTime_ms = 10*60*1000;
//            Py_Initialize();
//            np::initialize();
//
//            agent = new Agent(13, 10, 0, 2);
//            for (int i = 0; i < MISSION_NUM_TARGETS; i++) {
//                theta = (TAU * (float)i) / (float)MISSION_NUM_TARGETS;
//                pos[0] = cos(theta) * MISSION_TARGET_SPAWN_RADIUS + 10;
//                pos[1] = sin(theta) * MISSION_TARGET_SPAWN_RADIUS + 10;
//                roombas[i] = new TargetRoomba(pos[0], pos[1], theta);
//                roombas[i]->start();
//            }
//
//            for (int i = MISSION_NUM_TARGETS; i < MISSION_NUM_OBSTACLES + MISSION_NUM_TARGETS; i++) {
//                theta = (TAU * i) / MISSION_NUM_OBSTACLES;
//                pos[0] = cos(theta) * MISSION_OBSTACLE_SPAWN_RADIUS + 10;
//                pos[1] = sin(theta) * MISSION_OBSTACLE_SPAWN_RADIUS + 10;
//                roombas[i] = new ObstacleRoomba(pos[0], pos[1], (float)(theta - PI/2));
//                roombas[i]->start();
//            }
//
////            rews_map.insert(make_pair("game_reward", 0));
////            rews_map.insert(make_pair("speed_reward", 0));
////            rews_map.insert(make_pair("direction_reward", 0));
////            rews_map.insert(make_pair("target_reward", 0));
////            rews_map.insert(make_pair("num_good_exits", 0));
////            rews_map.insert(make_pair("num_bad_exits", 0));
//
//            reset();
//        }
//        void reset(){
//            float pos[2];
//            float theta;
//            num_good_exits = 0;
//            num_bad_exits = 0;
//            time_ms = 0;
////            time = 0;
//            score = 0;
//
//            agent->xy_vel[0] = 0;
//            agent->xy_vel[1] = 0;
//            agent->xy_pos[0] = 13;
//            agent->xy_pos[1] = 10;
//            agent->yaw = 0;
//            agent->z_pos = 2;
//
////            rews_map["game_reward"] =  0;
////            rews_map["speed_reward"]= 0;
////            rews_map["direction_reward"]= 0;
////            rews_map["target_reward"]= 0;
////            rews_map["num_good_exits"]= 0;
////            rews_map["num_bad_exits"]= 0;
//            rewards.last_num_bad_exits = 0;
//            rewards.last_num_good_exits=0;
//            rewards.last_game_score = 0;
//
//            for (int i = 0; i < MISSION_NUM_TARGETS; i++) {
//                theta = (TAU * i) / MISSION_NUM_TARGETS;
//                pos[0] = cos(theta) * MISSION_TARGET_SPAWN_RADIUS + 10;
//                pos[1] = sin(theta) * MISSION_TARGET_SPAWN_RADIUS + 10;
//                roombas[i]->reset();
//                roombas[i]->pos[0] = pos[0];
//                roombas[i]->pos[1] = pos[1];
//                roombas[i]->heading = theta;
//                roombas[i]->start();
//            }
//
//            for (int i = MISSION_NUM_TARGETS; i < MISSION_NUM_OBSTACLES + MISSION_NUM_TARGETS; i++) {
//                theta = (TAU * i) / MISSION_NUM_OBSTACLES;
//                pos[0] = cos(theta) * MISSION_OBSTACLE_SPAWN_RADIUS + 10;
//                pos[1] = sin(theta) * MISSION_OBSTACLE_SPAWN_RADIUS + 10;
//                roombas[i]->reset();
//                roombas[i]->pos[0] = pos[0];
//                roombas[i]->pos[1] = pos[1];
//                roombas[i]->heading = theta - PI/2;
//                roombas[i]->start();
//            }
//        }
//        void set_earlyTerminationTime_ms(int earlyTerminationTime_ms) {
//            earlyTerminationTime_ms = earlyTerminationTime_ms;
//        }
//        void seed(np::ndarray seeds) {
//            for (int i =0; i < MISSION_NUM_TARGETS; i++) {
//                roombas[i]->seed(p::extract<int>(seeds[i]));
//            }
//        }
////        map<string, float> rews_map;
////        p::dict rews;
//        float total_rew;
//        struct Rewards {
//            float game_reward;
//            float last_game_score;
//            float speed_reward;
//            float direction_reward;
//            float target_reward;
//            float num_good_exits;
//            float last_num_good_exits;
//            float num_bad_exits;
//            float last_num_bad_exits;
//            void reset() {
//                game_reward = 0;
//                speed_reward = 0;
//                direction_reward = 0;
//                target_reward = 0;
//                num_good_exits = 0;
//                num_bad_exits = 0;
//            }
//            Rewards()
//            : game_reward(0),
//            last_game_score(0),
//            speed_reward(0),
//            direction_reward(0),
//            target_reward(0),
//            num_good_exits(0),
//            last_num_good_exits(0),
//            num_bad_exits(0),
//            last_num_bad_exits(0)
//            { }
//        }rewards;
//
//        Roomba * roombas[MISSION_NUM_OBSTACLES + MISSION_NUM_TARGETS];
//        int num_good_exits;
//        int num_bad_exits;
//        int score;
////        float time;
//        int time_ms;
//        bool done;
//        p::dict get_rews_dict() {
//            p::dict rews;
//            rews["game_reward"] = rewards.game_reward;
//            rews["speed_reward"] = rewards.speed_reward;
//            rews["direction_reward"] = rewards.direction_reward;
//            rews["target_reward"] = rewards.target_reward;
//            rews["num_good_exits"] = rewards.num_good_exits;
//            rews["num_bad_exits"] = rewards.num_bad_exits;
//            return rews;
//        }
//        void masterUpdate(int rmba_index, bool top_or_front, bool engageTarget) {
//            static float max_drone_speed = 3;
//            float  aav_targ_pos[2];
//            float dist2targ, ang;
//
//            rewards.reset();
//
//            for (int i = 0; i < NUM_RENDER_SECS*10; i++) {
//                aav_targ_pos[0] = roombas[rmba_index]->pos[0];
//                aav_targ_pos[1] = roombas[rmba_index]->pos[1];
//                dist2targ = sqrt(pow(agent->xy_pos[0] - aav_targ_pos[0], 2) + pow(agent->xy_pos[1] - aav_targ_pos[1],2));
//
//                if (dist2targ <= 0.35) {
//                    agent->xy_vel[0] = 0;
//                    agent->xy_vel[1] = 0;
//                    if (engageTarget) {
//                        if (top_or_front) {
//                            roombas[rmba_index]->collision_top = true;
//                        }
//                        else {
//                            roombas[rmba_index]->collision_front = true;
//                        }
//                    }
//                }
//                else {
//                    ang = atan2(aav_targ_pos[1] - agent->xy_pos[1], aav_targ_pos[0] - agent->xy_pos[0]);
//                    agent->xy_vel[0] = max_drone_speed*cos(ang);
//                    agent->xy_vel[1] = max_drone_speed*sin(ang);
//                }
//                time_ms += 100;
//                update(0.1, time_ms);
//            }
////            rews_map["target_reward"] = -2*NUM_RENDER_SECS*(roombas[rmba_index]->state == ROOMBA_STATE_IDLE);
////            rews_map["direction_reward"] = 0.3*get_direction_reward();
////            rews_map["game_reward"] = score/100 - rewards.last_game_score;
////            rewards.last_game_score = score/100;
////            rews_map["speed_reward"] = rewards.game_reward * 0.3 * (10 * 60 * 1000 - time_ms) / 1000 / 60 / 10;
////            rews_map["num_good_exits"] = num_good_exits - rewards.last_num_good_exits;
////            rews_map["num_bad_exits"] = num_bad_exits - rewards.last_num_bad_exits;
////            rewards.last_num_good_exits = num_good_exits;
////            rewards.last_num_bad_exits = num_bad_exits;
////            rews = p::dict(rews_map);
//            rewards.target_reward = -2*NUM_RENDER_SECS*(roombas[rmba_index]->state == ROOMBA_STATE_IDLE);
//            rewards.direction_reward = 0.3*get_direction_reward();
//            rewards.game_reward = score/100 - rewards.last_game_score;
//            rewards.last_game_score = score/100;
//            rewards.speed_reward = rewards.game_reward * 0.3 * (10 * 60 * 1000 - time_ms) / 1000 / 60 / 10;
//            rewards.num_good_exits = num_good_exits - rewards.last_num_good_exits;
//            rewards.num_bad_exits = num_bad_exits - rewards.last_num_bad_exits;
//            rewards.last_num_good_exits = num_good_exits;
//            rewards.last_num_bad_exits = num_bad_exits;
//            total_rew = rewards.direction_reward +  \
//            rewards.target_reward + \
//            rewards.game_reward + \
//            rewards.speed_reward;
//            if ((num_bad_exits + num_good_exits) >= MISSION_NUM_TARGETS){
//                done = true;
//            }
//            else if(earlyTerminationTime_ms <= time_ms) {
//                done = true;
//            }
////            else if( time_ms >= 10 * 60 * 1000) {
////                done = true;
////            }
//            else {
//                done = false;
//            }
//        }
//        void update(float delta, int elapsed){
//            int rew = 0;
//            bool has_left = false;
//            float tmp;
//            time_ms = elapsed;
//            for (int i = 0; i < MISSION_NUM_TARGETS + MISSION_NUM_OBSTACLES; i++) {
//                if (roombas[i]->state == ROOMBA_STATE_IDLE){continue;}
//                roombas[i]->update(delta, elapsed);
//
//                for (int j = 0; j < MISSION_NUM_TARGETS + MISSION_NUM_OBSTACLES; j++) {
//                    if ( i == j || roombas[j]->state == ROOMBA_STATE_IDLE) {continue;}
//                    if (check_rmba_collision(roombas[i], roombas[j]) && check_rmba_is_facing(roombas[i], roombas[j]->pos)) {
//                        roombas[i]->collision_front = true;
//                    }
//                }
//
//                check_bounds(roombas[i], has_left, rew);
//                if (has_left){
//                    if (rew > 0) {
//                        num_good_exits += 1;
//                    }
//                    else {
//                        num_bad_exits += 1;
//                    }
//                    score += rew;
//                    roombas[i]->stop();
//                }
//            }
////            tmp = sqrt(pow(agent->xy_vel[0],2) + pow(agent->xy_vel[1],2));
////            if (tmp > DRONE_MAX_HORIZ_VELOCITY) {
////                cout << tmp << endl;
////                agent->xy_vel[0] *= (DRONE_MAX_HORIZ_VELOCITY / tmp);
////                agent->xy_vel[1] *= (DRONE_MAX_HORIZ_VELOCITY / tmp);
////            }
//
//            agent->xy_pos[0] += agent->xy_vel[0] * delta;
//            agent->xy_pos[1] += agent->xy_vel[1] * delta;
//        }
//        np::ndarray get_state() {
//            static np::ndarray state = np::zeros(p::make_tuple(43), np::dtype::get_builtin<float>());
//
//            for (int i = 0; i < MISSION_NUM_TARGETS; i++){
//
//                state[i*4 + 0] = roombas[i]->pos[0]; //p::extract<float>(roombas[i]->pos[0]);
//                state[i*4 + 1] = roombas[i]->pos[1]; //p::extract<float>(roombas[i]->pos[1]);
//                state[i*4 + 2] = roombas[i]->heading;
//                state[i*4 + 3] = (roombas[i]->state == ROOMBA_STATE_FORWARD);
//            }
//
//            state[40] = agent->xy_pos[0];
//            state[41] = agent->xy_pos[1];
//            state[42] = last_rmba;
//
//            return state;
//        }
//        np::ndarray get_screen() {
//            const int img_size = 32;
//            static np::ndarray arr = np::zeros(p::make_tuple(32, 32), np::dtype::get_builtin<int>());
//            for (int i=0; i < MISSION_NUM_TARGETS; i++){
//                if (roombas[i]->state != ROOMBA_STATE_IDLE) {
//                    arr[int(roombas[i]->pos[0] / 20.0 * (img_size - 1))][ int(roombas[i]->pos[1] / 20.0 * (img_size - 1))] = 255;
//                }
//            }
//            return arr;
//        }
//        np::ndarray get_target_pos(int rmba_index){
//            return np::from_data(roombas[rmba_index]->pos, np::dtype::get_builtin<float>(),
//                                     p::make_tuple(2),
//                                     p::make_tuple(sizeof(float)),
//                                     p::object());
//        }
//        int get_target_state(int rmba_index){
//            return roombas[rmba_index]->state;
//        }
//        float get_target_heading(int rmba_index){
//            return roombas[rmba_index]->heading;
//        }
//        float get_direction_reward() {
//            float dir_rew = 0;
//            for (int i=0; i < MISSION_NUM_TARGETS; i++){
//                if (roombas[i]->state != ROOMBA_STATE_IDLE) {
//                    dir_rew += ((float)(roombas[i]->state == ROOMBA_STATE_FORWARD))*20* NUM_RENDER_SECS *  1/ PI/3 * (fabs(PI - roombas[i]->heading) - PI/2) / (MISSION_NUM_TARGETS -(num_good_exits+num_bad_exits));
//                }
//            }
//            return dir_rew;
//        }
//        void collision(int rmba_index, bool top_or_front) {
//            if (top_or_front) {
//                roombas[rmba_index]->collision_top= true;
//            }
//            else {
//                roombas[rmba_index]->collision_front = true;
//            }
//        }
//
//        np::ndarray get_agent_pos() {
//            return np::from_data(agent->xy_pos, np::dtype::get_builtin<float>(),
//                                     p::make_tuple(2),
//                                     p::make_tuple(sizeof(float)),
//                                     p::object());
//        }
//        np::ndarray get_agent_vel() {
//            return np::from_data(agent->xy_vel, np::dtype::get_builtin<float>(),
//                                     p::make_tuple(2),
//                                     p::make_tuple(sizeof(float)),
//                                     p::object());
//        }
//        float get_agent_zpos() {
//            return agent->z_pos;
//        }
//        float get_agent_yaw() {
//            return agent->yaw;
//        }
//        void set_agent_vel(np::ndarray vel) {
//            agent->xy_vel[0] = p::extract<float>(vel[0]);
//            agent->xy_vel[1] = p::extract<float>(vel[1]);
//        }
//};
//
//BOOST_PYTHON_MODULE(test_ext)
//
//{
//
//    using namespace boost::python;
//    class_<Env>("Env")
//	.def("update", &Env::update)
//	.def("seed", &Env::seed)
//	.def("reset", &Env::reset)
//	.def_readonly("num_good_exits", &Env::num_good_exits)
//	.def_readonly("num_bad_exits", &Env::num_bad_exits)
//	.def_readonly("score", &Env::score)
//	.def_readonly("time_ms", &Env::time_ms)
//	.def("get_agent_pos", &Env::get_agent_pos)
//	.def("get_agent_zpos", &Env::get_agent_zpos)
//	.def("get_agent_yaw", &Env::get_agent_yaw)
//	.def("set_agent_vel", &Env::set_agent_vel)
//	.def("get_agent_vel", &Env::get_agent_vel)
//	.def("get_state", &Env::get_state)
//	.def("get_screen", &Env::get_screen)
//	.def("get_target_pos", &Env::get_target_pos)
//	.def("get_target_state", &Env::get_target_state)
//	.def("get_target_heading", &Env::get_target_heading)
//	.def("get_direction_reward", &Env::get_direction_reward)
//	.def("collision", &Env::collision)
//	.def("masterUpdate", &Env::masterUpdate)
//	.def("set_earlyTerminationTime_ms", &Env::set_earlyTerminationTime_ms)
//	.def_readonly("done", &Env::done)
//	.def_readonly("total_rew", &Env::total_rew)
//	.def("get_rews_dict", &Env::get_rews_dict)
////	.def("set_last_rmba", &Env::set_last_rmba)
////	.def_readwrite("roombas", &Env::roombas)
//	;
//
//
//}
//
