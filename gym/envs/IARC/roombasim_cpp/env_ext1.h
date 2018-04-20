#include "/usr/local/include/boost_1_66_0/boost/python.hpp"
#include "/usr/local/include/boost_1_66_0/boost/python/numpy.hpp"
#include "TargetRoomba.h"
#include "Agent.h"
#include "ObstacleRoomba.h"
#include <string>
#include <map>





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


using namespace std;
namespace np = boost::python::numpy;
namespace p = boost::python;

class Env{
   private:

        bool check_rmba_collision(const Roomba * rmba1, const Roomba * rmba2);
        bool check_rmba_is_facing(const Roomba * rmba, const float * pos);
        void check_bounds(const Roomba * rmba, bool & has_left, int & rew);
        Agent * agent;
        int last_rmba;
        int earlyTerminationTime_ms;
        struct Rewards {
            float game_reward;
            float last_game_score;
            float speed_reward;
            float direction_reward;
            float target_reward;
            float num_good_exits;
            float last_num_good_exits;
            float num_bad_exits;
            float last_num_bad_exits;
            void reset() {
                game_reward = 0;
                speed_reward = 0;
                direction_reward = 0;
                target_reward = 0;
                num_good_exits = 0;
                num_bad_exits = 0;
            }
            Rewards()
            : game_reward(0),
            last_game_score(0),
            speed_reward(0),
            direction_reward(0),
            target_reward(0),
            num_good_exits(0),
            last_num_good_exits(0),
            num_bad_exits(0),
            last_num_bad_exits(0)
            { }
        }rewards;
    public:

        Env();
        void reset();
        void set_earlyTerminationTime_ms(int earlyTerminationTime_ms) ;
        void seed(np::ndarray seeds);
//        map<string, float> rews_map;
//        p::dict rews;
        float total_rew;


        Roomba * roombas[MISSION_NUM_OBSTACLES + MISSION_NUM_TARGETS];
        int num_good_exits;
        int num_bad_exits;
        int score;
        int time_ms;
        bool done;
        p::dict get_rews_dict();
        void masterUpdate(int rmba_index, bool top_or_front, bool engageTarget);
        void update(float delta, int elapsed);
        np::ndarray get_state();
        np::ndarray get_screen();
        np::ndarray get_target_pos(int rmba_index);
        int get_target_state(int rmba_index);
        float get_target_heading(int rmba_index);
        float get_direction_reward();
        void collision(int rmba_index, bool top_or_front);
        np::ndarray get_agent_pos();
        np::ndarray get_agent_vel();
        float get_agent_zpos();
        float get_agent_yaw();
        void set_agent_vel(np::ndarray vel);
};



