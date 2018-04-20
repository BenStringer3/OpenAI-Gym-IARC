#include "Roomba.h"
#include <random>
class TargetRoomba : public Roomba {
    public:
        void update(float delta, int elapsed);
        TargetRoomba(float pos_x, float pos_y, float heading);
        ~TargetRoomba();
        TargetRoomba();
        void seed(int seed);
    private:
        std::random_device                  rand_dev;
        std::mt19937                        generator;//(rand_dev());
        std::uniform_real_distribution<float>  distr;//(-ROOMBA_HEADING_NOISE_MAX, ROOMBA_HEADING_NOISE_MAX);

};