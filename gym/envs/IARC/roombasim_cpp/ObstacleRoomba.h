#include "Roomba.h"

class ObstacleRoomba : public Roomba {
    public:
        void update(float delta, int elapsed);
        void seed(int seed);
        ObstacleRoomba(float pos_x, float pos_y, float heading);
        ObstacleRoomba();
        ~ObstacleRoomba();
};