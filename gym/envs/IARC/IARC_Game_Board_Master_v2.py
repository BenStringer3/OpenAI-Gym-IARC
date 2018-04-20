from gym.envs.IARC.roombasim.environment.environment import Environment
import math
import numpy as np
from gym.envs.IARC.roombasim import config as cfg
from gym.envs.IARC.roombasim_cpp import env_ext1

class IARCEnv_Master_v2(object):
    def init_Master(self):
        self.numRenderSecs = 2
        self.viewer = None

        self.earlyTerminationTime_ms = 10*60*1000
        self.render_bool = False

        # import gym.envs.IARC.roombasim.pittras.config
        # cfg.load(gym.envs.IARC.roombasim.pittras.config)

        self.environment = env_ext1.Env() #Environment()
        self.environment.reset()

    def reset_Master(self):
        self.time_elapsed_ms = 0
        # self.prev_score = 0
        self.environment.set_earlyTerminationTime_ms(int(self.earlyTerminationTime_ms))
        self.environment.reset()


    def getDirectionRew(self, rmba):
        return (rmba.state == cfg.ROOMBA_STATE_FORWARD) * (
                1 / math.pi / 3 * (math.fabs(math.pi - rmba.heading) - math.pi / 2)) / (
                                    cfg.MISSION_NUM_TARGETS - (
                                    self.environment.bad_exits + self.environment.good_exits))

    def _updateEnv(self, aav_targPos, actionFn):
        # break_early = False
        drone_speed = 3
        speed_rew=0
        for i in range(self.numRenderSecs *10):
            agent_pos = self.environment.get_agent_pos()
            dist2targ = np.linalg.norm(agent_pos - aav_targPos)
            if  dist2targ <= 0.35:
                self.environment.set_agent_vel(np.array([0.0, 0.0]))
                actionFn()
            else:
                ang = math.atan2(aav_targPos[1] - agent_pos[1], aav_targPos[0] - agent_pos[0])
                self.environment.set_agent_vel(np.array([drone_speed*np.cos(ang), drone_speed*np.sin(ang)]))

            self.time_elapsed_ms += 100
            self.environment.update(0.1, self.time_elapsed_ms)
            if self.render_bool:
                self._render()
            # if break_early:
            #     continue
        game_rew = self.environment.score/100 - self.prev_score
        speed_rew += game_rew * 0.3 * (10 * 60 * 1000 - self.environment.time_ms) / 1000 / 60 / 10
        self.prev_score = self.environment.score/100

        if (self.environment.num_bad_exits + self.environment.num_good_exits) >= cfg.MISSION_NUM_TARGETS:
            done = True
            # end_rew =  11 * (
            #             10 * 60 * 1000 - self.environment.time_ms) / 1000 / 60 / 10 * self.environment.good_exits / cfg.MISSION_NUM_TARGETS
        elif self.earlyTerminationTime_ms is not None and self.earlyTerminationTime_ms <= self.environment.time_ms:
            done = True
        elif self.environment.time_ms >= 10 * 60 * 1000:
            done = True
        else:
            done = False

        return game_rew, speed_rew, done