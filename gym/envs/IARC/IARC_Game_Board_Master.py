from gym.envs.IARC.roombasim import environment
import math
import numpy as np
from gym.envs.IARC.roombasim import config as cfg

class IARCEnv_Master(object):
    def init_Master(self):
        self.numRenderSecs = 2
        self.viewer = None

        self.earlyTerminationTime_ms = None
        self.render_bool = False

        import gym.envs.IARC.roombasim.pittras.config
        cfg.load(gym.envs.IARC.roombasim.pittras.config)

        self.environment = environment.Environment()
        self.environment.reset()

        # setup agent
        agent = cfg.AGENT([13, 10], 0)
        self.environment.agent = agent

    def reset_Master(self):
        self.time_elapsed_ms = 0
        self.prev_score = 0
        self.environment.reset()

        # setup agent
        agent = cfg.AGENT([13, 10], 0)

        self.environment.agent = agent
        self.environment.agent.z_pos = 2

    def getDirectionRew(self, rmba):
        return (rmba.state == cfg.ROOMBA_STATE_FORWARD) * (
                1 / math.pi / 3 * (math.fabs(math.pi - rmba.heading) - math.pi / 2)) / (
                                    cfg.MISSION_NUM_TARGETS - (
                                    self.environment.bad_exits + self.environment.good_exits))

    def _updateEnv(self, aav_targPos, actionFn):
        # break_early = False
        speed_rew=0
        for i in range(self.numRenderSecs *10):
            dist2targ = np.linalg.norm(self.environment.agent.xy_pos - aav_targPos)
            if  dist2targ <= 0.35:
                self.environment.agent.xy_vel = np.array([0.0, 0.0])
                actionFn()
            else:
                ang = math.atan2(aav_targPos[1] - self.environment.agent.xy_pos[1], aav_targPos[0] - self.environment.agent.xy_pos[0])
                self.environment.agent.xy_vel = np.array([cfg.DRONE_MAX_HORIZ_VELOCITY*np.cos(ang), cfg.DRONE_MAX_HORIZ_VELOCITY*np.sin(ang)])

            self.time_elapsed_ms += 100
            self.environment.update(0.1, self.time_elapsed_ms)
            if self.render_bool:
                self._render()
            # if break_early:
            #     continue
        game_rew = self.environment.score/100 - self.prev_score
        speed_rew += game_rew * 0.3 * (10 * 60 * 1000 - self.environment.time_ms) / 1000 / 60 / 10
        self.prev_score = self.environment.score/100

        if (self.environment.bad_exits + self.environment.good_exits) >= cfg.MISSION_NUM_TARGETS:
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