import math
import gym
from gym.utils import seeding
from gym import spaces
import numpy as np
from gym.envs.IARC.roombasim import config as cfg #TODO make same as cpp stuff
import pyglet
from gym.envs.IARC.IARC_Game_Board_Master_v2 import IARCEnv_Master_v2


class IARCEnv_5(gym.Env, IARCEnv_Master_v2):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 30
    }

    def __init__(self):
        self.last_rmba=4
        self.action_space = spaces.MultiDiscrete([cfg.MISSION_NUM_TARGETS, 2, 2])

        min_obs_template = list([0, 0, 0, False])
        max_obs_template = list([20, 20, math.pi * 2, True])
        min_obs = list()
        max_obs = list()
        for i in range(0, cfg.MISSION_NUM_TARGETS):  # (self.environment.roombas, start=1):
            min_obs = min_obs + min_obs_template
            max_obs = max_obs + max_obs_template

        min_obs = min_obs + list([0, 0, 0])
        max_obs = max_obs + list([20, 20, 9])

        self.observation_space = spaces.Box(np.asarray(min_obs), np.asarray(max_obs), dtype=np.float32)

        self.init_Master()
        self.reset()


    def _get_state(self):
        return self.environment.get_state()/self.observation_space.high

    def reset(self):
        self.reset_Master()
        self.state = self._get_state()
        # self.last_good_exits = 0
        # self.last_bad_exits = 0
        return self.state


    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        np.random.seed(seed)
        rand = (np.random.sample((cfg.MISSION_NUM_TARGETS,))*1000).astype(np.int32)
        self.environment.seed(rand)
        return [seed]

    def step(self, action):
        # rews = {'game_reward': 0.0, 'speed_reward': 0.0, 'direction_reward': 0.0, 'target_reward': 0.0, 'num_good_exits':0, 'num_bad_exits':0}


        # assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))
        if action.ndim >= 2:
            action = action[0]
            ac = {"rmba_sel": action[0], "ac_bool": action[1], "top_or_front": action[2]}
        else:
            ac = {"rmba_sel": action[0], "ac_bool": action[1], "top_or_front": action[2]}

        if self.render_bool:
            aav_targPos = self.environment.get_target_pos(ac["rmba_sel"])  # .environment.roombas[ac["rmba_sel"]].pos

            rews = {'game_reward': 0.0, 'speed_reward': 0.0, 'direction_reward': 0.0, 'target_reward': 0.0, 'num_good_exits':0, 'num_bad_exits':0}

            def rmbaInteract():
                if ac["ac_bool"]:
                    self.environment.collision(ac["rmba_sel"], bool(ac["top_or_front"]))

            rews["game_reward"], rews["speed_reward"], done = self._updateEnv(aav_targPos, rmbaInteract)
        else:
            self.environment.masterUpdate(ac['rmba_sel'], bool(ac['top_or_front']), bool(ac['ac_bool']))
            done = self.environment.done

        rews = self.environment.get_rews_dict();

        self.state = self._get_state()
        info = {"time_ms": self.time_elapsed_ms}

        info["img"] = np.zeros([32, 32, 1], dtype=np.float32)
        rews["num_bad_exits"] *= -1
        info["rews"] = rews
        # if done:
        #     self.environment.reset()
        #     info["img"] = np.self.get_screen()
        # else:
        #     info["img"] = self.get_screen()

        return self.state, self.environment.total_rew, done, info

    def get_screen(self):
        return self.environment.get_screen()
        # img_size = 32  # TODO remove hardcode
        # arr = np.zeros([img_size, img_size, 1], dtype=np.uint8)
        # for rmba in self.environment.roombas:
        #     if rmba.type and rmba.state is not cfg.ROOMBA_STATE_IDLE:
        #         arr[int(rmba.pos[0] / 20.0 * (img_size - 1)), int(rmba.pos[1] / 20.0 * (img_size - 1)), :] = 255
        # return arr


    def _render(self, mode='human', close=False):
        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        if self.viewer is None:
            from gym.envs.IARC.roombasim_cpp.display import Display
            # from gym.envs.classic_control import rendering
            self.viewer = Display(self.environment, timescale=1.0, self_update=False)

            pyglet.app.event_loop.start()

        # timeout = pyglet.app.event_loop.idle()
        # self.viewer.update(0.1)
        # self.viewer.on_draw()
        # self.viewer.

        # dt = self.clock.update_time()
        # redraw_all = self.clock.call_scheduled_functions(dt)

        # Redraw all windows
        self.viewer.update_time_only(0.1)
        for window in pyglet.app.windows:
            window.switch_to()
            window.dispatch_event('on_draw')
            window.flip()
            window._legacy_invalid = False
        pyglet.app.platform_event_loop.step(0.1)