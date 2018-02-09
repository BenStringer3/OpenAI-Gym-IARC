# -*- coding: utf-8 -*-
"""
Created on Fri Jan 12 12:49:46 2018

@author: User
"""
import math
import random
import gym 
from gym.utils import seeding
from gym import spaces
import numpy as np

NUM_OF_ACTIONS = 5

class Ground_Robot:
    def __init__(self, pos, vel):
        self.pos = pos
        self.vel = vel

class IARCEnv_0(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 30
    }
    
    def __init__(self):
        self.viewer = None

        self.action_space = spaces.Discrete(8)
        self.SPEED = 0.075
        self.screen_width = 400
        self.screen_height = 400
        self.grid_width = 20
        self.grid_height = 20
        self.scale = self.screen_width/self.grid_width, self.screen_height/self.grid_height
        low = np.array([-self.grid_width/2, -self.grid_width/2, -self.grid_width/2, -self.grid_width/2])
        high = np.array([self.grid_width/2, self.grid_width/2, self.grid_width/2, self.grid_width/2])
#        low = np.array([-self.grid_width/2, -self.grid_width/2])
#        high = np.array([self.grid_width/2, self.grid_width/2])
        self.observation_space = spaces.Box(low, high)

        self._seed()
        self.reset()

    def _reset(self):
        self.state = np.array([0, 0, 5, 5])
#        self.state = np.array([0, 0, random.choice([-5, 5]), random.choice([-5, 5])])
#        self.state = np.array([0, 0, self.np_random.uniform(low= -self.grid_width/2, high= self.grid_width/2), self.np_random.uniform(low= -self.grid_width/2, high= self.grid_width/2)])
#        self.initial_distance = self.distance([self.state[0], self.state[1]], [self.state[2], self.state[3]])
        self.initial_distance = math.sqrt(50)
        return np.array(self.state)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    def grid2Pixels(self, gridVect):
        x = gridVect[0]
        y = gridVect[1]
        pixelVect = np.zeros((2,1))
        pixelVect[0] = (x + self.grid_width/2)*self.scale[0]
        pixelVect[1] = (y + self.grid_height/2)*self.scale[1]
        return pixelVect
    
    def distance(self, obj1, obj2):
        return math.sqrt(math.pow(obj1[0]-obj2[0], 2) + math.pow(obj1[1]-obj2[1], 2))

    def _step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        reward = -1
        done = False

        x,y, targ_x, targ_y = self.state
#        if (action == 0):
#            dx, dy = 0, 0
#        elif (action == 1):
        if (action == 0):
            dx, dy = 0, 1
        elif (action == 1):
            dx, dy = 1, 0
        elif (action == 2):
            dx, dy = -1, 0
        elif (action == 3):
            dx, dy = 0, -1
        elif (action == 4):
            dx, dy = math.sqrt(2)/2, math.sqrt(2)/2
        elif (action == 5):
            dx, dy = -math.sqrt(2)/2, math.sqrt(2)/2
        elif (action == 7):
            dx, dy = math.sqrt(2)/2, -math.sqrt(2)/2
        elif (action == 6):
            dx, dy = -math.sqrt(2)/2, -math.sqrt(2)/2
        else:
            print ("action not in action space")

        
        #enforce grid boundaries
        if (x >= self.grid_width/2 and dx > 0): #if on right wall and moving right
            dx = 0 
            #reward -= 200
            print("right wall collision")
        elif (x <= -self.grid_width/2 and dx < 0):  #if on left wall and moving left
            dx = 0
            #reward -= 200
            print("left wall collision")
        elif (y <= -self.grid_height/2 and dy < 0):  #if on bottom wall and moving down
            dy = 0
            #reward -= 200
            print ("bottom wall collision")
        elif (y >= self.grid_height/2 and dy > 0):  #if on top wall and moving up
            dy = 0
            #reward -= 200
            print ("top wall collision")
            
        dx *= self.SPEED
        dy *= self.SPEED
        x += dx
        y += dy

        self.state = x,y,targ_x, targ_y
        
        distance = self.distance([x, y], [targ_x, targ_y])
        #print(distance)
#        distance = self.distance([x, y], [5, 5])
        
        if (distance < 1):
            done = True
            reward += 200
        
        #reward -= 1#distance/self.initial_distance
            
        return np.array(self.state), reward, done, {}
        

    def _height(self, xs):
        return np.sin(3 * xs)*.45+.55

    def _render(self, mode='human', close=False):
        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(self.screen_width, self.screen_height)
            
#            dot = rendering.
#            xs = np.linspace(self.min_position, self.max_position, 100)
#            ys = self._height(xs)
#            xys = list(zip((xs-self.min_position)*scale, ys*scale))
#
#            self.track = rendering.make_polyline(xys)
#            self.track.set_linewidth(4)
#            self.viewer.add_geom(self.track)

            clearance = 10
            
            drone = rendering.make_circle(1.25/2*self.scale[0])
            drone.add_attr(rendering.Transform(translation=(0, clearance)))
            self.dronetrans = rendering.Transform()
            drone.add_attr(self.dronetrans)
            self.viewer.add_geom(drone)
            
            target = rendering.make_circle(1.25/2*self.scale[0])
            target.add_attr(rendering.Transform(translation=(0, clearance)))
            self.targettrans = rendering.Transform()
            target.add_attr(self.targettrans)
            self.viewer.add_geom(target)
            



#            l,r,t,b = -carwidth/2, carwidth/2, carheight, 0
#            car = rendering.FilledPolygon([(l,b), (l,t), (r,t), (r,b)])
#            car.add_attr(rendering.Transform(translation=(0, clearance)))
#            self.cartrans = rendering.Transform()
#            car.add_attr(self.cartrans)
#            self.viewer.add_geom(car)
#            frontwheel = rendering.make_circle(carheight/2.5)
#            frontwheel.set_color(.5, .5, .5)
#            frontwheel.add_attr(rendering.Transform(translation=(carwidth/4,clearance)))
#            frontwheel.add_attr(self.cartrans)
#            self.viewer.add_geom(frontwheel)
#            backwheel = rendering.make_circle(carheight/2.5)
#            backwheel.add_attr(rendering.Transform(translation=(-carwidth/4,clearance)))
#            backwheel.add_attr(self.cartrans)
#            backwheel.set_color(.5, .5, .5)
#            self.viewer.add_geom(backwheel)
#            flagx = (self.goal_position-self.min_position)*scale
#            flagy1 = self._height(self.goal_position)*scale
#            flagy2 = flagy1 + 50
#            flagpole = rendering.Line((flagx, flagy1), (flagx, flagy2))
#            self.viewer.add_geom(flagpole)
#            flag = rendering.FilledPolygon([(flagx, flagy2), (flagx, flagy2-10), (flagx+25, flagy2-5)])
#            flag.set_color(.8,.8,0)
#            self.viewer.add_geom(flag)


        x,y = self.grid2Pixels([self.state[0], self.state[1]])
        self.dronetrans.set_translation(x,y)#(pos-self.min_position)*scale, self._height(pos)*scale)
        self.dronetrans.set_rotation(0)#math.cos(3 * pos))
        
        x, y = self.grid2Pixels([self.state[2], self.state[3]])
#        x, y = self.grid2Pixels([5, 5])
        self.targettrans.set_translation(x,y)

        
        return self.viewer.render(return_rgb_array = mode=='rgb_array')

            