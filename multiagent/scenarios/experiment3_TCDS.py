import numpy as np
import socket
from npsocket_sn import SocketNumpyArray
from multiagent.core import World, Agent, Landmark
from multiagent.scenario import BaseScenario
from math import *

class Scenario(BaseScenario):
    '''
    Experiment 3
    - 4 robots navigation + obstacle avoidance
    - Dynamic & Static obstacle avoidance
    '''

    def __init__(self):
        self.sock_sender = SocketNumpyArray()
        self.sock_sender.initialize_sender('localhost', 9998)
        self.n = None
        self.x = None
        self.y = None
        self.theta = None 
        self.phero = None

        # Object initialisation
        # Robot initial positions
        self.robot_positions = [[-2.5,0],[2.5,0],[0,-2.5],[0,2.5]]
        self.robot_angles = [0, pi, pi/2, -pi/2]
        self.target_positions = [[2.5,0],[-2.5,0],[0,2.5],[0,-2.5]]
        ## 20201209 - add random positions if the fixed training works.
        # self.target_index = 0
        # self.num_experiments = 20
        # self.radius = 4

    def make_world(self):
        world = World()
        # set any world properties first
        world.dim_c = 2
        num_agents = 4
        num_obstacle = 1
        num_target = 4
        world.collaborative = False
        # add agents
        world.agents = [Agent() for i in range(num_agents)]
        for i, agent in enumerate(world.agents):
            agent.name = 'agent %d' % i
            agent.collide = True
            agent.silent = True
            agent.size = 0.1

        # add obstacles
        obstacles = [Landmark() for i in range(num_obstacle)]
        for i, obstacle in enumerate(obstacles):
            obstacle.name = 'obstacle %d' % i
            obstacle.collide = True
            obstacle.movable = False
            obstacle.size = 0.1

        # add target
        targets = [Landmark() for i in range(num_target)]
        for i, target in enumerate(targets):
            target.name = 'target %d' % i 
            target.collide = False
            target.movable = False
            target.size = 0.1

        # Merge the landmarks (obstacles + target)
        world.landmarks = obstacles + targets

        # make initial conditions
        self.n = num_agents
        self.x = [0.0, 0.0]*num_agents
        self.y = [0.0, 0.0]*num_agents
        self.theta = [0.0, 0.0]*num_agents
        self.reset_world(world)
        
        return world

    def reset_world(self, world):
        # random properties for agent
        for i, agent in enumerate(world.agents):
            agent.color = np.array([0.35, 0.35, 0.85])
        # random properties for landmarks
        for i, landmark in enumerate(world.landmarks):
            if 'obstacle' in landmark.name:
                landmark.color = np.array([0.25, 0.25, 0.25])
            if 'target' in landmark.name:
                landmark.color = np.array([0.9, 0.1, 0.1])
        
        # ========================================================================= #
	    #                            TARGET UPDATE                                  #
	    # ========================================================================= #
        # set random initial states
        # robot position (0, 0), distance between robots and target (4 m)
        # angle_target = self.target_index*2*pi/self.num_experiments        

        # self.target_x = self.radius*cos(angle_target)
        # self.target_y = self.radius*sin(angle_target)
        
        # if self.target_index < self.num_experiments-1:
        #     self.target_index += 1
        # else:
        #     self.target_index = 0

        # ========================================================================= #
	    #                             OBJECT RESET                                  #
	    # ========================================================================= #

        # Agent update 
        # 20201209 Fixed initial positions (later, I'll apply random initial positions)
        for i, agent in enumerate(world.agents):
            agent.state.p_pose = np.asarray(self.robot_positions[i] + self.robot_angles[i]) 
            agent.state.p_pos  = self.robot_positions[i]
            # self.target = [[self.x[1], self.y[1]], [self.x[0], self.y[0]]] 

            agent.state.p_vel = np.zeros(world.dim_p)
            agent.state.c = np.zeros(world.dim_c)
            agent.state.target = self.target_positions[i] # 20201201 target
            
        # for i, landmark in enumerate(world.landmarks):
        #     if landmark.name[0] == 'o':
        #         landmark.state.p_pos = [0, 0] #np.random.uniform(-1, +1, world.dim_p)
        #         landmark.state.p_vel = np.zeros(world.dim_p)
        
        # Obstacle update
        for i, obstacle in enumerate([ob for ob in world.landmarks if 'obstacle' in ob]):
            obstacle.state.p_pos = np.asarray([0,0])
            obstacle.state.p_vel = np.zeros(world.dim_p)
            
        # Target update
        for i, target in enumerate([tg for tg in world.landmarks if 'target' in tg]):
            target.state.p_pos = np.asarray(self.target_positions[i])
            target.state.p_vel = np.zeros(world.dim_p)

    def benchmark_data(self, agent, world):
        rew = 0
        collisions = 0
        occupied_landmarks = 0
        min_dists = 0
        for l in world.landmarks:
            dists = [np.sqrt(np.sum(np.square(a.state.p_pos - l.state.p_pos))) for a in world.agents]
            min_dists += min(dists)
            rew -= min(dists)
            if min(dists) < 0.1:
                occupied_landmarks += 1
        if agent.collide: 
            for a in world.agents:
                if self.is_collision(a, agent):
                    rew -= 1
                    collisions += 1
        return (rew, collisions, min_dists, occupied_landmarks)


    def is_collision(self, agent1, agent2):
        delta_pos = agent1.state.p_pos - agent2.state.p_pos
        dist = np.sqrt(np.sum(np.square(delta_pos)))
        dist_min = agent1.size + agent2.size
        return True if dist < dist_min else False

    def reward(self, agent, world):
        '''
        Compute rewards
        '''
        distance_reward = 0.0
        phero_reward = 0.0
        goal_reward = 0.0
        collision_reward = 0.0
        #angular_punish_rewards = [0.0]*self.num_robots
        #linear_punish_rewards = [0.0]*self.num_robots

        # 1. Distance Reward
        goal_progress = agent.state.distance_to_goal_prev - agent.state.distance_to_goal
        if abs(goal_progress) < 0.1:
            print("Goal Progress: {}".format(goal_progress))
            if goal_progress >= 0:
                    distance_reward = goal_progress * 1.2
            else:
                    distance_reward = goal_progress
        else:
            distance_reward = 0.0

        # 2. Phero Reward
        # phero_sum = np.sum(self.phero)
        # phero_reward = -phero_sum*2

        # 3. Goal Reward
        if agent.state.distance_to_goal <= 0.3:
            goal_reward = 50.0
            #done = True
            #self.reset(model_state, id_bots=idx[i])

        # 4. Collision Penalty
        
        for i, obstacle in enumerate([ob for ob in world.landmarks if 'obstacle' in ob]):
            is_collision = self.is_collision(agent, obstacle)
            if is_collision == True:
                collision_reward = -50.0
        
        reward = distance_reward*(5/world.dt)+phero_reward+goal_reward+collision_reward
        print("distance reward: {}".format(distance_reward))
        print("collision_reward: {}".format(collision_reward))
        print("goal_reward: {}".format(goal_reward))

        return reward

    def observation(self, agent, world):
        
        id = int(agent.name[-1])
        # get positions of all entities in this agent's reference frame
        entity_pos = []
        for entity in world.landmarks:  # world.entities:
            entity_pos.append(entity.state.p_pos - agent.state.p_pos)

        # entity colors
        entity_color = []
        for entity in world.landmarks:  # world.entities:
            entity_color.append(entity.color)

        positions = np.asarray(agent.state.p_pos)
        self.sock_sender.send_numpy_array(positions)
        data = self.sock_sender.receive_from_server()
        self.phero = phero = np.asarray(data).reshape(1,9)
        
        # 20201209 needs to work on merging static & dynamic pheromone values
        
        obs = np.hstack((phero, [agent.action.twist], np.asarray([agent.state.distance_to_goal]).reshape(1,1), np.asarray([agent.state.angle_diff]).reshape(1,1)))
        world.obs_n = np.shape(obs)[1]
        return obs 

    def done(self, agent, world):
        agent.state.done = False
        
        # 1. Goal arrival
        if agent.state.distance_to_goal <= 0.3:
            agent.state.done = True

        # 2. Out of range
        if abs(agent.state.p_pos[0]) > 4.6 or abs(agent.state.p_pos[1]) > 4.6:
            agent.state.done = True
            print("out of range!!!!")

        # 3. collision
        for i, obstacle in enumerate([ob for ob in world.landmarks if 'obstacle' in ob]):
            is_collision = self.is_collision(agent, obstacle)
            if is_collision == True:
                agent.state.done = True

        done = agent.state.done

        return done
    
    def info(self, agent, world):
        info = [{"targets": agent.state.target}]
        return info