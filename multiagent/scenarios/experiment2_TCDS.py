import numpy as np
import socket
from npsocket_sn import SocketNumpyArray
from multiagent.core import World, Agent, Landmark
from multiagent.scenario import BaseScenario
from math import *

class Scenario(BaseScenario):

    def __init__(self):
        self.sock_sender = SocketNumpyArray()
        self.sock_sender.initialize_sender('localhost', 9999)
        self.n = None
        self.x = None
        self.y = None
        self.theta = None 
        self.phero = None

    def make_world(self):
        world = World()
        # set any world properties first
        world.dim_c = 2
        num_agents = 2
        num_landmarks = 1
        world.collaborative = True
        # add agents
        world.agents = [Agent() for i in range(num_agents)]
        for i, agent in enumerate(world.agents):
            agent.name = 'agent %d' % i
            agent.collide = True
            agent.silent = True
            agent.size = 0.1
        # add landmarks
        world.landmarks = [Landmark() for i in range(num_landmarks)]
        for i, landmark in enumerate(world.landmarks):
            landmark.name = 'landmark %d' % i
            landmark.collide = True
            landmark.movable = False
            landmark.size = 0.06

        # make initial conditions
        self.n = num_agents
        self.x = [0.0, 0.0]*num_agents
        self.y = [0.0, 0.0]*num_agents
        self.theta = [0.0, 0.0]*num_agents
        self.reset_world(world)
        
        return world

    def reset_world(self, world):
        # random properties for agents
        for i, agent in enumerate(world.agents):
            agent.color = np.array([0.35, 0.35, 0.85])
        # random properties for landmarks
        for i, landmark in enumerate(world.landmarks):
            landmark.color = np.array([0.25, 0.25, 0.25])
        # set random initial states
        d = 3
        angle = pi*np.random.uniform(-1, +1)        


        self.x[0] = (d/2)*cos(angle)
        self.y[0] = (d/2)*sin(angle)

        self.x[1] = (d/2)*cos(angle+pi)
        self.y[1] = (d/2)*sin(angle+pi)

        self.theta[0] = angle + pi
        self.theta[1] = angle 

        world.agents[0].state.p_pose = np.asarray([self.x[0], self.y[0], self.theta[0]])
        world.agents[1].state.p_pose = np.asarray([self.x[1], self.y[1], self.theta[1]])
        world.agents[0].state.p_pos  = world.agents[0].state.p_pose[:2]
        world.agents[1].state.p_pos  = world.agents[1].state.p_pose[:2]
        self.target = [[self.x[1], self.y[1]], [self.x[0], self.y[0]]] #20201130 Target 
        for i, agent in enumerate(world.agents):
            #agent.state.p_pos = np.random.uniform(-1, +1, world.dim_p)
            agent.state.p_vel = np.zeros(world.dim_p)
            agent.state.c = np.zeros(world.dim_c)
            agent.state.target = self.target[i] # 20201201 target
            
        for i, landmark in enumerate(world.landmarks):
            landmark.state.p_pos = [0, 0] #np.random.uniform(-1, +1, world.dim_p)
            landmark.state.p_vel = np.zeros(world.dim_p)

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
        # Agents are rewarded based on minimum agent distance to each landmark, penalized for collisions
        rew = 0
        for l in world.landmarks:
            dists = [np.sqrt(np.sum(np.square(a.state.p_pos - l.state.p_pos))) for a in world.agents]
            rew -= min(dists)
        if agent.collide:
            for a in world.agents:
                if self.is_collision(a, agent):
                    rew -= 1
        distance_reward = 0.0
        phero_reward = 0.0
        goal_reward = 0.0
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
        phero_sum = np.sum(self.phero)
        phero_reward = -phero_sum*2

        # 3. Goal Reward
        if agent.state.distance_to_goal <= 0.3:
            goal_reward = 50.0
            #done = True
            #self.reset(model_state, id_bots=idx[i])
        
        reward = distance_reward*(5/world.dt)+phero_reward+goal_reward

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
        # communication of all other agents
        comm = []
        other_pos = []
        for other in world.agents:
            if other is agent: continue
            comm.append(other.state.c)
            other_pos.append(other.state.p_pos - agent.state.p_pos)
        positions = np.asarray([agent.state.p_pos for agent in world.agents])
        #print(positions)
        self.sock_sender.send_numpy_array(positions)
        data = self.sock_sender.receive_from_server()
        #print("Data received from receiver: {}, size: {}".format(data, data.shape))
        #print("twist: {}".format(agent.action.twist))
        self.phero = phero = data[id].reshape(1,9)
        #print("distances: {}, {}".format(agent.distance_to_goal_prev, agent.distance_to_goal))
        # Distance to Goal (state)
        #self.distance_to_goal = 
        # Difference of local angle and global angle to head to the goal
        #self.angle_diff
        #print("shape 0: {}, shape 1: {}".format(phero.shape, agent.state.p_vel))
        obs = np.hstack((phero, [agent.action.twist], np.asarray([agent.state.distance_to_goal]).reshape(1,1), np.asarray([agent.state.angle_diff]).reshape(1,1)))
        world.obs_n = np.shape(obs)[1]
        print("obs size: {}".format(np.shape(obs)))
        return obs # 20201201 observation is changed (pheromone + velocity, distance, anglediff ) 1*13

    def done(self, agent, world):
        agent.state.done = False
        
        # 1. Goal arrival
        if agent.state.distance_to_goal <= 0.3:
            agent.state.done = True

        # 2. Out of range
        print("pos[0]: {}, pos[1]: {}".format(agent.state.p_pos[0], agent.state.p_pos[1]))
        print("abspos[0]: {}, abspos[1]: {}".format(abs(agent.state.p_pos[0]), abs(agent.state.p_pos[1])))
        if abs(agent.state.p_pos[0]) > 4.8 or abs(agent.state.p_pos[1]) > 4.8:
            agent.state.done = True
            print("out of range!!!!")

        # 3. collision
        agents = [agent for agent in world.agents]
        is_collision = self.is_collision(agents[0], agents[1])
        if is_collision == True:
            agent.state.done = True

        done = agent.state.done

        return done
    
    def info(self, agent, world):
        info = [{"targets": agent.state.target}]
        return info