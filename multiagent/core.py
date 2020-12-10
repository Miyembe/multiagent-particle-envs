import numpy as np
<<<<<<< HEAD
from math import *

# Miscellanous functions

=======
>>>>>>> 69ee7f85811c77ee651722bc3c332677b2195da1

# physical/external base state of all entites
class EntityState(object):
    def __init__(self):
        # physical position
        self.p_pos = None
        # physical velocity
        self.p_vel = None
<<<<<<< HEAD
        # robot pose (x, y, theta)
        self.p_pose = None
=======
>>>>>>> 69ee7f85811c77ee651722bc3c332677b2195da1

# state of agents (including communication and internal/mental state)
class AgentState(EntityState):
    def __init__(self):
        super(AgentState, self).__init__()
        # communication utterance
        self.c = None
<<<<<<< HEAD
        self.target = None
        # 20201201 distances
        self.distance_to_goal_prev = 0.0
        self.distance_to_goal = 0.0
        # 20201201 angle_diff
        self.global_angle = 0.0
        self.angle_diff = 0.0
        # 20201201 done
        self.done = False
=======
>>>>>>> 69ee7f85811c77ee651722bc3c332677b2195da1

# action of the agent
class Action(object):
    def __init__(self):
        # physical action
        self.u = None
        # communication action
        self.c = None
<<<<<<< HEAD
        # Twist (linear.x, angular.z) - wheeled mobile robot style
        self.twist = [0, 0]

=======
>>>>>>> 69ee7f85811c77ee651722bc3c332677b2195da1

# properties and state of physical world entity
class Entity(object):
    def __init__(self):
        # name 
        self.name = ''
        # properties:
        self.size = 0.050
        # entity can move / be pushed
        self.movable = False
        # entity collides with others
        self.collide = True
        # material density (affects mass)
        self.density = 25.0
        # color
        self.color = None
        # max speed and accel
        self.max_speed = None
        self.accel = None
        # state
        self.state = EntityState()
        # mass
        self.initial_mass = 1.0

    @property
    def mass(self):
        return self.initial_mass

# properties of landmark entities
class Landmark(Entity):
     def __init__(self):
        super(Landmark, self).__init__()

# properties of agent entities
class Agent(Entity):
    def __init__(self):
        super(Agent, self).__init__()
        # agents are movable by default
        self.movable = True
        # cannot send communication signals
        self.silent = False
        # cannot observe the world
        self.blind = False
        # physical motor noise amount
        self.u_noise = None
        # communication noise amount
        self.c_noise = None
        # control range
        self.u_range = 1.0
        # state
        self.state = AgentState()
        # action
        self.action = Action()
        # script behavior to execute
        self.action_callback = None
<<<<<<< HEAD
        
    def twistToU(self):
        '''
        Twist has to be converted to u as it is used as acceleration vector in the next state calculation.
        So linear.x and angular.y in the local frame has to be converted to x and y acceleration vectors.
        '''
        if self.action.twist is not None:
            if self.action.twist.shape == (1,2):
                self.action.twist = self.action.twist.reshape(2,)
            theta = pi*self.state.p_pose[2]
            self.action.u = np.asarray([self.action.twist[0]*cos(theta)+self.action.twist[1]*cos(theta+pi/2),
                                        self.action.twist[0]*sin(theta)+self.action.twist[1]*sin(theta+pi/2)])


=======
>>>>>>> 69ee7f85811c77ee651722bc3c332677b2195da1

# multi-agent world
class World(object):
    def __init__(self):
        # list of agents and entities (can change at execution-time!)
        self.agents = []
        self.landmarks = []
        # communication channel dimensionality
        self.dim_c = 0
        # position dimensionality
        self.dim_p = 2
        # color dimensionality
        self.dim_color = 3
        # simulation timestep
        self.dt = 0.1
        # physical damping
        self.damping = 0.25
        # contact response parameters
        self.contact_force = 1e+2
        self.contact_margin = 1e-3
<<<<<<< HEAD
        self.obs_n = None
=======
>>>>>>> 69ee7f85811c77ee651722bc3c332677b2195da1

    # return all entities in the world
    @property
    def entities(self):
        return self.agents + self.landmarks

    # return all agents controllable by external policies
    @property
    def policy_agents(self):
        return [agent for agent in self.agents if agent.action_callback is None]

    # return all agents controlled by world scripts
    @property
    def scripted_agents(self):
        return [agent for agent in self.agents if agent.action_callback is not None]

<<<<<<< HEAD
    def angle0To360(self, angle):
        if angle < 0:
            angle = angle + 2*pi
        return angle
    
    def anglepiTopi(self, angle):
        if angle < -pi:
            angle = angle + 2*pi
        if angle > pi:
            angle = angle - 2*pi
        return angle

=======
>>>>>>> 69ee7f85811c77ee651722bc3c332677b2195da1
    # update state of the world
    def step(self):
        # set actions for scripted agents 
        for agent in self.scripted_agents:
            agent.action = agent.action_callback(agent, self)
        # gather forces applied to entities
        p_force = [None] * len(self.entities)
        # apply agent physical controls
        p_force = self.apply_action_force(p_force)
        # apply environment forces
        p_force = self.apply_environment_force(p_force)
<<<<<<< HEAD
        # 20201201 distance_prev 
        for agent in self.agents:
            agent.state.distance_to_goal_prev = sqrt((agent.state.p_pos[0]-agent.state.target[0])**2+(agent.state.p_pos[1]-agent.state.target[1])**2)
        # integrate physical state
        self.integrate_state(p_force)
        # 20201201 distance & angle diff
        for agent in self.agents:
            agent.state.distance_to_goal = sqrt((agent.state.p_pos[0]-agent.state.target[0])**2+(agent.state.p_pos[1]-agent.state.target[1])**2)
            agent.state.global_angle = self.angle0To360(atan2(agent.state.target[1] - agent.state.p_pos[1], agent.state.target[0] - agent.state.p_pos[0]))
            agent.state.angle_diff = self.anglepiTopi((agent.state.global_angle - self.angle0To360(agent.state.p_pose[2])))
        # update agent state
        for agent in self.agents:
            self.update_agent_pose(agent)
            self.update_agent_state(agent)
            # print("global angle: {}".format(agent.state.global_angle))
            # print("Local Angle: {}".format(agent.state.p_pose[2]))
            # print("angle diff: {}".format(agent.state.angle_diff))
=======
        # integrate physical state
        self.integrate_state(p_force)
        # update agent state
        for agent in self.agents:
            self.update_agent_state(agent)
>>>>>>> 69ee7f85811c77ee651722bc3c332677b2195da1

    # gather agent action forces
    def apply_action_force(self, p_force):
        # set applied forces
        for i,agent in enumerate(self.agents):
            if agent.movable:
                noise = np.random.randn(*agent.action.u.shape) * agent.u_noise if agent.u_noise else 0.0
                p_force[i] = agent.action.u + noise                
        return p_force

    # gather physical forces acting on entities
    def apply_environment_force(self, p_force):
        # simple (but inefficient) collision response
        for a,entity_a in enumerate(self.entities):
            for b,entity_b in enumerate(self.entities):
                if(b <= a): continue
                [f_a, f_b] = self.get_collision_force(entity_a, entity_b)
                if(f_a is not None):
                    if(p_force[a] is None): p_force[a] = 0.0
                    p_force[a] = f_a + p_force[a] 
                if(f_b is not None):
                    if(p_force[b] is None): p_force[b] = 0.0
                    p_force[b] = f_b + p_force[b]        
        return p_force

    # integrate physical state
    def integrate_state(self, p_force):
        for i,entity in enumerate(self.entities):
            if not entity.movable: continue
            entity.state.p_vel = entity.state.p_vel * (1 - self.damping)
            if (p_force[i] is not None):
                entity.state.p_vel += (p_force[i] / entity.mass) * self.dt
            if entity.max_speed is not None:
                speed = np.sqrt(np.square(entity.state.p_vel[0]) + np.square(entity.state.p_vel[1]))
                if speed > entity.max_speed:
                    entity.state.p_vel = entity.state.p_vel / np.sqrt(np.square(entity.state.p_vel[0]) +
                                                                  np.square(entity.state.p_vel[1])) * entity.max_speed
            entity.state.p_pos += entity.state.p_vel * self.dt
<<<<<<< HEAD
    def update_agent_pose(self, agent):
        '''
        Pose is updated in this function as intergrate_state only cares x, y p,v,a. 
        '''
        # 20201201 Wrapper for angle conversion is added. 
        agent.state.p_pose[2] = self.anglepiTopi(agent.state.p_pose[2]+agent.action.twist[1]*self.dt)
        agent.state.p_pose = [agent.state.p_pos[0], agent.state.p_pos[1], agent.state.p_pose[2]]

=======
>>>>>>> 69ee7f85811c77ee651722bc3c332677b2195da1

    def update_agent_state(self, agent):
        # set communication state (directly for now)
        if agent.silent:
            agent.state.c = np.zeros(self.dim_c)
        else:
            noise = np.random.randn(*agent.action.c.shape) * agent.c_noise if agent.c_noise else 0.0
            agent.state.c = agent.action.c + noise      

    # get collision forces for any contact between two entities
    def get_collision_force(self, entity_a, entity_b):
        if (not entity_a.collide) or (not entity_b.collide):
            return [None, None] # not a collider
        if (entity_a is entity_b):
            return [None, None] # don't collide against itself
        # compute actual distance between entities
        delta_pos = entity_a.state.p_pos - entity_b.state.p_pos
        dist = np.sqrt(np.sum(np.square(delta_pos)))
        # minimum allowable distance
        dist_min = entity_a.size + entity_b.size
        # softmax penetration
        k = self.contact_margin
        penetration = np.logaddexp(0, -(dist - dist_min)/k)*k
        force = self.contact_force * delta_pos / dist * penetration
        force_a = +force if entity_a.movable else None
        force_b = -force if entity_b.movable else None
<<<<<<< HEAD
        return [force_a, force_b]

    

=======
        return [force_a, force_b]
>>>>>>> 69ee7f85811c77ee651722bc3c332677b2195da1
