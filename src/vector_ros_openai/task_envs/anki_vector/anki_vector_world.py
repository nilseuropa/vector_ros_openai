import rospy
import numpy
from gym import spaces
from vector_ros_openai.robot_envs import anki_vector_env
from gym.envs.registration import register
from geometry_msgs.msg import Vector3
from vector_ros_openai.task_envs.task_commons import LoadYamlFileParamsTest
from vector_ros_openai.openai_ros_common import ROSLauncher
import os


class AnkiVectorWorldEnv(anki_vector_env.AnkiVectorEnv):
    def __init__(self):
        """
        This Task Env is designed for having the AnkiVector in the anki_vector world
        closed room with columns.
        It will learn how to move around without crashing.
        """
        # This is the path where the simulation files, the Task and the Robot gits will be downloaded if not there
        ros_ws_abspath = rospy.get_param("/anki_vector/ros_ws_abspath", None)
        assert ros_ws_abspath is not None, "You forgot to set ros_ws_abspath in your yaml file of your main RL script. Set ros_ws_abspath: \'YOUR/SIM_WS/PATH\'"
        assert os.path.exists(ros_ws_abspath), "The Simulation ROS Workspace path " + ros_ws_abspath + \
                                               " DOES NOT exist, execute: mkdir -p " + ros_ws_abspath + \
                                               "/src;cd " + ros_ws_abspath + ";catkin_make"

        ROSLauncher(rospackage_name="anki_description",
                    launch_file_name="start_world.launch",
                    ros_ws_abspath=ros_ws_abspath)

        # Load Params from the desired Yaml file
        LoadYamlFileParamsTest(rospackage_name="vector_ros_openai",
                               rel_path_from_package_to_file="src/vector_ros_openai/task_envs/anki_vector/config",
                               yaml_file_name="anki_vector_world.yaml")


        # Here we will add any init functions prior to starting the MyRobotEnv
        super(AnkiVectorWorldEnv, self).__init__(ros_ws_abspath)

        # Only variable needed to be set here
        number_actions = rospy.get_param('/anki_vector/n_actions')
        self.action_space = spaces.Discrete(number_actions)

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)


        #number_observations = rospy.get_param('/anki_vector/n_observations')
        """
        We set the Observation space for the 6 observations
        cube_observations = [
            round(current_disk_roll_vel, 0),
            round(y_distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(y_linear_speed,1),
            round(yaw, 1),
        ]
        """

        # Actions and Observations
        self.linear_forward_speed = rospy.get_param('/anki_vector/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/anki_vector/linear_turn_speed')
        self.angular_speed = rospy.get_param('/anki_vector/angular_speed')
        self.init_linear_forward_speed = rospy.get_param('/anki_vector/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param('/anki_vector/init_linear_turn_speed')

        self.new_ranges = rospy.get_param('/anki_vector/new_ranges')
        self.min_range = rospy.get_param('/anki_vector/min_range')
        self.max_laser_value = rospy.get_param('/anki_vector/max_laser_value')
        self.min_laser_value = rospy.get_param('/anki_vector/min_laser_value')
        self.max_linear_aceleration = rospy.get_param('/anki_vector/max_linear_aceleration')

        # laser_scan = self.get_laser_scan()
        # num_laser_readings = int(len(laser_scan.ranges)/self.new_ranges)
        num_laser_readings = 1 # single TOF sensor
        high = numpy.full((num_laser_readings), self.max_laser_value)
        low = numpy.full((num_laser_readings), self.min_laser_value)

        # We only use two integers
        self.observation_space = spaces.Box(low, high)

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))

        # Rewards
        self.forwards_reward = rospy.get_param("/anki_vector/forwards_reward")
        self.turn_reward = rospy.get_param("/anki_vector/turn_reward")
        self.end_episode_points = rospy.get_param("/anki_vector/end_episode_points")

        self.cumulated_steps = 0.0


    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_base( self.init_linear_forward_speed,
                        self.init_linear_turn_speed,
                        epsilon=0.05,
                        update_rate=10)

        return True


    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False


    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the AnkiVector
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """

        rospy.logdebug("Start Set Action ==>"+str(action))
        if action == 0: #FORWARD
            linear_speed = self.linear_forward_speed
            angular_speed = 0.0
            self.last_action = "FORWARDS"
        elif action == 1: #LEFT
            linear_speed = self.linear_turn_speed
            angular_speed = self.angular_speed
            self.last_action = "TURN_LEFT"
        elif action == 2: #RIGHT
            linear_speed = self.linear_turn_speed
            angular_speed = -1*self.angular_speed
            self.last_action = "TURN_RIGHT"

        # We tell AnkiVector the linear and angular speed to set to execute
        self.move_base(linear_speed, angular_speed, epsilon=0.05, update_rate=10)

        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        AnkiVectorEnv API DOCS
        :return:
        """
        # rospy.logdebug("Start Get Observation ==>")
        # We get the laser scan data
        # laser_scan = self.get_laser_scan()
        # discretized_observations = self.discretize_scan_observation(laser_scan,self.new_ranges)
        # rospy.logdebug("Observations==>"+str(discretized_observations))
        # rospy.logdebug("END Get Observation ==>")
        # return discretized_observations

        observations = [0]
        observations[0] = self.get_laser_scan()
        return observations


    def _is_done(self, observations):

        self._episode_done = False
        if (self.min_range > observations[0].range > 0):
            self._episode_done = True

        if self._episode_done:
            rospy.logerr("AnkiVector is Too Close to wall==>")
        else:
            rospy.logwarn("AnkiVector is NOT close to a wall ==>")# +str(observations[0]))

        # Now we check if it has crashed based on the imu
        imu_data = self.get_imu()
        linear_acceleration_magnitude = self.get_vector_magnitude(imu_data.linear_acceleration)
        if linear_acceleration_magnitude > self.max_linear_aceleration:
            rospy.logerr("AnkiVector Crashed==>"+str(linear_acceleration_magnitude)+">"+str(self.max_linear_aceleration))
            self._episode_done = True
        else:
            rospy.logerr("DID NOT crash AnkiVector ==>"+str(linear_acceleration_magnitude)+">"+str(self.max_linear_aceleration))


        return self._episode_done

    def _compute_reward(self, observations, done):

        if not done:
            if self.last_action == "FORWARDS":
                reward = self.forwards_reward
            else:
                reward = self.turn_reward
        else:
            reward = -1*self.end_episode_points


        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward


    # Internal TaskEnv Methods
    # LaserScan to discrete array

    # def discretize_scan_observation(self,data,new_ranges):
    #     """
    #     Discards all the laser readings that are not multiple in index of new_ranges
    #     value.
    #     """
    #     self._episode_done = False
    #
    #     discretized_ranges = []
    #     mod = len(data.ranges)/new_ranges
    #
    #     rospy.logdebug("data=" + str(data))
    #     rospy.logdebug("new_ranges=" + str(new_ranges))
    #     rospy.logdebug("mod=" + str(mod))
    #
    #     for i, item in enumerate(data.ranges):
    #         if (i%mod==0):
    #             if item == float ('Inf') or numpy.isinf(item):
    #                 discretized_ranges.append(self.max_laser_value)
    #             elif numpy.isnan(item):
    #                 discretized_ranges.append(self.min_laser_value)
    #             else:
    #                 discretized_ranges.append(int(item))
    #
    #             if (self.min_range > item > 0):
    #                 rospy.logerr("done Validation >>> item=" + str(item)+"< "+str(self.min_range))
    #                 self._episode_done = True
    #             else:
    #                 rospy.logdebug("NOT done Validation >>> item=" + str(item)+"< "+str(self.min_range))
    #
    #
    #     return discretized_ranges


    def get_vector_magnitude(self, vector):
        """
        It calculated the magnitude of the Vector3 given.
        This is usefull for reading imu accelerations and knowing if there has been
        a crash
        :return:
        """
        contact_force_np = numpy.array((vector.x, vector.y, vector.z))
        force_magnitude = numpy.linalg.norm(contact_force_np)

        return force_magnitude
