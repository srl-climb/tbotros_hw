from __future__ import annotations

import yaml
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from custom_msgs.msg import Statusword, MotorPosition, CanError, DigitalInputs
from custom_actions.action import MoveMotor
from std_srvs.srv import Trigger
from std_msgs.msg import String, Int16, Float64, Int8, Bool
from canopen.pdo.base import Map
from canopen.emcy import EmcyError
from threading import Lock
from ament_index_python import get_package_prefix
from .canopen_network_base_node import BaseNode


class StateChangeError(Exception):
    """Exception raised when motor controller can not perform the state change."""
    def __init__(self):
        super().__init__("Invalid state change.")

class FaulhaberMotorNode(BaseNode):

    def __init__(self, **args) -> None:

        super().__init__(**args)

        ### ROS OBJECTS ###
        
        name = self.get_name() + '/'
        
        #create publishers/subscriber
        self._pub_statusword = self.create_publisher(Statusword, name + 'statusword', 1)
        self._pub_inputs = self.create_publisher(DigitalInputs, name + 'inputs', 1)
        self._pub_mode = self.create_publisher(Int8, name + 'mode', 1)
        self._pub_state = self.create_publisher(String, name + 'state', 1)
        self._pub_position = self.create_publisher(MotorPosition, name + 'position', 1)
        self._pub_velocity = self.create_publisher(Float64, name + 'velocity', 1)
        self._pub_current = self.create_publisher(Int16, name + 'current', 1)
        self._pub_running = self.create_publisher(Bool, name + 'running', 1)
        self._pub_canerror = self.create_publisher(CanError, name + 'can_error', 1)
        self.create_subscription(MotorPosition, name + 'target_position', self.target_position_callback, 100)

        # create services for controlling the motor's state machine
        self.create_service(Trigger, name + 'shut_down', self.shut_down_callback)
        self.create_service(Trigger, name + 'switch_on', self.switch_on_callback)
        self.create_service(Trigger, name + 'disable_voltage', self.disable_voltage_callback)
        self.create_service(Trigger, name + 'quick_stop', self.quick_stop_callback)
        self.create_service(Trigger, name + 'disable_operation', self.disable_operation_callback)
        self.create_service(Trigger, name + 'enable_operation', self.enable_operation_callback)
        self.create_service(Trigger, name + 'fault_reset', self.fault_reset_callback)
        self.create_service(Trigger, name + 'save_home', self.save_home_callback)

        # create action for moving the motor
        self.create_action_server(MoveMotor, name + 'move', 
                                  callback_group = ReentrantCallbackGroup(),
                                  execute_callback = self.execute_move_callback,
                                  goal_callback = self.goal_move_callback,
                                  cancel_callback = self.cancel_move_callback)

        # lock for protecting execute_move_callback
        self._lock = Lock()

        ### LOAD Files ###
        
        # file paths
        self._config_files = self._params.get('config_files', ['/home/srl-orin/ros2_ws/config/motor_joint.yaml', 
                                                               '/home/srl-orin/ros2_ws/config/motor_joint0.yaml'])
        self._eds_file = self._params.get('eds_file', '/home/srl-orin/ros2_ws/eds/605.3150.68-E.eds')
        self._pos_file = self._params.get('pos_file', '/home/srl-orin/ros2_ws/config/motor_joint0_position.yaml')

        # parse file paths
        for i in range(len(self._config_files)):
            self._config_files[i] = self.parse_file_path(self._config_files[i])
        self._eds_file = self.parse_file_path(self._eds_file)
        self._pos_file = self.parse_file_path(self._pos_file)

        # load config files
        self._config = {}
        self.load_configs()

        # load position file
        self._pos = {}
        self.load_pos()

        ### SET UP CAN NODE ###

        # create motor can node
        self._can_id = self._config.get('can_id', 0)
        self._node = self._network.add_node(self._can_id, self._eds_file, upload_eds = False)
        #self._node.sdo.MAX_RETRIES = 2
        #self._node.sdo.PAUSE_BEFORE_SEND = 0.001
        #self._node.sdo.RESPONSE_TIMEOUT = 1

        # can bus error handling
        self._node.emcy.add_callback(self.can_error_callback)
        
        # apply sdos
        self.apply_sdos()
        self.apply_sdos()
        # NOTE: Apply twice to make sure sdos are configured correctly (some sdos change dependent on another)
        
        ### INITIALIZE MOTOR PARAMETERS ###

        # motor parameters
        self._state = ''
        self._running = False
        self._statusword = Statusword()
        self._actual_position = 0
        self._target_position = 0
        self._initial_position = self._pos.get('position', 0)
        self._homing_offset = self._config.get('homing_offset', 0)
        self._homing_method = self._config.get('homing_method', 0)
        self._csp_velocity = self._config.get('csp_velocity', 0)
        self._csp_acceleration = self._config.get('csp_acceleration', 0)
        self._csp_deceleration = self._config.get('csp_deceleration', 0)
        self._factor = self._config.get('factor', 1) # conversion from ros units -> motor controllers' user defined units -> ros units * factor = motor units
        self._gearratio = float(self._node.sdo[0x6091][0x01].raw) / float(self._node.sdo[0x6091][0x02].raw)
        self._feed = float(self._node.sdo[0x6092][0x01].raw) / float(self._node.sdo[0x6092][0x02].raw)

        ### CONFIGURE PDOS ###

        # read current pdo configuration
        self._node.tpdo.read()
        self._node.rpdo.read()
        
        # modify pdos
        # tpdo[1]: statusword, send whenever there is a change or every 1000ms
        self._node.tpdo[1].clear()
        self._node.tpdo[1].add_variable(0x6041) # statusword
        self._node.tpdo[1].add_variable(0x6060) # operation mode
        self._node.tpdo[1].add_variable(0x60FD) # digital inputs
        self._node.tpdo[1].trans_type = 255
        self._node.tpdo[1].event_timer = 1000
        self._node.tpdo[1].enabled = True
        self._node.tpdo[1].add_callback(self.statusword_tpdo_callback)
        # tdpo[2]: target position/position actual value, send after every sync msg
        self._node.tpdo[2].clear()
        self._node.tpdo[2].add_variable(0x607A) # target position
        self._node.tpdo[2].add_variable(0x6064) # position actual value
        self._node.tpdo[2].trans_type = 5
        self._node.tpdo[2].enabled = True
        self._node.tpdo[2].add_callback(self.position_tpdo_callback)
        # rpdo[2]: target position, send after every sync msg
        self._node.rpdo[2].clear()
        self._node.rpdo[2].add_variable(0x607A) # target position
        self._node.rpdo[2].trans_type = 1
        self._node.rpdo[2].enabled = True
        # tpdo[3]: velocity/current actual value, send after every sync msg
        self._node.tpdo[3].clear()
        self._node.tpdo[3].add_variable(0x606C) # velocity actual value
        self._node.tpdo[3].add_variable(0x6078) # current actual value
        self._node.tpdo[3].trans_type = 5
        self._node.tpdo[3].enabled = True
        self._node.tpdo[3].add_callback(self.velocity_tpdo_callback)
        # Note: pdo max size is 64bit: position, velocity, current values do not fit into one pdo

        # set state to pre-operational to save pdos
        self._node.nmt.state = 'PRE-OPERATIONAL'

        # save pdo configuration
        self._node.tpdo.save()
        self._node.rpdo.save()

        # set state back to operational
        self._node.nmt.state = 'OPERATIONAL'

        ### ADDITIONAL STUFF ###

        # variable used by target_position_callback to identify missing messages
        self._last_message_id = 0

        # variable used by execute_move_callback to determine if a new move was started
        self._current_move_id = int(-1)

        # variable to enable/disable target position subscription
        self._target_position_sub_enabled = False

        self.get_logger().info('Initialization completed')

    def load_configs(self):

        for config_file in self._config_files:
            try:
                with open(config_file, "r") as stream:        
                    data: dict = yaml.safe_load(stream) or {}                
                    self._config.update(data)                 
            except Exception as exc:
                self.get_logger().error('Failed loading config file: ' + str(exc))

    def load_pos(self):

        try:
            with open(self._pos_file, "r") as stream:
                self._pos: dict = yaml.safe_load(stream) or {}
        except Exception as exc:
            self.get_logger().error('Failed loading position file: ' + str(exc))

    def parse_file_path(self, file: str):

        if file.startswith('package://'):
            pkg_name = (file.split('package://'))[1].split('/')[0]
            file = file.replace('package://', get_package_prefix(pkg_name) + '/share/')

        return file

    def apply_sdos(self):

        sdos = self._config['sdos']
   
        for index in sdos.keys():
        
            for subindex in sdos[index].keys():
                value = sdos[index][subindex]['value']

                try:
                    if subindex == 0: 
                        if isinstance(value, list):
                            for i in range(len(value)):
                                self._node.sdo[index].bits[i] = bool(value[i])
                        else:
                            self._node.sdo[index].raw = int(value)
                        # NOTE: For sdos with a single subindex 0x0, [index][subindex] has to be replaced with just [index]
                    else:
                        if isinstance(value, list):
                            for i in range(len(value)):
                                self._node.sdo[index][subindex].bits[i] = bool(value[i])
                        else:
                            self._node.sdo[index][subindex].raw = int(value)
        
                except Exception as exc:
                    self.get_logger().warn('Failed applying sdo (' + hex(int(index)) + ', ' + hex(int(subindex)) + '): ' + str(exc))

    def statusword_tpdo_callback(self, canmsg:Map):
        
        # statusword
        stwd = canmsg[0].bits
        # mode
        mode = canmsg[1].raw
        # inputs
        inputs = canmsg[2].bits

        # publish state
        if (stwd[0],stwd[1],stwd[2],stwd[3],stwd[6]) == (0,0,0,0,0):
            state = 'not ready to switch on'
        elif (stwd[0],stwd[1],stwd[2],stwd[3],stwd[6]) == (0,0,0,0,1):
            state = 'switch on disabled'
        elif (stwd[0],stwd[1],stwd[2],stwd[3],stwd[5],stwd[6]) == (1,0,0,0,1,0):
            state = 'ready to switch on'
        elif (stwd[0],stwd[1],stwd[2],stwd[3],stwd[5],stwd[6]) == (1,1,0,0,1,0):
            state = 'switched on'
        elif (stwd[0],stwd[1],stwd[2],stwd[3],stwd[5],stwd[6]) == (1,1,1,0,1,0):
            state = 'operation enabled'
        elif (stwd[0],stwd[1],stwd[2],stwd[3],stwd[5],stwd[6]) == (1,1,1,0,0,0):
            state = 'quick stop active'
        elif (stwd[0],stwd[1],stwd[2],stwd[3],stwd[6]) == (1,1,1,1,0):
            state = 'fault reaction active'
        elif (stwd[0],stwd[1],stwd[2],stwd[3],stwd[6]) == (0,0,0,1,0):
            state = 'fault'
        else:
            state = 'unknown state'

        msg = String()
        msg.data = state
        self._pub_state.publish(msg)
        self._state = state
        
        # publish statusword
        msg = Statusword()
        msg.ready_to_switch_on = bool(stwd[0])
        msg.switched_on = bool(stwd[1])
        msg.operation_enabled = bool(stwd[2])
        msg.fault = bool(stwd[3])
        msg.voltage_enabled = bool(stwd[4])
        msg.quick_stop = bool(stwd[5])
        msg.switch_on_disabled = bool(stwd[6])
        msg.warning = bool(stwd[7])
        msg.remote = bool(stwd[9])
        msg.target_reached = bool(stwd[10])
        msg.internal_limit_active = bool(stwd[11])
        msg.setpoint_acknowledge_or_speed_or_homing_attained = bool(stwd[12])
        msg.deviation_error = bool(stwd[13])
        
        self._pub_statusword.publish(msg)
        self._statusword = msg

        # publish inputs
        msg = DigitalInputs()
        msg.negative_limit_switch = bool(inputs[0])
        msg.positive_limit_switch = bool(inputs[1])
        msg.homing_switch = bool(inputs[2])

        self._pub_inputs.publish(msg)

        # publish mode
        msg = Int8()
        msg.data = int(mode)

        self._pub_mode.publish(msg)

        # publish running
        msg = Bool()
        msg.data = bool(self._running)

        self._pub_running.publish(msg)

    def position_tpdo_callback(self, canmsg:Map):

        # position
        self._target_position = float(canmsg[0].raw / self._factor)
        self._actual_position = float(canmsg[1].raw / self._factor)
        
        # position message
        msg = MotorPosition()
        msg.target_position = self._target_position
        msg.actual_position = self._actual_position 

        self._pub_position.publish(msg)

    def velocity_tpdo_callback(self, canmsg:Map):

        # velocity message
        msg = Float64()
        msg.data = float(canmsg[0].raw / self._gearratio / (1/self._feed) / self._factor / 60)

        self._pub_velocity.publish(msg)

        # current message
        msg = Int16()
        msg.data  = canmsg[1].raw

        self._pub_current.publish(msg)

    def shut_down_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:          
        
        try:
            if self._state == 'switch on disabled' or \
               self._state == 'switched on' or \
               self._state == 'operation enabled':
                self._node.sdo[0x6040].raw = 0x0006
            else:
                raise StateChangeError()
        except Exception as exc:
            self.get_logger().error(str(exc))
            response.success = False
            response.message = str(exc)
        else:
            response.success = True

        return response
    
    def switch_on_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:          
        
        try:
            if self._state == 'ready to switch on':
                self._node.sdo[0x6040].raw = 0x0007
            else:
                raise StateChangeError()
        except Exception as exc:
            self.get_logger().error(str(exc))
            response.success = False
            response.message = str(exc)
        else:
            response.success = True

        return response
    
    def disable_voltage_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:          
        
        try:
            if self._state == 'ready to switch on' or \
               self._state == 'operation enabled' or \
               self._state == 'switched on' or \
               self._state == 'quick stop active':
                self._node.sdo[0x6040].raw = 0x0008
            else:
                raise StateChangeError()
        except Exception as exc:
            self.get_logger().error(str(exc))
            response.success = False
            response.message = str(exc)
        else:
            response.success = True

        return response
    
    def quick_stop_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:          
        
        try:
            if self._state == 'ready to switch on' or \
               self._state == 'switched on' or \
               self._state == 'operation enabled':
                self._node.sdo[0x6040].raw = 0x000A
            else:
                raise StateChangeError()
        except Exception as exc:
            self.get_logger().error(str(exc))
            response.success = False
            response.message = str(exc)
        else:
            response.success = True

        return response
    
    def disable_operation_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:          
        
        try:
            if self._state == 'operation enabled':
                self._node.sdo[0x6040].raw = 0x0007
            else:
                raise StateChangeError()
        except Exception as exc:
            self.get_logger().error(str(exc))
            response.success = False
            response.message = str(exc)
        else:
            response.success = True

        return response
    
    def enable_operation_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:     
        
        try:
            if self._state == 'ready to switch on' or \
            self._state == 'switched on':
                self._node.sdo[0x6040].raw = 0x000F
            else:
                raise StateChangeError()
        except Exception as exc:
            self.get_logger().error(str(exc))
            response.success = False
            response.message = str(exc)
        else:
            response.success = True

        return response
    
    def fault_reset_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:          
        
        try:
            if self._state == 'fault':
                self._node.sdo[0x6040].raw = 0x0007
            else:
                raise StateChangeError()
        except Exception as exc:
            self.get_logger().error(str(exc))
            response.success = False
            response.message = str(exc)
        else:
            response.success = True

        return response

    def save_home_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:

        response = Trigger.Response()

        try:
            position = self._node.sdo[0x6064].raw / self._factor
        except Exception as exc:
            self.get_logger().error(str(exc))
            response.success = False
            response.message = str(exc)
            return response

        try:
            with open(self._pos_file, "w") as stream:
                data = {'position': position}
                yaml.safe_dump(data, stream)
        except Exception as exc:
            self.get_logger().error('Failed saving position file: ' + str(exc))
            response.success = False
            response.message = str(exc)
            return response

        self._initial_position = position
        response.success = True

        return response

    def execute_move_callback(self, goal_handle: ServerGoalHandle):
        
        move_id = self.get_move_id()
        self._running = True
        
        with self._lock:
            
            self.get_logger().info('Move [%s]: Start' %move_id)
            
            # profile position move parameters
            if goal_handle.request.mode == 0: 
                self._node.sdo[0x6060].raw = 1
                self._node.sdo[0x6040].bits[6] = goal_handle.request.absolute_relative
                self._node.sdo[0x607A].raw = goal_handle.request.target_position * self._factor # 0x607A in in user units (-> the controller internally factors in gear ratio and feed)
                self._node.sdo[0x6081].raw = goal_handle.request.profile_velocity * self._gearratio * (1/self._feed) * self._factor * 60 # 0x6081 is in 1/min
                self._node.sdo[0x6083].raw = goal_handle.request.profile_acceleration * self._gearratio * (1/self._feed) * self._factor  # 0x6081 is in 1/s^2
                self._node.sdo[0x6084].raw = goal_handle.request.profile_deceleration * self._gearratio * (1/self._feed) * self._factor  # 0x6081 is in 1/s^2
            # homing move with config parameters
            elif goal_handle.request.mode == 1: 
                self._node.sdo[0x607C].raw = self._homing_offset * self._factor
                self._node.sdo[0x6098].raw = self._homing_method
                self._node.sdo[0x6060].raw = 6
            # cyclic synchronous positioning parameters
            elif goal_handle.request.mode == 2:
                self._node.sdo[0x607A].raw = self._node.sdo[0x6064].raw # set target position to actual position
                self._node.sdo[0x6081].raw = self._csp_velocity * self._gearratio * (1/self._feed) * self._factor * 60 # 0x6081 is in 1/min
                self._node.sdo[0x6083].raw = self._csp_acceleration * self._gearratio * (1/self._feed) * self._factor  # 0x6081 is in 1/s^2
                self._node.sdo[0x6084].raw = self._csp_acceleration * self._gearratio * (1/self._feed) * self._factor  # 0x6081 is in 1/s^2
                self._node.sdo[0x6060].raw = 8
            # homing move with initial position parameters
            elif goal_handle.request.mode == 3:
                self._node.sdo[0x6060].raw = 6
                self._node.sdo[0x607C].raw = self._initial_position * self._factor
                self._node.sdo[0x6098].raw = 35
            # homing move with offset
            elif goal_handle.request.mode == 4:
                self._node.sdo[0x6060].raw = 6
                self._node.sdo[0x607C].raw = goal_handle.request.homing_offset * self._factor
                self._node.sdo[0x6098].raw = 35
            
            # subscriber for receiving position commands
            if goal_handle.request.mode == 2:
                self._target_position_sub_enabled = True

            # start move
            self._node.sdo[0x6040].bits[4] = 0 # start
            self._node.sdo[0x6040].bits[4] = 1 # move

            # publish feedback
            feedback = MoveMotor.Feedback()
            rate     = self.create_rate(4, self.get_clock())
            aborted        = False
            canceled       = False
            done           = False
            quick_stopped  = False
            
            while not aborted and not canceled and not done and not quick_stopped:   
                rate.sleep()     

                feedback.actual_position = self._actual_position
                feedback.target_position = self._target_position
                goal_handle.publish_feedback(feedback)

                aborted       = move_id != self._current_move_id   # move was overwritten by new move      
                canceled      = goal_handle.is_cancel_requested    # move was canceled/stopped
                quick_stopped = self._state == 'quick stop active' # move was quick stopped
                done          = self._statusword.target_reached    # move reached target

            if goal_handle.request.mode == 2:
                self._target_position_sub_enabled = False

            if canceled:
                goal_handle.canceled()
                self.get_logger().info('Move [%s]: Canceled' %move_id)
            elif aborted or quick_stopped:
                goal_handle.abort()
                self.get_logger().info('Move [%s]: Aborted' %move_id)
            else:
                goal_handle.succeed()
                self.get_logger().info('Move [%s]: Completed' %move_id)  

        self._running = False

        return MoveMotor.Result()

    def cancel_move_callback(self, _):
        
        # stop move elegantly
        self._node.sdo[0x6081].raw = 0      # this stops positioning
        self._node.sdo[0x6040].bits[4] = 0  # this stops homing

        return CancelResponse.ACCEPT
    
    def goal_move_callback(self, _):

        return GoalResponse.ACCEPT

    def target_position_callback(self, msg: MotorPosition): 
        
        if self._target_position_sub_enabled:
            self._node.rpdo[2][0].raw = msg.target_position * self._factor
            self._node.rpdo[2].transmit()

    def get_move_id(self) -> int:
        
        self._current_move_id = self._current_move_id + 1
        
        return self._current_move_id

    def can_error_callback(self, error: EmcyError):
        
        msg = CanError()
        msg.code = int(error.code)
        msg.canregister = int(error.register)
        msg.timestamp = float(error.timestamp)
        msg.description = str(error.get_desc())
        
        self._pub_canerror.publish(msg)

        self.get_logger().error('CAN error: ' + msg.description)