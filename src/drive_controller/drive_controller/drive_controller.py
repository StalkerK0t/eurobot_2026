#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from enum import Enum

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose, FollowWaypoints, ComputePathToPose, ComputePathThroughPoses
from nav2_msgs.srv import LoadMap, ClearEntireCostmap, ManageLifecycleNodes, GetCostmap

from tf_transformations import quaternion_from_euler

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import ParameterType

from std_msgs.msg import String
from std_msgs.msg import UInt8, Int16


class NavigationResult(Enum):    
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3 

class BasicNavigator(Node):
    def __init__(self):
        super().__init__(node_name='drive_controller')        

        self.declare_parameter('points', [0.0])    # последовательность точек загрузки/разгрузки   
        self.declare_parameter('point_zones', [''])  # последовательность отключаемых зон (None, если ничего не отключаем)
        self.declare_parameter('time_until_end')    

        # Read params from YAML
        raw_coords = self.get_parameter("points").get_parameter_value().double_array_value
        raw_zones = self.get_parameter("point_zones").get_parameter_value().string_array_value

        # Validate length
        if len(raw_coords) % 3 != 0:
            raise ValueError("point_coords must be divisible by 3 (x, y, yaw per point)")

        num_points = len(raw_coords) // 3
        if len(raw_zones) != num_points:
            raise ValueError("point_zones length must match number of points")        

      #  self.get_logger().info(f"START NAVIGATOR {raw_points}, {len(raw_points)}")

        self.points = []
        for i in range(num_points):
            idx = i * 3
            self.points.append({
                'x': raw_coords[idx],
                'y': raw_coords[idx+1],
                'yaw': raw_coords[idx+2],
                'kz': raw_zones[i],   # имя keepout zone, которую надо отключить после достижения точки; если не надо ничего отключать = None
            })    

        self.time_until_end = self.get_parameter("time_until_end").get_parameter_value().integer_value             

        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        self.start_timer = time.time()      # TEMP!

        self.navigation_in_progress = False # True means that robot is going to point
        # self.elevator_in_progress = False # True means that robot work with elevator        
        self.current_obstacle = 0  
        self.current_point = 0        

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.initial_pose_received = False
        # self.nav_through_poses_client = ActionClient(self,
        #                                              NavigateThroughPoses,
        #                                              'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        # self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        # self.compute_path_through_poses_client = ActionClient(self, ComputePathThroughPoses,
        #                                                       'compute_path_through_poses')
        

        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        self.timer_sub = self.create_subscription(UInt8, '/timer', self.timer_counter, 10)          
        
        
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        self.obstacle_pub = self.create_publisher(String, '/keepout_zone', 10)

        # self.change_maps_srv = self.create_client(LoadMap, '/map_server/load_map')
        # self.clear_costmap_global_srv = self.create_client(
        #     ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        # self.clear_costmap_local_srv = self.create_client(
        #     ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        # self.get_costmap_global_srv = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        # self.get_costmap_local_srv = self.create_client(GetCostmap, '/local_costmap/get_costmap')

        self.waitUntilNav2Active()  
        self.create_timer(1, self.timer_callback)

    def setInitialPose(self, initial_pose):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def set_goal_pose(self, waypoint_index : int = 0):        
        yaw = self.points[waypoint_index]['yaw']
        quaternion = quaternion_from_euler(0, 0, yaw)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float( self.points[waypoint_index]['x'] )
        goal_pose.pose.position.y = float( self.points[waypoint_index]['y'] )
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]
  
        return goal_pose

    def timer_counter(self, msg):
        if self.start_timer is None and msg.data == 1:
            self.get_logger().debug("Start")
            self.start_timer = time.time()

    def timer_callback(self):        
        if self.start_timer is not None:            
            self.get_logger().info(f"Time = {time.time() - self.start_timer}")

            # when time is up, go to final point 
            # if (time.time() - self.start_timer) >= self.time_until_end and (time.time() - self.start_timer) < 100:
            if (time.time() - self.start_timer) < self.time_until_end:
                self.get_logger().info(f"Len wayp = {len(self.points)}, current_point = {self.current_point}")                
                self.go_to_pose( self.set_goal_pose( self.current_point ) )

            else:
                self.get_logger().info(f"Go to the base. Time = {time.time() - self.start_timer}, Points = {len(self.points)}")    

                index = len(self.points) - 1 # Base (last point)
                self.go_to_pose( self.set_goal_pose( index ) )                    

    def go_to_pose(self, pose):
        self.navigation_in_progress = True

        # Sends a `NavToPose` action request
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                      str(pose.pose.position.y) + '...')
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected')

            # Цель недостижима. Можно пропустить тогда и перейти к следующему складу брусков / зоне выгрузки ? 
            self.current_point += 2 # пропускаем по 2, т. к. если не доехали до зоны загрузки, нам снова нужна именно зона загрузки
                    
            # self.action_tact += 1
            self.navigation_in_progress = False
            return
        
        self.get_logger().debug('Goal accepted')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        self.get_logger().info("Call_result_callback")
        self.navigation_in_progress = False

        if self.isNavComplete():            
            self.update_obstacle( self.points[self.current_point]['kz'] ) # закрываем область, куда выгрузили орехи (если не None)
            self.current_point += 1

        if len(self.points) == self.current_point:
            self.get_logger().info("All goals complite")

            # self.screen_sum += self.term_order[len(self.term_order) - 1]
            # self.current_term = len(self.term_order)
            # msg = Int16()
            # msg.data = self.screen_sum
            # self.screen_pub.publish(msg)        

    def update_obstacle(self, obstacle_name):
        """
        Перекрываем на keepout_layer зону, на которую мы или противник выложили бруски
        """
        msg = String()
        msg.data = f'{obstacle_name}'
        self.obstacle_pub.publish(msg)
        #self.get_logger().debug(f'Sent command: {msg.data}')
        self.current_obstacle += 1

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def is_nav_complete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Goal succeeded!')
        return True

    # def isNavComplete(self):
    #     if not self.result_future:
    #         # task was cancelled or completed
    #         return True
    #     rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
    #     if self.result_future.result():
    #         self.status = self.result_future.result().status
    #         if self.status != GoalStatus.STATUS_SUCCEEDED:
    #             self.debug('Goal with failed with status code: {0}'.format(self.status))
    #             return True
    #     else:
    #         # Timed out, still processing, not complete yet
    #         return False

    #     self.debug('Goal succeeded!')
    #     return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return NavigationResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return NavigationResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return NavigationResult.CANCELED
        else:
            print(self.status)
            return NavigationResult.UNKNOWN

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    # def followWaypoints(self, poses):
    #     # Sends a `FollowWaypoints` action request
    #     self.debug("Waiting for 'FollowWaypoints' action server")
    #     while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
    #         self.info("'FollowWaypoints' action server not available, waiting...")

    #     goal_msg = FollowWaypoints.Goal()
    #     goal_msg.poses = poses

    #     self.info('Following ' + str(len(goal_msg.poses)) + ' goals.' + '...')
    #     send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg,
    #                                                                     self._feedbackCallback)
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error('Following ' + str(len(poses)) + ' waypoints request was rejected!')
    #         return False

    #     self.result_future = self.goal_handle.get_result_async()
    #     return True

    # def getPath(self, start, goal):
    #     # Sends a `NavToPose` action request
    #     self.debug("Waiting for 'ComputePathToPose' action server")
    #     while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
    #         self.info("'ComputePathToPose' action server not available, waiting...")

    #     goal_msg = ComputePathToPose.Goal()
    #     goal_msg.goal = goal
    #     goal_msg.start = start

    #     self.info('Getting path...')
    #     send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal_msg)
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error('Get path was rejected!')
    #         return None

    #     self.result_future = self.goal_handle.get_result_async()
    #     rclpy.spin_until_future_complete(self, self.result_future)
    #     self.status = self.result_future.result().status
    #     if self.status != GoalStatus.STATUS_SUCCEEDED:
    #         self.warn('Getting path failed with status code: {0}'.format(self.status))
    #         return None

    #     return self.result_future.result().result.path

    # def getPathThroughPoses(self, start, goals):
    #     # Sends a `NavToPose` action request
    #     self.debug("Waiting for 'ComputePathThroughPoses' action server")
    #     while not self.compute_path_through_poses_client.wait_for_server(timeout_sec=1.0):
    #         self.info("'ComputePathThroughPoses' action server not available, waiting...")

    #     goal_msg = ComputePathThroughPoses.Goal()
    #     goal_msg.goals = goals
    #     goal_msg.start = start

    #     self.info('Getting path...')
    #     send_goal_future = self.compute_path_through_poses_client.send_goal_async(goal_msg)
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error('Get path was rejected!')
    #         return None

    #     self.result_future = self.goal_handle.get_result_async()
    #     rclpy.spin_until_future_complete(self, self.result_future)
    #     self.status = self.result_future.result().status
    #     if self.status != GoalStatus.STATUS_SUCCEEDED:
    #         self.warn('Getting path failed with status code: {0}'.format(self.status))
    #         return None

    #     return self.result_future.result().result.path

    # def changeMap(self, map_filepath):
    #     while not self.change_maps_srv.wait_for_service(timeout_sec=1.0):
    #         self.info('change map service not available, waiting...')
    #     req = LoadMap.Request()
    #     req.map_url = map_filepath
    #     future = self.change_maps_srv.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     status = future.result().result
    #     if status != LoadMap.Response().RESULT_SUCCESS:
    #         self.error('Change map request failed!')
    #     else:
    #         self.info('Change map request was successful!')
    #     return

    # def clearAllCostmaps(self):
    #     self.clearLocalCostmap()
    #     self.clearGlobalCostmap()
    #     return

    # def clearLocalCostmap(self):
    #     while not self.clear_costmap_local_srv.wait_for_service(timeout_sec=1.0):
    #         self.info('Clear local costmaps service not available, waiting...')
    #     req = ClearEntireCostmap.Request()
    #     future = self.clear_costmap_local_srv.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return

    # def clearGlobalCostmap(self):
    #     while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=1.0):
    #         self.info('Clear global costmaps service not available, waiting...')
    #     req = ClearEntireCostmap.Request()
    #     future = self.clear_costmap_global_srv.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return

    # def getGlobalCostmap(self):
    #     while not self.get_costmap_global_srv.wait_for_service(timeout_sec=1.0):
    #         self.info('Get global costmaps service not available, waiting...')
    #     req = GetCostmap.Request()
    #     future = self.get_costmap_global_srv.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result().map

    # def getLocalCostmap(self):
    #     while not self.get_costmap_local_srv.wait_for_service(timeout_sec=1.0):
    #         self.info('Get local costmaps service not available, waiting...')
    #     req = GetCostmap.Request()
    #     future = self.get_costmap_local_srv.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     return future.result().map

    # def lifecycleStartup(self):
    #     self.info('Starting up lifecycle nodes based on lifecycle_manager.')
    #     srvs = self.get_service_names_and_types()
    #     for srv in srvs:
    #         if srv[1][0] == 'nav2_msgs/srv/ManageLifecycleNodes':
    #             srv_name = srv[0]
    #             self.info('Starting up ' + srv_name)
    #             mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
    #             while not mgr_client.wait_for_service(timeout_sec=1.0):
    #                 self.info(srv_name + ' service not available, waiting...')
    #             req = ManageLifecycleNodes.Request()
    #             req.command = ManageLifecycleNodes.Request().STARTUP
    #             future = mgr_client.call_async(req)

    #             # starting up requires a full map->odom->base_link TF tree
    #             # so if we're not successful, try forwarding the initial pose
    #             while True:
    #                 rclpy.spin_until_future_complete(self, future, timeout_sec=0.10)
    #                 if not future:
    #                     self._waitForInitialPose()
    #                 else:
    #                     break
    #     self.info('Nav2 is ready for use!')
    #     return

    # def lifecycleShutdown(self):
    #     self.info('Shutting down lifecycle nodes based on lifecycle_manager.')
    #     srvs = self.get_service_names_and_types()
    #     for srv in srvs:
    #         if srv[1][0] == 'nav2_msgs/srv/ManageLifecycleNodes':
    #             srv_name = srv[0]
    #             self.info('Shutting down ' + srv_name)
    #             mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
    #             while not mgr_client.wait_for_service(timeout_sec=1.0):
    #                 self.info(srv_name + ' service not available, waiting...')
    #             req = ManageLifecycleNodes.Request()
    #             req.command = ManageLifecycleNodes.Request().SHUTDOWN
    #             future = mgr_client.call_async(req)
    #             rclpy.spin_until_future_complete(self, future)
    #             future.result()
    #     return



    # def goThroughPoses(self, poses):
    #     # Sends a `NavThroughPoses` action request
    #     self.debug("Waiting for 'NavigateThroughPoses' action server")
    #     while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
    #         self.info("'NavigateThroughPoses' action server not available, waiting...")

    #     goal_msg = NavigateThroughPoses.Goal()
    #     goal_msg.poses = poses

    #     self.info('Navigating with ' + str(len(goal_msg.poses)) + ' goals.' + '...')
    #     send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg,
    #                                                                      self._feedbackCallback)
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error('Goal with ' + str(len(poses)) + ' poses was rejected!')
    #         return False

    #     self.result_future = self.goal_handle.get_result_async()
    #     return True

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

def main(args=None):
    rclpy.init(args=args)
    node = BasicNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    