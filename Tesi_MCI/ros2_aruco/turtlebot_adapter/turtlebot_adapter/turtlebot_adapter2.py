import argparse
import socket
import threading
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from ros2_aruco_interfaces.msg import ArucoMarkers
import threading
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool




class TurtleBotAdapter(Node):
    def __init__(self , IP , port): #self , session ,useGoogleKey ,IP ,port
        super().__init__('turtlebot_adapter2')

        # #socket connection
        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serverSocket.bind((IP, port))
        self.serverSocket.listen(10) #da modificare
        self.get_logger().info(f'Listening on {IP}:{port}')
        self.condition = threading.Condition()
        self.result = None
        self.take_package_goal = [1.0, 1.5] 
        self.bring_package_goal = [1.0, 0.5]
        self.action_client1 = ActionClient(self, NavigateToPose, 'robot2/navigate_to_pose')
        self.action_client1.wait_for_server( timeout_sec=10.0)
        self.marker_pose_subscriber = self.create_subscription(ArucoMarkers, '/robot2/aruco_markers', self.marker_pose_callback, 10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.ack_pub = self.create_publisher(Bool, '/robot2/ack', 10)
        self.ack = False
        self.marker = None
        
        self.state = None
        self.robot_pose = [2.0 , 0.0]
        self.get_logger().info('TurtleBot Adapter 2 has been started')
        
    def marker_pose_callback(self, msg):
        self.marker = msg
        
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
           
    def get_result_callback(self, future):
        with self.condition:
            self.result = future.result().result
            self.condition.notify_all()
    def handle_service_request(self):
        try:
            while True:
                
                (clientConnected, clientAddress) = self.serverSocket.accept()
                print ("Accepted a connection request from %s:%s"%(clientAddress[0], clientAddress[1]))
                
                dataFromClient = clientConnected.recv(1024)
                clientConnected.settimeout(440)
                request = str(dataFromClient.decode())
                ID = request.split("|")[0]
                request_type = request.split("|")[1]
                data = request.split("|")[2]
                flag = False
                self.get_logger().info(f'Request type: {request_type}')

                if request_type == "PERFORMING":
                    self.ack_pub.publish(Bool(data=False))
                    self.state = 'PART1'
                    start_time = time.time()  # Start the timer
                    self.performAction(data)
                    with self.condition:
                        while self.result is None:
                            self.condition.wait()
                        self.get_logger().info('PART1  concluded, goal reached') 
                    time.sleep(3)
                    self.result = None
                    self.state = 'PART2'
                    
                    self.performAction(data)
                    
                    with self.condition:
                        while self.result is None:
                            elapsed_time = time.time() - start_time  # Calculate elapsed time
                            if elapsed_time > 100:  # If elapsed time is more than 15 seconds
                                clientConnected.send(str(False).encode())  # Send False
                                flag = True
                                self.get_logger().info('Action not performed')
                                break
                            self.condition.wait()
                        self.get_logger().info('PART2 concluded, goal reached') 
                    if flag == False:
                        clientConnected.send(str(True).encode())  
                        self.get_logger().info('Action performed')
                    else :
                        flag = False
                    self.result = None 
                    
                    
                elif request_type == "VERIFYING":
                    start_time = time.time()  # Start the timer
                    self.ack = False 
                    self.ack_pub.publish(Bool(data=True))
                    perceived_outcome = self.performVerification(data)
                    while self.ack == False:
                        elapsed_time = time.time() - start_time  # Calculate elapsed time
                        if elapsed_time > 100:  # If elapsed time is more than 70 seconds
                            perceived_outcome = str(False).encode() # Send False
                            break
                    self.ack_pub.publish(Bool(data=False))
                    clientConnected.send(perceived_outcome)
                    self.get_logger().info('Verification performed')
        except KeyboardInterrupt:
            ## Stop recognition
            pass 
            
    def performAction(self,data): #self, data
        action_ID = data.split("/")[1]
        goal_msg = NavigateToPose.Goal()
        
        if  action_ID == 'A1' :  #take the package
            if self.state == 'PART1':
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.pose.position.x = 0.22
                goal_msg.pose.pose.position.y = 2.73
                goal_msg.pose.pose.orientation.w = 1.0
                self.robot_pose = [self.take_package_goal[0], self.take_package_goal[1]]
                self.send_goal1(goal_msg)
                
            elif self.state== 'PART2':
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.pose.position.x = -2.64
                goal_msg.pose.pose.position.y = -0.64
                goal_msg.pose.pose.orientation.w = -1.0
                self.robot_pose = [0.5, -0.5]
                self.send_goal1(goal_msg)
                
        elif action_ID == 'A2':
            if self.state == 'PART1':
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.pose.position.x = 1.0
                goal_msg.pose.pose.position.y = 0.5
                goal_msg.pose.pose.orientation.w = 1.0
                self.robot_pose = [1.0, 0.5]
                self.send_goal1(goal_msg)  
            elif self.state == 'PART2':
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.pose.position.x = 2.0
                goal_msg.pose.pose.position.y = 0.5
                goal_msg.pose.pose.orientation.w = 0.5
                self.robot_pose = [0.0, 0.5]
                self.send_goal1(goal_msg)  
    
    def send_goal1(self, goal_msg):
        self.goal_handle_future = self.action_client1.send_goal_async(goal_msg)
        self.goal_handle_future.add_done_callback(self.goal_response_callback)
        
    def performVerification(self,data ):    
        action_ID = data.split("/")[1]
        perceived_outcome = str(False).encode()
        velocity = Twist()
        if action_ID == 'A1':
            self.get_logger().info('Verification of A1 started')
            start_time = time.time()  # Start the timer

            while self.marker is None:
                elapsed_time = time.time() - start_time  # Calculate elapsed time
                if elapsed_time > 100:  # If elapsed time is more than 15 seconds
                    perceived_outcome = str(False).encode() # Return False
                    return perceived_outcome
                velocity.linear.x = 0.0
                velocity.angular.z = 0.6
                self.cmd_vel_publisher.publish(velocity)

            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self.cmd_vel_publisher.publish(velocity)
            self.ack = True 
            computed_distance = ((self.robot_pose[0] - 0.5)**2 + (self.robot_pose[1] + 0.5)**2)**0.5
            real_distance = ((self.robot_pose[0] - self.marker.poses[0].position.x)**2 + (self.robot_pose[1] - self.marker.poses[0].position.y)**2 + (0.0 - self.marker.poses[0].position.z)**2)**0.5
            if computed_distance - real_distance < 0.5:
                perceived_outcome = str(True).encode()
                self.get_logger().info('Verified action A1')
            else:
                perceived_outcome = str(False).encode()
                self.get_logger().info('Verification of action A1 failed')
            
            
        elif action_ID == 'A2':
            
            self.get_logger().info('Verification of action A2 started')
            
            start_time = time.time()  # Start the timer

            while self.marker is None:
                elapsed_time = time.time() - start_time  # Calculate elapsed time
                if elapsed_time > 100:  # If elapsed time is more than 15 seconds
                    perceived_outcome = str(False).encode() # Return False
                    return perceived_outcome
                velocity.linear.x = 0.0
                velocity.angular.z = 0.6
                self.cmd_vel_publisher.publish(velocity)

            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self.cmd_vel_publisher.publish(velocity)
            self.ack = True
            #compute distance between robot and marker
            computed_distance = ((self.robot_pose[0] - 0.0)**2 + (self.robot_pose[1] - 0.5)**2)**0.5
            real_distance = ((self.robot_pose[0] - self.marker.poses[0].position.x)**2 + (self.robot_pose[1] - self.marker.poses[0].position.y)**2 + (0.0 - self.marker.poses[0].position.z)**2)**0.5
            if computed_distance - real_distance < 0.5:
                perceived_outcome = str(True).encode()
                self.get_logger().info('Verified action A2')
            else:
                perceived_outcome = str(False).encode()
                self.get_logger().info('Verification of action A2 failed')
            
            
        self.marker = None
        return perceived_outcome
            
        
def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.50.204")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    executor = MultiThreadedExecutor()
    turtlebot_adapter = TurtleBotAdapter(args.ip, args.port)
    
    service_request_thread = threading.Thread(target=turtlebot_adapter.handle_service_request)
    service_request_thread.start()

    executor.add_node(turtlebot_adapter)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    # On shutdown..
    turtlebot_adapter.get_logger().info('Shutting down...')
    turtlebot_adapter.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
    
