import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool,SetMode
from rclpy.duration import Duration
from math import sin,cos,pi

class OffboardControlNode(Node):
     
     def __init__(self):
          super().__init__('offb_node_py')

          self.current_state=State()

          self.state_sub=self.create_subscription(State,'mavros/state',self.state_cb,10)

          self.local_pos_pub=self.create_publisher(PoseStamped,'mavros/setpoint_position/local',10)

          self.arming_client=self.create_client(CommandBool,'mavros/cmd/arming')

          self.set_mode_client=self.create_client(SetMode,'mavros/set_mode')

          while not self.arming_client.wait_for_service(timeout_sec=1.0):
              self.get_logger().info('Waiting for arming service')
          
          while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
              self.get_logger().info('Waiting for set_mode service')

          self.pose=PoseStamped()
          self.pose.pose.position.x=0.0
          self.pose.pose.position.y=0.0
          self.pose.pose.position.z=5.0

          self.last_req=self.get_clock().now()

          self.timer=self.create_timer(0.05,self.timer_callback)

          #self.radius=10.0
          #self.Theta=0
          self.dt=0.05

     def state_cb(self,msg):
          self.current_state=msg

     def response_callback(self,future):
           try:
               response=future.result()
               self.get_logger().info(f'Result of MAV_CMD:{response}')
           except Exception as e:
                self.get_logger().error(f'Service call failed:{str(e)}')

     def timer_callback(self):
          if self.current_state.mode !='OFFBOARD' and (self.get_clock().now()-self.last_req)> Duration(seconds=10):
               offb_set_mode=SetMode.Request()
               offb_set_mode.custom_mode='OFFBOARD'
               future=self.set_mode_client.call_async(offb_set_mode)
               future.add_done_callback(self.response_callback)

               self.last_req=self.get_clock().now()

          elif self.current_state.mode=='OFFBOARD':
               if not self.current_state.armed and (self.get_clock().now() - self.last_req)>Duration(seconds=10):
                    arm_cmd=CommandBool.Request()
                    arm_cmd.value=True
                    future=self.arming_client.call_async(arm_cmd)
                    future.add_done_callback(self.response_callback)

                    self.last_req=self.get_clock().now()

          self.pose.pose.position.x+=self.dt
          #self.pose.pose.position.y=self.radius*sin(self.Theta)
          #self.pose.pose.position.z=2.0

          

          self.local_pos_pub.publish(self.pose)

          #self.Theta=self.Theta+self.dt

def main(args=None):
     rclpy.init(args=args)
     node= OffboardControlNode()           

     rclpy.spin(node)

     node.destroy_node()

     rclpy.shutdown()

if __name__=='__main__':
     main()            
                    
          
     
