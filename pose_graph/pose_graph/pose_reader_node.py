import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
import atexit

class PoseReaderNode(Node):
   def __init__(self):
      super().__init__("pose_reader_node")
      self.time = 0
      self.prev_time = 0
      # self.expected_trans_x = 0
      # self.expected_trans_y = 0
      # self.expected_rot_z = 0
      # self.expected_rot_w = 0
      
      self.estimated_pose = pd.DataFrame({
         "time": [],
         "Estimated x-position": [], 
         "Estimated y-position": [], 
         "Estimated z-rotation": [],
      })
      
      self.goal_pose = pd.DataFrame({
         "Goal x-position": [],
         "Goal y-position": [],
         "Goal z-rotation": [],
      })
      
      self.goal_pose_sub_ = self.create_subscription(
         PoseStamped, 
         "/goal_pose",
         self.goal_pose_callback,
         10
      )
      
      self.amcl_pose_sub_ = self.create_subscription(
         PoseWithCovarianceStamped,
         "/amcl_pose",
         self.amcl_pose_callback,
         10
      )
      self.goal_pose_sub_
      self.amcl_pose_sub_
      
      atexit.register(self.save_data)
   
   def goal_pose_callback(self, msg: PoseStamped):
      theta = math.acos(msg.pose.orientation.w)*2
      self.goal_pose.loc[0] = [
         msg.pose.position.x, 
         msg.pose.position.y, 
         theta,
      ]
      self.get_logger().info(f'Goal Pose: [{msg.pose.position.x},{msg.pose.position.y},{theta}]')

   def amcl_pose_callback(self, data: PoseWithCovarianceStamped):
      if(self.prev_time > 0 ):
         dt = data.header.stamp.sec - self.prev_time
         self.time +=  dt
         
      theta = math.acos(data.pose.pose.orientation.w)*2
      self.estimated_pose.loc[-1] = [
         self.time,
         data.pose.pose.position.x,
         data.pose.pose.position.y,
         theta
      ]
      self.prev_time = data.header.stamp.sec
      self.estimated_pose.index = self.estimated_pose.index + 1
      self.estimated_pose = self.estimated_pose.sort_index()
      
      self.get_logger().info(f'AMCL Pose: [{self.time},{data.pose.pose.position.x},{data.pose.pose.position.y},{theta}]')

   def save_data(self):
      # print(self.estimated_pose)
      # print("\n\n")
      # print(self.expected_pose)
      
      df = pd.concat([self.estimated_pose, self.goal_pose], axis=1)
      time = df['time'].to_numpy()
      
      estimated_x_position = df['Estimated x-position'].to_numpy()
      estimated_y_position = df['Estimated y-position'].to_numpy()
      goal_x_position = df['Goal x-position'].to_numpy()[0]
      goal_y_position = df['Goal y-position'].to_numpy()[0]
      
      estimated_z_rotation = df['Estimated z-rotation'].to_numpy()
      goal_z_rotation = df['Goal z-rotation'].to_numpy()[0]
      
      fig, axs = plt.subplots(2,1)
      axs[0].plot(time, estimated_x_position, label="Estimated Position in x-axis", color='r')
      axs[0].plot(time, estimated_y_position, label="Estimated Position in y-axis", color='b')
      axs[0].axhline(y=goal_x_position, label="Goal Position in x-axis", color='r', linestyle='--')
      axs[0].axhline(y=goal_y_position, label="Goal Position in y-axis", color='b', linestyle='--')
      axs[0].set_xlabel("Time")
      axs[0].set_ylabel("Meter/s")
      axs[0].set_title("Estimated Position vs Time")
      axs[0].legend()

      axs[1].plot(time, estimated_z_rotation, label="Estimated Angle of Rotation along z-axis", color='g')
      axs[1].axhline(y=goal_z_rotation, label="Goal Angle of Rotation along z-axis", color='g', linestyle='--')
      axs[1].set_xlabel("Time")
      axs[1].set_ylabel("Radian/s")
      axs[1].set_title("Estimated Orientation vs Time")
      axs[1].legend()
      
   
      
      # plt.plot(time, m_pose_x, label="Estimated Position in x-axis", color='r')
      # plt.plot(time, m_pose_y, label="Estimated Position in y-axis", color='b')
      # plt.axhline(y=goal_x_position, label="Goal Position in x-axis", color='r', linestyle='--')
      # plt.axhline(y=goal_y_position, label="Goal Position in y-axis", color='b', linestyle='--')
      # plt.xlabel("Time")
      # plt.ylabel("Meter/s")
      # plt.legend()
      plt.show()
      
      
      # directory = "/home/ervinpicardal/dev_ws/src/pose_graph/data"

      # df.to_csv("/home/ervinpicardal/dev_ws/src/pose_graph/data/pose.csv", encoding="utf-8", index=False)


def main(args=None):
   rclpy.init(args=args)
   
   # create node
   pose_reader_node = PoseReaderNode()
   
   # spin node
   rclpy.spin(pose_reader_node)
   
   pose_reader_node.destroy_node()
   rclpy.shutdown()
   
if __name__ == "__main__":
   main()