import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
# from std_msgs.msg import String

from vosk import Model, KaldiRecognizer
import pyaudio
import json
import subprocess
import time

model = Model(r"/home/ervinpicardal/python_ws/voice_speech_detection/vosk-model-en")
recognizer = KaldiRecognizer(model, 16000)
mic = pyaudio.PyAudio()
stream = mic.open(format=pyaudio.paInt16, channels=1,
                     rate=16000, input=True, frames_per_buffer=8192)
sound_files_path = '/home/ervinpicardal/robot_ws/src/arns_voice_cmd/resource'

class Voice_CMD_Node(Node):
   def __init__(self):
      super().__init__("voice_cmd")
      self.timeout = 10
      stream.start_stream()
      self.wake_up=False
      self.goal_pose_publisher_ = self.create_publisher(PoseStamped, "/goal_pose", 10)
      self.cmd_vel_publisher_ = self.create_publisher(Twist, "/diff_cont/cmd_vel_unstamped", 10)
      timer_period = 0.0333333333
      self.timer = self.create_timer(timer_period, self.voice_cmd)
      
   def voice_cmd(self): 
      voice_data = self.record_audio()
      self.respond(voice_data)
   
   def record_audio(self, ask=False):
      audio = stream.read(4096)
      voice_data = ''
      if recognizer.AcceptWaveform(audio):
         voice_data = json.loads(recognizer.Result())['text']
         print(voice_data)

      return voice_data

   def respond(self, voice_data):
      if not self.wake_up:
         if 'alexa' in voice_data.lower():
            self.text2speech(f"{sound_files_path}/wake_up.mp3")
            self.wake_up = True
      else:
         if 'kitchen' in voice_data.lower():
            self.text2speech(f"{sound_files_path}/kitchen.mp3")
            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = 0.69
            goal_pose.pose.position.y = 0.66
            self.goal_pose_publisher_.publish(goal_pose)
            self.wake_up = False
         elif 'living room' in voice_data.lower():
            pass
         

   def text2speech(self, file):
      cmd = "mpg123 " + file
      p = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
      (output, err) = p.communicate()
      return output



def main(args=None): 
   rclpy.init(args=args)
   
   test_node = Voice_CMD_Node()
   
   # test_node.text2speech("How can I help you?")
   
   rclpy.spin(test_node)
   
   
   test_node.destroy_node()
   rclpy.shutdown()
   stream.stop_stream()
   stream.close()
   mic.terminate()

if __name__ == "__main__":
   main()