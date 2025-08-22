import rclpy
from rclpy.node import Node
from self_interface.msg import LCMTPosition
from custom_py.robot_control_cmd_lcmt import robot_control_cmd_lcmt
from std_msgs.msg import Float64

import lcm
import toml
import sys
import os
import time


lc=lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
msg=robot_control_cmd_lcmt()


def findAllFile(base):
    for root, ds, fs in os.walk(base):
        for f in fs:
            if f.endswith('.toml'):
                yield  f

def loadtoml():
    base='/home/fsm/src/custom_py/'
    num=0
    filelist=[]
    for i in findAllFile(base):
        filelist.append(i)
        print(str(num)+","+str(filelist[num]))
        num=num+1
    print('Input a toml ctrl file number:')
    numInput=int(input())

    file = os.path.join(base,filelist[numInput])
    print("Load file=%s\n" % file)
    steps = toml.load(file)
    return steps

# 传入一个toml的脚本动作，然后解析成为msg
def send_message(step):
    msg.mode = step['mode']
    msg.value = step['value']
    msg.contact = step['contact']
    msg.gait_id = step['gait_id']
    msg.duration = step['duration']
    print(msg.life_count)
    # if msg.life_count == 127:
    #     msg.life_count = 0
    msg.life_count += 1

    for i in range(3):
        msg.vel_des[i] = step['vel_des'][i]
        msg.rpy_des[i] = step['rpy_des'][i]
        msg.pos_des[i] = step['pos_des'][i]
        msg.acc_des[i] = step['acc_des'][i]
        msg.acc_des[i+3] = step['acc_des'][i+3]
        msg.foot_pose[i] = step['foot_pose'][i]
        msg.ctrl_point[i] = step['ctrl_point'][i]
    for i in range(2):
        msg.step_height[i] = step['step_height'][i]

    lc.publish("robot_control_cmd",msg.encode())
    print('robot_control_cmd lcm publish mode :',msg.mode , "gait_id :",msg.gait_id , "msg.duration=" , msg.duration)
    time.sleep( 0.1 )

def send_message_heart(step):
    msg.mode = step['mode']
    msg.value = step['value']
    msg.contact = step['contact']
    msg.gait_id = step['gait_id']
    msg.duration = step['duration']
    for i in range(3):
        msg.vel_des[i] = step['vel_des'][i]
        msg.rpy_des[i] = step['rpy_des'][i]
        msg.pos_des[i] = step['pos_des'][i]
        msg.acc_des[i] = step['acc_des'][i]
        msg.acc_des[i+3] = step['acc_des'][i+3]
        msg.foot_pose[i] = step['foot_pose'][i]
        msg.ctrl_point[i] = step['ctrl_point'][i]
    for i in range(2):
        msg.step_height[i] = step['step_height'][i]

    lc.publish("robot_control_cmd",msg.encode())
    # print('robot_control_cmd lcm publish mode :',msg.mode , "gait_id :",msg.gait_id , "msg.duration=" , msg.duration)
    time.sleep( 0.1 )


class GaoTaiSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.count = 0
        self.subscription_gaotai = self.create_subscription(LCMTPosition,'/gazebo_position',
            self.gaotai_callback, qos_profile= 10)   
        self.subscription_gaotai = self.create_subscription(Float64,'/heart',
            self.heart_callback, qos_profile= 10)  
        self.step1 = loadtoml()
        self.step2 = loadtoml()
        self.test1 = True
        self.test2 = True
        self.test3 = True

    def heart_callback(self, msg):
        send_message_heart(self.step2['step'][0])


    def gaotai_callback(self, msg_pos):
        x = msg_pos.cydog_pose.position.x
        y = msg_pos.cydog_pose.position.y
        z = msg_pos.cydog_pose.position.z
        yaw = msg_pos.cydog_rpy.z

        if self.count == 0:
            for step in self.step1['step']:
                send_message(step)
            self.count += 1
        
        elif self.count == 1:
            if z > 0.36 and y > 5.74:
                if self.test1 and yaw < 1.54:
                    send_message(self.step2['step'][1])# 这里的目的是让他转正
                    self.test1 = False
                    # 如果转正了，就判断count
                    # 如果count 为一个数就
                elif yaw > 1.54 and (x > 8.7 or x < 8.6) and self.test2:
                    if x > 8.65:
                        send_message(self.step2['step'][2])
                    elif x <= 8.65:
                        send_message(self.step2['step'][6])
                    self.test2 = False
                elif yaw > 1.54 and self.test3 and x < 8.7 and x > 8.6:
                    send_message(self.step2['step'][3])
                    self.test3 = False
                if y > 5.97:
                    self.test1 = True
                    self.test2 = True
                    self.test3 = True
                    self.count += 1
        
        elif self.count == 2:
            for step in self.step1['step']:
                send_message(step)
            self.count += 1

        elif self.count == 3:
            if z > 0.51 and y > 6.5:
                if self.test1 and yaw < 1.54:
                    print(yaw)
                    self.get_logger().info(f'111')
                    send_message(self.step2['step'][1])# 这里的目的是让他转正
                    self.test1 = False
                    # 如果转正了，就判断count
                    # 如果count 为一个数就
                elif yaw > 1.54 and (x > 8.7 or x < 8.6) and self.test2:
                    print(yaw)
                    self.get_logger().info(f'222')
                    if x > 8.65:
                        send_message(self.step2['step'][2])
                    elif x <= 8.65:
                        send_message(self.step2['step'][6])
                    self.test2 = False
                elif yaw > 1.54 and self.test3 and x < 8.7 and x > 8.6:
                    self.get_logger().info(f'333')
                    send_message(self.step2['step'][3])
                    self.test3 = False
                if y > 6.63:
                    self.count += 1
        
        elif self.count == 4:
            send_message(self.step2['step'][4])
            send_message(self.step2['step'][5])
            send_message(self.step2['step'][7])
            send_message(self.step2['step'][4])
            self.count += 1
        
        elif self.count == 5:
            send_message(self.step2['step'][0])
            self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = GaoTaiSubscriber('gaotai')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
