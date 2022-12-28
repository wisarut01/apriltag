#!/usr/bin/env python3
import rospy
import smach
import smach_ros
import tf
import time
from mavros_msgs.srv import SetMode, SetModeRequest
from precision_landing.srv import SelectTag, SelectTagRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import RCIn

#define state Start
class START(smach.State):
    def __init__(self,outcomes=['big_boxes_land','hold']):
        super().__init__(outcomes)
        rospy.loginfo('Executing state START')
        self.ch_8_pwm = rospy.Subscriber('/mavros/rc/in',RCIn,self.toggle_switch)
        self.change_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)
        self.enable = False
               
    def execute(self,userdata):
        time.sleep(1)
        change_service = SetModeRequest()
        change_service.custom_mode= 'GUIDED'
        if self.enable == False:
            return 'hold'
        self.change_mode(change_service)
        return 'big_boxes_land'

    def toggle_switch(self,data):
        if data.channels[7] >= 1500:
            self.enable = True


#define state Big_Boxes_Landing
class BIG_BOXES_LANDING(smach.State):
    def __init__(self, outcomes=['small_boxes_land','hold']):
        super().__init__(outcomes)
        rospy.loginfo('Executing state Big Boxes Landing')
        self.select_bundle = rospy.ServiceProxy('/apriltag/tag_select',SelectTag)
        self.position_subscribe = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, callback=self.get_Altitude)
        self.angle_subscribe = rospy.Subscriber('/mavros/imu/data',Imu,callback=self.get_Angle)
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.position = None
        self.orientation = None
        time.sleep(1)
     
    def get_Altitude(self, data):
        self.position = data.pose.position
        self.orientation = data.pose.orientation

    def get_Angle(self,data):
        qua = [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
        roll , pitch ,yaw = tf.transformations.euler_from_quaternion(qua)
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def execute(self,userdata):
        select_bundle_param = SelectTagRequest()
        select_bundle_param.tag_no = 'box_bundle_1'
        select_bundle_param.altitude = 0.0
        print('send request1')
        print('altitude send = ', select_bundle_param.altitude)
        control_respone = self.select_bundle(select_bundle_param) 
        while (not control_respone.sucess):
            if self.roll <= 3 and self.pitch <= 3:
                # a = input('you want to continue?')
                print('send request2')
                select_bundle_param.altitude = -self.position.z * 0.5
                print('altitude send = ', select_bundle_param.altitude)
                control_respone = self.select_bundle(select_bundle_param) 
            else: 
                print('Angle of your drone exceed to 5 deg')
                return 'hold'
        return 'small_boxes_land'
        
#define state Small_Boxes_Landing
class SMALL_BOXES_LANDING(smach.State):
    def __init__(self, outcomes=['land_mode','hold']):
        super().__init__(outcomes)
        rospy.loginfo('Executing state Small Boxes Landing')
        self.select_bundle = rospy.ServiceProxy('/apriltag/tag_select',SelectTag)
        self.position_subscribe = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, callback=self.get_Altitude)
        self.angle_subscribe = rospy.Subscriber('/mavros/imu/data',Imu,callback=self.get_Angle)
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.position = None
        self.orientation = None
        time.sleep(1)
     
    def get_Altitude(self, data):
        self.position = data.pose.position
        self.orientation = data.pose.orientation

    def get_Angle(self,data):
        qua = [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
        roll , pitch ,yaw = tf.transformations.euler_from_quaternion(qua)
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def execute(self,userdata):
        select_bundle_param = SelectTagRequest()
        select_bundle_param.tag_no = 'box_bundle_2'    #change when real test 
        select_bundle_param.altitude = 0.0
        print('send request1')
        print('altitude send = ', select_bundle_param.altitude)
        control_respone = self.select_bundle(select_bundle_param) 
        while (not control_respone.sucess):
            if self.roll <= 3 and self.pitch <= 3:
                # a = input('you want to continue?')
                print('send request2')
                select_bundle_param.altitude = -self.position.z * 0.5
                print('altitude send = ', select_bundle_param.altitude)
                control_respone = self.select_bundle(select_bundle_param) 
            else: 
                print('Angle of your drone exceed to 5 deg')
                return 'hold'
        return 'land_mode'

#define state Set_Land_mode
class SET_LAND_MODE(smach.State):
    def __init__(self, outcomes=['landed']):
        super().__init__(outcomes) 
        rospy.loginfo('Executing state Set Land Mode')
        self.change_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)

    def execute(self, userdata):
        mode = SetModeRequest()
        mode.custom_mode = 'Land'
        self.change_mode(mode)
        return 'landed'
     
#main 
if __name__ == '__main__':
    rospy.init_node('precision_landing',anonymous=True)
    #create SMACH state machine 
    sm = smach.StateMachine(outcomes=['FINISH'])
    #Open the container and add states to the container 
    with sm: 
        smach.StateMachine.add('START', START(),
                                transitions={'big_boxes_land':'BIG_BOXES_LANDING','hold':'START'})
        smach.StateMachine.add('BIG_BOXES_LANDING',BIG_BOXES_LANDING(),
                                transitions={'small_boxes_land':'SMALL_BOXES_LANDING','hold':'BIG_BOXES_LANDING'})
        smach.StateMachine.add('SMALL_BOXES_LANDING',SMALL_BOXES_LANDING(),
                                transitions={'land_mode':'SET_LAND_MODE','hold':'SMALL_BOXES_LANDING'})
        smach.StateMachine.add('SET_LAND_MODE',SET_LAND_MODE(),
                                transitions={'landed':'FINISH'})
    outcome = sm.execute()
