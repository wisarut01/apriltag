#!/usr/bin/env python3
import rospy
import smach
import smach_ros
import tf
from mavros_msgs.srv import SetMode, SetModeRequest
from precision_landing.srv import SelectTag, SelectTagRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

#define state Start
class Start(smach.State):
    def __init__(self,outcomes=['Big_Boxes_Land_Start','Hold']):
        super().__init__(outcomes)
        rospy.loginfo('Executing state START')
        self.change_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)

    def execute(self,userdata):
        mode = SetModeRequest()
        mode.custom_mode= 'GUIDED'
        self.change_mode(mode)
        return 'Big_Boxes_Land_Start'

#define state Big_Boxes_Landing
class Big_Boxes_Landing(smach.State):
    def __init__(self, outcomes=['Small_Boxes_Land','Hold']):
        super().__init__(outcomes)
        rospy.loginfo('Executing state Big Boxes Landing')
        self.position_subscribe = rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.Get_Altitude)
        self._angle_subscribe = rospy.Subscriber('/mavros/imu/data',Imu,self.Get_Angle)
        self.select_bundle = rospy.ServiceProxy('/apriltag/tag_select',SelectTag)
        self.position = None
        self.roll = None
        self.pitch = None
        rospy.spin()

    def Get_Angle(self,data):
        rpy = tf.transformations.euler_from_quaternion(data)
        self.roll = rpy[0]
        self.pitch = rpy[1]

    def Get_Altitude(self,data):
        self.position = data.pose.position
    
    def execute(self,userdata):
        select_bundle_param = SelectTagRequest()
        select_bundle_param.tag_no = 'box_bundle_1'
        control_respone.sucess = False
        while (not control_respone.sucess):
            if self.roll <= 5 and self.pitch <= 5:
                select_bundle_param.altitude = -self.position.z*0.5
                control_respone = self.select_bundle(select_bundle_param) 
            else: 
                print('Angle of your drone exceed to 5 deg')
                return 'Hold'
        return 'Small_Boxes_Land'
        
#define state Small_Boxes_Landing
class Small_Boxes_Landing(smach.State):
    def __init__(self, outcomes=['Land_Mode','Hold']):
        super().__init__(outcomes)

    def Get_Angle(self,data):
        rpy = tf.transformations.euler_from_quaternion(data)
        self.roll = rpy[0]
        self.pitch = rpy[1]

    def Get_Altitude(self,data):
        self.position = data.pose.position
    
    def execute(self,userdata):
        select_bundle_param = SelectTagRequest()
        select_bundle_param.tag_no = 'box_bundle_2'
        control_respone.sucess = False
        while (not control_respone.sucess):
            if self.roll <= 5 and self.pitch <= 5:
                select_bundle_param.altitude = -self.position.z*0.5
                control_respone = self.select_bundle(select_bundle_param) 
            else: 
                print('Angle of your drone exceed to 5 deg')
                return 'Hold'
        return 'Land_Mode'

#define state Set_Land_mode
class Set_Land_Mode(smach.State):
    def __init__(self, outcomes=['Landed']):
        super().__init__(outcomes) 
        rospy.loginfo('Executing state Set Land Mode')
        self.change_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)

    def execute(self, userdata):
        mode = SetModeRequest()
        mode.custom_mode = 'Land'
        self.change_mode(mode)
        return 'Landed'
     
#main 
if __name__ == '__main__':
    rospy.init_node('Precision_landing',anonymous=True)

    #create SMACH state machine 
    sm = smach.StateMachine(outcomes=['FINISH'])

    #Open the container and add states to the container 
    with sm: 
        smach.StateMachine.add('START', Start(),
                                transitions={'Big_Boxes_Land_Start':'BIG_BOXES_LANDING','Hold':'START'})
        smach.StateMachine.add('BIG_BOXES_LANDING',Big_Boxes_Landing(),
                                transitions={'Small_Boxes_Land':'SMALL_BOXES_LANDING','Hold':'BIG_BOXES_LANDING'})
        smach.StateMachine.add('SMALL_BOXES_LANDING',Small_Boxes_Landing(),
                                transitions={'Land_Mode':'SET_LAND_MODE','Hold':'SMALL_BOXES_LANDING'})
        smach.StateMachine.add('SET_LAND_MODE',Set_Land_Mode(),
                                transitions={'Landed':'FINISH'})