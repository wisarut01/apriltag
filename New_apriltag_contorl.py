#!/usr/bin/env python3
import rospy 
import tf
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from geographic_msgs.msg import GeoPoseStamped
from precision_landing.srv import SelectTag,SelectTagResponse 

#define class apriltagcontrol 
class ApriltagControl(object):
    def __init__(self):
        rospy.init_node('ApriltagControl',anonymous=True)
        self.select_bundle = rospy.Service('/apriltag/tag_select',SelectTag,self.Go_To_Tags)
        self.transformation_listener = tf.TransformListener()
        self.position_target_publish = rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,self.Setpoint_Publish)
        self.new_GPS_data_recovery = rospy.Publisher('/mavros/setpoint_position/global',GeoPoseStamped,self.Go_To_Tags)
        self.position_subscribe = rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.Get_Position)
        self.rate = rospy.Rate(10)
        self.process_time = 3.0 
        self.position = None
        self.orientation = None 
        rospy.spin()

    #define get_position function 
    def Get_Position(self,data):
        self.pose_z = data.pose.position.z

    #define new_GPS_data function 
    def New_GPS_Data(self,position):
        new_GPS_data = GeoPoseStamped()
        new_GPS_data.pose.position.latitude = position[0]
        new_GPS_data.pose.position.longitude = position[1]
        new_GPS_data.pose.position.altitude = position[2]
        new_GPS_data.pose.orientation.x = 0
        new_GPS_data.pose.orientation.y = 0
        new_GPS_data.pose.orientation.z = 0
        new_GPS_data.pose.orientation.w = 1
        #publish GPS position recovery to vechicle
        self.new_GPS_data_recovery.publish(new_GPS_data)
        self.rate.sleep()

    #define setpoint_publish function 
    def Setpoint_Publish(self,pose,orientation,velocity,accel):
        position_target = PositionTarget()
        position_target.coordinate_frame = 9
        position_target.type_mask = 2048
        position_target.position.x = pose[0]
        position_target.position.y = pose[1]
        position_target.position.z = pose[2] 
        position_target.velocity.x = velocity[0]
        position_target.velocity.y = velocity[1]
        position_target.velocity.z = velocity[2]
        position_target.acceleration_or_force.x = accel[0]
        position_target.acceleration_or_force.y = accel[1]
        position_target.acceleration_or_force.z = accel[2]
        euler_yaw = tf.transformations.euler_from_quaternion(orientation)
        position_target.yaw = -euler_yaw[2]
        #publish position targrt to vechicle
        self.position_target_publish.publish(position_target)
        self.rate.sleep()

    #define Go_To_Tags function 
    def Go_To_Tags(self,data):
        bundle_no = '/' + data.tag_no
        transition = None
        rotation = None
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                self.transformation_listener.waitForTransform(target_frame='/usb_cam',source_frame=bundle_no,time=now,timeout=rospy.Duration(3.0))
                (transition,rotation) = self.transformation_listener.lookupTransform(target_frame='/usb_cam',source_frame=bundle_no,time=now)
                if transition == None or rotation == None : 
                    if data.altitude >=50 : 
                        print("Can't see any tags, Please Input New GPS Data")
                        lat = float(input('Input New latitude: '))
                        long = float(input('Input New longitude: '))
                        pose = [lat,long,data.altitude]
                        self.New_GPS_Data(pose)
                        return SelectTagResponse(False)
                    else:
                        print("Can't see any tags, increase altitude ")
                        altitude_recovery = data.altitude*2 #not sure direction 
                        pose = [0,0,altitude_recovery]
                        orientation = [0,0,0,1]
                        velocity = [0,0,(altitude_recovery-data.altitude)/self.process_time]
                        accel = [0,0,velocity[2]/self.process_time]  
                        self.Setpoint_Publish(pose,orientation,velocity,accel)        
                        return SelectTagResponse(False)
                else:
                    pose = [-transition[0],-transition[1],data.altitude]
                    orientation = [rotation[0],rotation[1],rotation[2],rotation[3]]
                    velocity = [transition[0]/self.process_time,transition[1]/self.process_time,transition[2]/self.process_time]
                    accel = [velocity[0]/self.process_time,velocity[1]/self.process_time,velocity[2/self.process_time]]
                    self.Setpoint_Publish(pose,orientation,velocity,accel)       
                    if self.pose_z <= 5 : 
                        return SelectTagResponse(True)
                    else:
                        return SelectTagResponse(False)
            except Exception as e:
                print(e)
                continue

#main
if __name__ == '__main__':
    try :
        ApriltagControl()
    except rospy.ROSInternalException:
        pass

