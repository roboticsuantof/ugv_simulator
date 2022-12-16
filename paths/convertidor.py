#!/usr/bin/python3
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint, trajectory_msgs
from geometry_msgs.msg import Point, PoseStamped, Transform, Quaternion
from nav_msgs.msg import Path
#from rospy_message_converter import message_converter
import yaml
import datetime

global multi_dof_joint_trajectory_point
global path
global path_yaml

hora_full=datetime.datetime.now()
year=hora_full.year
mes=hora_full.month
dia=hora_full.day
hora=hora_full.hour
minuto=hora_full.minute
segundo=hora_full.second
size=0
frame_id='ugv'

multi_dof_joint_trajectory = MultiDOFJointTrajectory()
hola=multi_dof_joint_trajectory.points.reverse
nombre = "ugv_solar"

path = Path()
#path.header
#path_yaml=yaml()

def converter():
    data_multi=MultiDOFJointTrajectory()
    data_multi_reverse=MultiDOFJointTrajectory()
    data_multi=rospy.wait_for_message('/ugv_path',MultiDOFJointTrajectory)
    print("trayectoria recibida")
    #multi_dof_joint_trajectory=data_multi
    size=len(data_multi.points)
    data_multi.points.reverse
    #pose_stamped=PoseStamped()
    #path.header.seq=data_multi.header.seq
    #path.header.frame_id=data_multi.header.frame_id
    #path.header.stamp=data_multi.header.stamp
    header= {'header': 'solar_ugv','seq':1,'stamp':int(str(hora)+str(minuto)+str(segundo)),'frame_id':frame_id,'size':size}
    data={'ugv_solar':header}
    cont=0
    for point in range(size):
        transform=data_multi.points[point].transforms
        translation=transform[0].translation
        rotation=transform[0].rotation
        data['ugv_solar']['poses%d' %(point)]= dict(header='ugv%d' %(point),seq=point,frame_id=frame_id)
        data['ugv_solar']['poses%d' %(point)]['pose']= dict(position={'x':translation.x,'y':translation.y,'z':translation.z},orientation={'x':rotation.x,'y':rotation.y,'z':rotation.z,'w':rotation.w})
        #pose_stamped.pose.position.x = translation.x
        #pose_stamped.pose.position.y = translation.y
        #pose_stamped.pose.position.z = translation.z
        #print("tras_x",translation.x)
        #print("pose_x",pose_stamped.pose.position.x)
        #print("tras_y",translation.y)
        #print("pose_y",pose_stamped.pose.position.y)
        #print(translation.z)
        #print(pose_stamped.pose.position.z)
        #pose_stamped.pose.orientation.x = rotation.x
        #pose_stamped.pose.orientation.y = rotation.y
        #pose_stamped.pose.orientation.z = rotation.z
        #pose_stamped.pose.orientation.w = rotation.w
        #print("rot_x",rotation.x)
        #print("ori_x",pose_stamped.pose.orientation.x)
        #print("rot_y",rotation.y)
        #print("ori_y",pose_stamped.pose.orientation.y)
        #print("rot_y",rotation.w)
        #print("ori_y",pose_stamped.pose.orientation.w)
        #path.poses.append(pose_stamped)
        cont+=1

    with open("tramo_%s_%s_%s_%s%s%s.yaml" %(year,mes,dia,hora,minuto,segundo), "w") as file:
            #file.write(path_yaml)
            yamlFile = yaml.dump(data, file,sort_keys=False)


    #path_yaml = message_converter.convert_ros_message_to_dictionary(path)
    #with open("tramo.yaml", "w") as file:
    #    #file.write(path_yaml)
    #    yamlFile = yaml.dump(path_yaml, file)

def mainros():
    rospy.init_node('Path2yaml', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():  
    hola_str = "esperando datos... %s" % rospy.get_time()
    rospy.loginfo(hola_str)
    aux=converter()
    rate.sleep

if __name__ == '__main__':
    try:
        mainros()
    except rospy.ROSInterruptException:
        pass
