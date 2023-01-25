#!/usr/bin/python3
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint, trajectory_msgs
from geometry_msgs.msg import Point, PoseStamped, Transform, Quaternion
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler, quaternion_multiply
#from rospy_message_converter import message_converter
import yaml
import datetime
import math

global multi_dof_joint_trajectory_point
global path
global path_yaml
global optimizado

hora_full=datetime.datetime.now()
year=hora_full.year
mes=hora_full.month
dia=hora_full.day
hora=hora_full.hour
minuto=hora_full.minute
segundo=hora_full.second
size=0
frame_id='ugv'
optimizado =False
distanciaLimite =1
multi_dof_joint_trajectory = MultiDOFJointTrajectory()
hola=multi_dof_joint_trajectory.points.reverse
nombre = "ugv_solar"

path = Path()
#path.header
#path_yaml=yaml()

def calculate_quaternion(x1, y1, x2, y2):
    # Calcula el ángulo de rotación entre el punto anterior(x1, y1) y el punto actual (x2, y2)
    yaw = math.atan2(y2 - y1, x2 - x1)

    # Crea un cuaternio a partir del ángulo de rotación
    quat = quaternion_from_euler(0, 0, yaw)

    # Crea un mensaje Quaternion con los valores del cuaternio
    quaternion_msg = Quaternion()
    quaternion_msg.x = float(quat[0])
    quaternion_msg.y = float(quat[1])
    quaternion_msg.z = float(quat[2])
    quaternion_msg.w = float(quat[3])

    return quaternion_msg


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
    cont=1
    transform=data_multi.points[0].transforms
    translation=transform[0].translation
    rotation=transform[0].rotation
    x=translation.x
    y=translation.y
    data['ugv_solar']['poses0']= dict(header='ugv0',seq=0,frame_id=frame_id)
    data['ugv_solar']['poses0']['pose']= dict(position={'x':translation.x,'y':translation.y,'z':translation.z},orientation={'x':rotation.x,'y':rotation.y,'z':rotation.z,'w':rotation.w})
    x_ant=x
    y_ant=y
    for point in range(size):
        #optimizado=True
        transform=data_multi.points[point].transforms
        translation=transform[0].translation
        rotation=transform[0].rotation
        x=translation.x
        y=translation.y
        if(optimizado):
            print("optimizado")
            if(point in range(1,(size-1))):
                print("punto",point)
                distancia=math.sqrt((x-x_ant)**2+(y-y_ant)**2)
                print("distancia",distancia)
                if(distancia >= distanciaLimite):
                    quaternion = calculate_quaternion(x, y, x_ant, y_ant)
                    print("punto agregado")
                    data['ugv_solar']['poses%d' %(cont)]= dict(header='ugv%d' %(cont),seq=cont,frame_id=frame_id)
                    data['ugv_solar']['poses%d' %(cont)]['pose']= dict(position={'x':translation.x,'y':translation.y,'z':translation.z},orientation={'x':quaternion.x,'y':quaternion.y,'z':quaternion.z,'w':quaternion.w})
                    data['ugv_solar']['poses%d' %(cont - 1)]['pose']['orientation']['x']=quaternion.x
                    data['ugv_solar']['poses%d' %(cont - 1)]['pose']['orientation']['y']=quaternion.y
                    data['ugv_solar']['poses%d' %(cont - 1)]['pose']['orientation']['z']=quaternion.z
                    data['ugv_solar']['poses%d' %(cont - 1)]['pose']['orientation']['w']=quaternion.w
                    cont+=1  
                    x_ant=x
                    y_ant=y
            elif(point == size):
                    data['ugv_solar']['poses%d' %(cont)]= dict(header='ugv%d' %(cont),seq=cont,frame_id=frame_id)
                    data['ugv_solar']['poses%d' %(cont)]['pose']= dict(position={'x':translation.x,'y':translation.y,'z':translation.z},orientation={'x':rotation.x,'y':rotation.y,'z':rotation.z,'w':rotation.w})
                    cont+=1 
        else:
            data['ugv_solar']['poses%d' %(point)]= dict(header='ugv%d' %(point),seq=point,frame_id=frame_id)
            data['ugv_solar']['poses%d' %(point)]['pose']= dict(position={'x':translation.x,'y':translation.y,'z':translation.z},orientation={'x':rotation.x,'y':rotation.y,'z':rotation.z,'w':rotation.w})
            cont+=1
    header['size'] = cont      


    if(optimizado):

        with open("optimizado_tramo_%s_%s_%s_%s%s%s.yaml" %(year,mes,dia,hora,minuto,segundo), "w") as file:
                #file.write(path_yaml)
                yamlFile = yaml.dump(data, file,sort_keys=False)
    else:
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
