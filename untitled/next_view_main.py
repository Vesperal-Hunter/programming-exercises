#! /usr/bin/env python
# coding: utf8

import rospy
import numpy as np
from util import quaternion_to_radians, quaternion_to_degrees, cylinder, getquaternion
import tf

from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2, Image, JointState
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, Vector3, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray

from recognition_srv_definitions.srv import recognize, recognizeResponse, recognizeRequest
from classifier_srv_definitions.srv import segment_and_classify, segment_and_classifyRequest, segment_and_classifyResponse
from segmentation_srv_definitions.srv import segment, segmentResponse, segmentRequest
from time import sleep
import octomap  # wwoorrkkss!!! Thank you Hongru!
from octomap_msgs.msg import Octomap as octomsg

#The 0.1 number is, for some reason, vitally important - no documentation on why this is so. 
tree = octomap.OcTree(0.1)


ids = None
pointcloud = None
current_pos = None
count = 1
goal = PoseStamped()
goal_tolerance = .1     #percent of error allowed in goal attainment 
pan = None
tilt = None
last_ot = "None"

#The minimum confidence levels for each level of recognition  - currently set at arbitrary numbers
target_conf_instance  = .6 
target_conf_class     = .6  

def got_pose(position):
    global current_pos, goal
    current_pos = position
    if goal.pose.position.x == 0 and goal.pose.position.y == 0 and goal.pose.position.z == 0:
        goal.pose.position = position.position
        goal.pose.orientation = position.orientation
        print "initialised" 


def got_points(initpoints):
    global pointcloud, goal
    pointcloud = initpoints
#    print current_pos.position.x*(1-goal_tolerance)
#    print goal.pose.position.x
#    print current_pos.position.x*(1+goal_tolerance)
#    if abs(current_pos.position.x*(1-goal_tolerance)) <= abs(goal.pose.position.x) <= abs(current_pos.position.x*(1+goal_tolerance)):
#        print "correct location"
    ins_recog()

def got_pantilt(state):
    global pan,tilt
    pan = state.position[0]
    tilt = state.position[1]


#Instance Recognition 
def ins_recog():
    global count,last_ot
    instance_name = '/recognition_service/sv_recognition/'
    rospy.wait_for_service(instance_name)
    recog = rospy.ServiceProxy(instance_name,recognize)

  


    
    
    try: 
        req = recognizeRequest(pointcloud,None,None,None)
        #requests: pointcloud,transform,scene_name,view_name
        #TODO need to check if the args are correct 
        resp = recog(req)
#        print "ID: " + str(resp.ids)
#        print "conf: " + str(resp.confidence)
    except rospy.ServiceException as err:
        print ("Service did not process the request: " + str(err))
    
    id_num = len(resp.ids)
    
    for i in xrange(0,id_num):
        if resp.confidence[i]>target_conf_instance:
            print resp.ids[i]
#            print resp.centroid[i]
#            print count
            count += 1
            
            
            #may need to initialise this as a vector first
            #These may need to be switched around- was done in a rush
            #
            
            #transformation to map coordinates from camera             
            distance = resp.centroid[i]
            orientation = quaternion_to_radians(current_pos.orientation)

#                  
            obj_location = PointStamped()
            obj_location.header.frame_id = "map"
#            obj_location.point = resp.centroid[i]
#            print "obj_location, theirs: " + str(obj_location.point)
           
           
           
           
           
       #Octree part
            #determines if the octree needs to be reloaded   
            file_id = str(resp.ids[i])
#            file_id = "paper_cup"
            if file_id != last_ot:
                last_ot = file_id
                filename = "/home/chrismcg/models/models/"+ file_id[6:len(file_id)] +"/3D_model.bt"
                if tree.readBinary(filename):
                    print "Octree imported successfully from ", filename
                else:
                    print "Import Error, no model loaded."           
            
              
            #casts a ray and returns if it hits a node
#            for i in xrange(1,10):
#                first = tree.castRay(np.array([0,0,0],np.double),np.array([i,i+1,i+2],np.double),np.array([1,1,1],np.double),1,1)
#                if first:
#                    raycount +=1
#            print "raycount: " + str(raycount)
#            print "i: " + str(i)

            
            
            #Determines the location of the object in map coordinates 
            
            pan_adjust = np.rad2deg(orientation) + np.rad2deg(pan)
            if pan_adjust<-180 or pan_adjust>180:
                diff = np.abs(pan_adjust)-180
                newpan = 180-diff
                if pan_adjust > 0:
                    pan_adjust = np.deg2rad(-newpan)
                else:
                    pan_adjust = np.deg2rad(newpan)
            else:
                pan_adjust = np.deg2rad(pan_adjust)
           lookupTransform
            obj_location.point.x = current_pos.position.x + (np.cos(pan_adjust) * distance.z)
            obj_location.point.y = current_pos.position.y + (np.sin(pan_adjust) * distance.z)
            obj_location.point.z = 1.6 - (np.sin(tilt)*distance.z)
            # TODO: ALSO NEED TO OFFSET THIS BY THE X/Y POSITIONS, AS THEY ARE NOT ALWAYS IN THE CENTRE OF THE CAMERA#####
            pub2.publish(obj_location)
            
            
            change = tf.TransformBroadcaster()
            
            posx = current_pos.position.x + (np.cos(pan_adjust) * distance.z)
            posy = current_pos.position.y + (np.sin(pan_adjust) * distance.z)
            posz = 1.6 - (np.sin(tilt)*distance.z)
            
            quat = [current_pos.orientation.x,current_pos.orientation.y,current_pos.orientation.z,current_pos.orientation.w]
            orien = tf.transformations.euler_from_quaternion(quat)
            eul = tf.transformations.quaternion_from_euler(orien[0],orien[1],orien[2])
            
            change.sendTransform((posx,posy,posz),eul,rospy.Time.now(),"camera","map")

            
            
            #Publishes cirles around the object in rviz
            [next_cand_x,next_cand_y,next_cand_z] = cylinder(1.5) #specify the size of the area. 
   
            views = PoseArray()
            views.header.frame_id = "map"    
            for iter1 in xrange(0,len(next_cand_x)-1):
                for iter2 in xrange(0,9):
                    p=Pose()
                    
                    posx = next_cand_x[iter1,iter2]
                    posy = next_cand_y[iter1,iter2] 
                    
                    p.position.x = posx + obj_location.point.x
                    
                    p.position.y = next_cand_y[iter1,iter2] + obj_location.point.y
                    
                    p.position.z = 0
                    
                    hyp = (np.sqrt(posx**2+posy**2))
                    cosA = posy/hyp 

                    if posx == 0 and posy == 0:
                        cosA = 0
                    else:
                        if posx > 0:
                            cosA =  np.deg2rad(270) -np.arccos(cosA)     
                        else:
                            cosA =  np.deg2rad(90) -np.arccos(-cosA)
       
                   
                    p.orientation = getquaternion(cosA)
                    
                    views.poses.append(p)                    
                    
            pubviews.publish(views)
            

            
#=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#Publishes the goal location for the next view
    goal = PoseStamped()
    
    goal.header.frame_id = "map"    
    
    goal.pose.position.x = current_pos.position.x
    goal.pose.position.y = current_pos.position.y
    goal.pose.position.z = current_pos.position.z
    
    #This works - returns the correct radian 

    goal.pose.orientation = getquaternion(108)
#    goal.pose.orientation = current_pos.orientation
    
#    pub.publish(goal)
#    print ("current position: " + str(current_pos.position.x))
#    print ("new position: " + str(goal.pose.position.x))
    
#    goal.pose.orientation.x = 0#current_pos.orientation.x 
#    goal.pose.orientation.y = 0#current_pos.orientation.y 
#    goal.pose.orientation.z = 0.5 #current_pos.orientation.z 
#    goal.pose.orientation.w = current_pos.orientation.w 
    

    


    movetilt = Vector3()
    movetilt.y = np.deg2rad(0)
    # y = pan
    movetilt.z = np.deg2rad(0)
    # z = tilt
    tiltpub.publish(movetilt)
    
    
     
    

    #TODO need to set up section to determine when to move onto class recog. 

#Segmentation Part - segmented indices are then passed to the classification service

def seg():
    seg_name = '/pcl_segmentation_service/pcl_segmentation'    
    rospy.wait_for_service(seg_name)
    seg = rospy.ServiceProxy(seg_name,segment)
    try:
        req = segmentRequest(pointcloud,None)  
        cluster_indices = seg(req)
    except rospy.ServiceException as err:
        print ("Service did not process the request: " + str(err))
    print cluster_indices





#Class Recognition

#TODO add the segment and classify parts to this and the parts which passes to the classifier if the instance does not work. 

def class_recog():
    
    classifier_name='/classifier_service/classify/'

    rospy.wait_for_service(classifier_name)
    classify = rospy.ServiceProxy(classifier_name,segment_and_classify)

    try:
        req = segment_and_classifyRequest(pointcloud)
        #requests: pointcloud,transform
        resp = classify(req)
    except rospy.ServiceException as err:
        print ("Service did not process the request: " + str(err))

        #TODO need to set up section to determine when to move onto occlussion recog 



#Octree analysis 




if __name__ == "__main__":
    rospy.init_node('next_view_main')
    rospy.Subscriber('/robot_pose',Pose,got_pose)
    rospy.Subscriber('/camera/depth_registered/points',PointCloud2,got_points)
    rospy.Subscriber('/ptu/state',JointState,got_pantilt)
    pub = rospy.Publisher("move_base_simple/goal",PoseStamped,queue_size=100)
    pub2 = rospy.Publisher("obj_location",PointStamped,queue_size=100)
    tiltpub = rospy.Publisher("ptu",Vector3,queue_size=100)
    pubviews = rospy.Publisher("viewcands",PoseArray,queue_size=100)
    octopub = rospy.Publisher("octrees",octomsg,queue_size=100)
    markpub = rospy.Publisher("marks",MarkerArray,queue_size=100)
    rospy.spin()    



















