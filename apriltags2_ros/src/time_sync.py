#!/usr/bin/env python
import rospy
import rosbag
from threading import Lock
from shutil import copy

class ToLocalPose:
    def __init__(self):
        """
        listens to pose estimation returned by apriltag2_ros node and converts it
        into robot pose expressed in the global frame
        """
        host_package = rospy.get_namespace() # as defined by <group> in launch file
        self.node_name = 'publish_detections_in_local_frame' # node name , as defined in launch file
        host_package_node = host_package + self.node_name
        self.veh = host_package.split('/')[1]

        # initialize the node
        rospy.init_node('publish_detections_in_local_frame_node', anonymous=False)

        # Parameters
        operation_mode = rospy.get_param(param_name= "/operation_mode")

        self.lock = Lock()
        self.lock.acquire()
        copy(input_bag, self.output_bag)
        self.lock.release()


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbDetection(self,msg):
        if(len(msg.detections)>0): # non-emtpy detection message
            # unpack the position and orientation returned by apriltags2 ros
            t_msg = msg.detections[0].pose.pose.pose.position
            q_msg = msg.detections[0].pose.pose.pose.orientation

            # convert the message content into a numpy array as robot_pose_in_world_frame requires so.
            t = np.array([t_msg.x, t_msg.y, t_msg.z])
            q = np.array([q_msg.x, q_msg.y, q_msg.z, q_msg.w])

            # express relative rotation of the robot wrt the global frame.
            veh_R_world, veh_t_world = robot_pose_in_word_frame(q,t)
            veh_feaXYZ_world = rotation_matrix_to_euler(veh_R_world)

            # convert from numpy float to standart python float to be written into the message
            veh_t_world = veh_t_world.tolist()
            veh_feaXYZ_world = veh_feaXYZ_world.tolist()

            # form message to publish
            veh_pose_euler_msg = VehiclePoseEuler()
            veh_pose_euler_msg.header.stamp = rospy.Time.now()
            # position
            veh_pose_euler_msg.posx = veh_t_world[0]
            veh_pose_euler_msg.posy = veh_t_world[1]
            veh_pose_euler_msg.posz = veh_t_world[2]
            # orientation
            veh_pose_euler_msg.rotx = veh_feaXYZ_world[0]
            veh_pose_euler_msg.roty = veh_feaXYZ_world[1]
            veh_pose_euler_msg.rotz = veh_feaXYZ_world[2]
            # finally publish the message
            self.pub_detection_in_robot_frame.publish(veh_pose_euler_msg)

    def cbDetectionSync(self,msg):
        print "cbDetectionSync Called"

        # Check that there are still images to process
        param_name = "/" + self.veh + "/buffer_node/process_status"
        param_val  = rospy.get_param(param_name=param_name)

        if param_val != 1:
            if(len(msg.detections)>0): # non-emtpy detection message
                # unpack the position and orientation returned by apriltags2 ros
                t_msg = msg.detections[0].pose.pose.pose.position
                q_msg = msg.detections[0].pose.pose.pose.orientation

                # convert the message content into a numpy array as robot_pose_in_world_frame requires so.
                t = np.array([t_msg.x, t_msg.y, t_msg.z])
                q = np.array([q_msg.x, q_msg.y, q_msg.z, q_msg.w])

                # express relative rotation of the robot wrt the global frame.
                veh_R_world, veh_t_world = robot_pose_in_word_frame(q,t)
                veh_feaXYZ_world = rotation_matrix_to_euler(veh_R_world)

                # convert from numpy float to standart python float to be written into the message
                veh_t_world = veh_t_world.tolist()
                veh_feaXYZ_world = veh_feaXYZ_world.tolist()

                # form message to publish
                veh_pose_euler_msg = VehiclePoseEuler()
                veh_pose_euler_msg.header.stamp = rospy.Time.now()
                # position
                veh_pose_euler_msg.posx = veh_t_world[0]
                veh_pose_euler_msg.posy = veh_t_world[1]
                veh_pose_euler_msg.posz = veh_t_world[2]
                # orientation
                veh_pose_euler_msg.rotx = veh_feaXYZ_world[0]
                veh_pose_euler_msg.roty = veh_feaXYZ_world[1]
                veh_pose_euler_msg.rotz = veh_feaXYZ_world[2]

                # finally publish the message
                self.pub_detection_in_robot_frame.publish(veh_pose_euler_msg)
                # save the message to a bag file
                #self.output_rosbag.write(self.pub_topic_name, veh_pose_euler_msg)
                #self.output_rosbag.write(self.pub_topic_name, Bool(True))

                self.lock.acquire()
                output_rosbag = rosbag.Bag(self.output_bag, 'a') # open bag to write
                output_rosbag.write(self.pub_topic_name, veh_pose_euler_msg)
                output_rosbag.close()
                self.lock.release()

                rospy.loginfo("[{}] wrote image {}".format(self.node_name, self.numb_written_images))
                self.numb_written_images += 1

                # request a new image from "buffer.py"
                req_msg = Bool(True)
                self.pub_image_request.publish(req_msg)
        else:
            rospy.loginfo("[{}] {}".format(self.node_name, "recording is finished"))
            #rospy.loginfo("[{}] {}".format(self.node_name, "closed rosbag"))

if __name__ == '__main__':
    to_local_pose = ToLocalPose()

    rospy.spin()
