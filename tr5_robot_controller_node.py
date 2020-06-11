#!/usr/bin/env python  
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from tr5_robot_controller.srv import KinematicMode, KinematicModeResponse
from tr5_kinematics.srv import DoForwardKinematics, DoInverseKinematics

class Controller:

    def __init__(self):
        # load private parameters from the Parameter Server
        # services
        self.svc_kin_mode = rospy.get_param("~kin_mode_service", "/kin_mode")
        self.svc_fk = rospy.get_param("~for_kin_service", "/do_fk")
        self.svc_ik = rospy.get_param("~inv_kin_service", "/do_ik")
        # topics
        self.tp_gui_jt = rospy.get_param("~gui_joints_topic", "/gui_joints")
        self.tp_goal_jt = rospy.get_param("~goal_joints_topic", "/goal_joints")
        self.tp_goal_ps = rospy.get_param("~goal_pose_topic", "/visualization_marker")
        self.tp_in_update = rospy.get_param("~input_pose_update_topic", "/input_pose/update")
        self.tp_in_feedback = rospy.get_param("~input_pose_feedback_topic", "/input_pose/feedback")  
        # kinematics mode flag
        self.ik_mode = False

        # non-interactive visual marker object for rviz
        self.m = Marker()
        self.m.header.frame_id = "/world"
        self.m.header.stamp = rospy.Time.now()
        self.m.ns = "fk marker"
        self.m.id = 0
        self.m.type = self.m.SPHERE
        self.m.action = self.m.ADD
        self.m.scale.x = self.m.scale.y = self.m.scale.z = 0.05
        self.m.color.r = 1.0
        self.m.color.a = 0.5
        self.m.color.g = self.m.color.b = 0.0
        self.m.lifetime = rospy.Duration(1.0)

        # interactive marker
        self.i_marker = InteractiveMarker()
        self.dot_marker = Marker()  
        
        # marker server
        self.m_marker_server = InteractiveMarkerServer("input_pose")
        self.create_interactive_marker()
        self.kinmode_handler = rospy.Service(self.svc_kin_mode, KinematicMode, self.kinematics_mode)
        self.do_fk_client_handler = rospy.ServiceProxy(self.svc_fk, DoForwardKinematics)
        self.do_ik_client_handler = rospy.ServiceProxy(self.svc_ik, DoInverseKinematics)
        self.gui_tp_sub = rospy.Subscriber(self.tp_gui_jt, JointState, self.gui_joints_callback)
        self.feedback_sub = rospy.Subscriber( self.tp_in_feedback, Pose, self.input_pose_callback)
        self.goal_tp_pub = rospy.Publisher(self.tp_goal_jt, JointState, queue_size=10)
        self.fk_vis_marker_pub = rospy.Publisher(self.tp_goal_ps, Marker, queue_size=10)
       

    def create_interactive_marker(self):
        self.i_marker.header.frame_id = "/world"
        self.i_marker.header.stamp = rospy.Time.now()
        self.i_marker.name = "user_input_marker"
        self.i_marker.description = "Input Pose"
        
        
        # create a grey box marker
        self.dot_marker.type = self.dot_marker.SPHERE
        self.dot_marker.scale.x = self.dot_marker.scale.y = self.dot_marker.scale.z = 0.05
        self.dot_marker.color.r = self.dot_marker.color.g = 0.0
        self.dot_marker.color.b = 1.0
        self.dot_marker.color.a = 0.5
        self.dot_marker.pose.position.x = 0.45
        self.dot_marker.pose.position.y = 0.0
        self.dot_marker.pose.position.z = 0.27
        
        #create a non-interactive control which contains the dot marker
        dot_control = InteractiveMarkerControl()
        dot_control.always_visible = True
        dot_control.markers.append(self.dot_marker)
        dot_control.orientation_mode = InteractiveMarkerControl.FIXED
        dot_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        #add the control to the interactive marker
        self.i_marker.controls.append(dot_control)

        self.m_marker_server.insert(self.i_marker, self.input_pose_callback)
        self.m_marker_server.applyChanges()

    def input_pose_callback(self, msg):
        # ignore feedback from the interactive marker while in forward kinematics mode
        rospy.wait_for_service(self.svc_kin_mode)
        if not self.ik_mode:
            return
        
        srv = Pose()
        srv.position.x = msg.pose.position.x + 0.45
        srv.position.y = msg.pose.position.y
        srv.position.z = msg.pose.position.z + 0.27
        srv.orientation.x = msg.pose.orientation.x
        srv.orientation.y = msg.pose.orientation.y
        srv.orientation.z = msg.pose.orientation.z
        srv.orientation.w = msg.pose.orientation.w
        
        res =  self.do_ik_client_handler.call(srv)

        if res:
            rospy.wait_for_service(self.svc_ik)
            self.goal_tp_pub.publish(res.Joints)
            #self.fk_vis_marker_pub.publish(self.i_marker)
        else:
            rospy.logerr("Failed to call service inverse kinematics...")

    def gui_joints_callback(self, msg):
        
        if(self.ik_mode):
            return
        joint = JointState()
        joint.position = msg.position
        rospy.wait_for_service(self.svc_fk)
        res = self.do_fk_client_handler.call(joint)
        
   
        if res:     
            self.m.pose.position.x = res.Cordinates.position.x
            self.m.pose.position.y = res.Cordinates.position.y
            self.m.pose.position.z = res.Cordinates.position.z
            self.fk_vis_marker_pub.publish(self.m)         
        else:
            rospy.logerr("Failed to call service for forward kinematics...")
        self.goal_tp_pub.publish(msg)

    def kinematics_mode(self, srv):
        if srv.mode_req == 0:
            self.ik_mode = False  
            
            self.m_marker_server.erase("user_input_marker")
            self.m_marker_server.applyChanges()
            return KinematicModeResponse(False)
        elif srv.mode_req == 1:
            self.ik_mode = True      
            self.create_interactive_marker()    
            return KinematicModeResponse(True)
        

    def run(self):
        rospy.spin()   


if __name__ == "__main__":
    rospy.init_node("tr5_robot_controller_node")
    controller = Controller()
    controller.run()
 