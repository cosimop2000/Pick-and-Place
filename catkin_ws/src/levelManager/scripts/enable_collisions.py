#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelProperties, SetLinkProperties, SetLinkPropertiesRequest

def enable_collision(model_name, link_name):
    rospy.wait_for_service('/gazebo/get_model_properties')
    try:
        get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
        model_properties = get_model_properties(model_name)
        
        if not model_properties.success:
            rospy.logerr("Failed to get model properties for: " + model_name)
            return
        
        link_found = False
        for link in model_properties.body_names:
            if link == link_name:
                link_found = True
                break
        
        if not link_found:
            rospy.logerr("Link " + link_name + " not found in model " + model_name)
            return
        
        rospy.wait_for_service('/gazebo/set_link_properties')
        try:
            set_link_properties = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
            link_properties_req = SetLinkPropertiesRequest()
            link_properties_req.link_name = model_name + '::' + link_name
            link_properties_req.gravity_mode = True
            link_properties_req.mass = 111.0
            link_properties_req.ixx = 0.1
            link_properties_req.iyy = 0.1
            link_properties_req.izz = 0.1
            link_properties_req.ixy = 0.0
            link_properties_req.ixz = 0.0
            link_properties_req.iyz = 0.0
            link_properties_req.com.position.x = 0.0
            link_properties_req.com.position.y = 0.0
            link_properties_req.com.position.z = 0.0
            link_properties_req.com.orientation.x = 0.0
            link_properties_req.com.orientation.y = 0.0
            link_properties_req.com.orientation.z = 0.0
            link_properties_req.com.orientation.w = 1.0
            link_properties_req.linear_damping = 0.0
            link_properties_req.angular_damping = 0.0
            link_properties_req.max_contacts = 10  # Enable collision by setting max_contacts to a positive value
            
            resp = set_link_properties(link_properties_req)
            if not resp.success:
                rospy.logerr("Failed to set link properties for: " + model_name + "::" + link_name)
                return
            
            rospy.loginfo("Successfully enabled collision for: " + model_name + "::" + link_name)
        
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('enable_collision_node')
    enable_collision("table", "table")
