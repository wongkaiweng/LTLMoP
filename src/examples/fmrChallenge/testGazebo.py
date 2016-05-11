import rospy
import gazebo_msgs.msg
import gazebo_msgs.srv

import geometry_msgs.msg


rospy.init_node('test_gazebo')

def callback(msg, agent):
    egoIdx = msg.name.index('ego')
    #print msg.pose[egoIdx]
    #print agent

# get ego pose
rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, callback, 'agent0')

# set ego velocity
#pub_ego = rospy.Publisher('/ego/mobile_base/commands/velocity', geometry_msgs.msg.Twist, queue_size=10)

#pub_ego_twist = geometry_msgs.msg.Twist()
#pub_ego_twist.linear.x = -1.0

pub_ego = rospy.Publisher('/gazebo/set_model_state', gazebo_msgs.msg.ModelState, queue_size=10)
pub_ego_modelState = gazebo_msgs.msg.ModelState()
pub_ego_modelState.model_name = 'ego'
pub_ego_modelState.pose.position.x = 2.0
pub_ego_modelState.pose.position.y = -4.0
pub_ego_modelState.reference_frame = 'world'

rospy.wait_for_service('/gazebo/set_model_state')
try:
    service_ego = rospy.ServiceProxy('/gazebo/set_model_state', gazebo_msgs.srv.SetModelState)
    output = service_ego(pub_ego_modelState)
    print output.success
except rospy.ServiceException, e:
    print "Service call failed: %s"%e


# set velocity
#r = rospy.Rate(10) # 10hz
#while not rospy.is_shutdown():
    #pub_ego.publish(pub_ego_twist)
    #pub_ego.publish(pub_ego_modelState)
    #r.sleep()
#rospy.spin()