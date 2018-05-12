#!/usr/bin/env python
import moveit_commander
import rospy
import tf
import geometry_msgs.msg

def main():
    rospy.init_node("moveit_command_sender")
    
    robot = moveit_commander.RobotCommander()
    
    print "=" * 10, " Robot Groups:"
    print robot.get_group_names()
    
    print "=" * 10, " Printing robot state"
    print robot.get_current_state()
    print "=" * 10 
    
    arm = moveit_commander.MoveGroupCommander("arm")
    
    print "=" * 15, " arm ", "=" * 15
    print "=" * 10, " Reference frame: %s" % arm.get_planning_frame()
    print "=" * 10, " Reference frame: %s" % arm.get_end_effector_link()
    
    
    #Right Arm Initial Pose
    arm_initial_pose = arm.get_current_pose().pose
    print "=" * 10, " Printing Hand initial pose: "
    print arm_initial_pose
    
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = -5.11003161302e-05
    #target_pose.position.x = -0.11003161302
    target_pose.position.y = -0.0447297517042
    target_pose.position.z = 0.151916904484
    target_pose.orientation.x = 0.526711473227
    #target_pose.orientation.y = -0.000300863664384
    target_pose.orientation.y = -0.00300863664384
    target_pose.orientation.z = -0.000485554895381
    target_pose.orientation.w = 0.850043938681
    #target_pose.orientation.w = 0.950043938681
    arm.set_pose_target(target_pose)
    
    print "=" * 10," plan1 ..."
    arm.set_goal_position_tolerance(0.01)
    arm.set_goal_orientation_tolerance(10)
    #print arm.get_goal_position_tolerance()
    #print arm.get_goal_orientation_tolerance()
    arm.go()
    rospy.sleep(5)
    
    #Clear pose
    #arm.clear_pose_targets()
    
    
    print "=" * 10,"Initial pose ..."
    arm.set_pose_target(arm_initial_pose)
    arm.go()
    rospy.sleep(5)
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
