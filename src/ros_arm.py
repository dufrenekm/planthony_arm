#!/usr/bin/env python
"""
ros_arm.py
"""
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Vector3
from math import pi, radians
from std_srvs.srv import Empty

from plant_arm_project.srv import PlantLocation, PlantLocationResponse

class ArmNode:
    def __init__(self):
        return
        
    def start(self, node_name="arm_node", service_name="vis"):

        # rospy.wait_for_service("vis_node")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node(node_name)

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())

            self.planthony_srv = rospy.Service("planthony", PlantLocation, self.planthony_callback)
        except Exception as e:
            print (e)
        
        #rospy.init_node(node_name)
        #rospy.Subscriber("vis_node", Vector3, self._node_request_handler)
        #self.go_home()
        self.go_to_start()
        
        actual_pose = self.get_cartesian_pose()
        actual_pose.position.z = .35 # x positive is front
        actual_pose.position.x = .3 # x positive is front
        self.reach_cartesian_pose(actual_pose, tolerance=.01, constraints=None)


        
        rospy.spin()
        return

    def go_home(self):
        success = True
        rospy.loginfo("Reaching Named Target Home...")
        success &= self.reach_named_position("home")
        print (success)

    def go_to_start(self):
        success = True
        rospy.loginfo("Reaching Joint Angles...")  
        success &= self.reach_joint_angles2(tolerance=0.01) #rad
        print (success)
        actual_pose = self.get_cartesian_pose()
        print(actual_pose)

    def reach_named_position(self, target):
        arm_group = self.arm_group
        
        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        planned_path1 = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(planned_path1, wait=True)

    def reach_joint_angles2(self, tolerance):
        arm_group = self.arm_group
        success = True

        # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions before movement :")
        for p in joint_positions: rospy.loginfo(p)

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        #Base rotation
        joint_positions[0] = 0#radians(51.11)#0
        # base angle
        joint_positions[1] = pi/6#radians(64.2)#0#pi/6
        #Shoulder rotation
        joint_positions[2] = pi#radians(246.79)#pi
        #print(joint_positions[2])
        # Shoulder angle
        joint_positions[3] = -pi/2#radians(123.83)#-3*pi/4 + pi/3
        joint_positions[4] = 0#radians(101.02)#0
        joint_positions[5] = -pi/3#radians(84.8)#-pi/3
        joint_positions[6] = pi/2#radians(189.88)#pi/2
        arm_group.set_joint_value_target(joint_positions)
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)

        # Show joint positions after movement
        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions after movement :")
        for p in new_joint_positions: rospy.loginfo(p)
        return success

    '''
    def _node_request_handler(self, point):

        rospy.loginfo(f"Got point ({point.x}, {point.y}, {point.z})")

        return
    '''

    def planthony_callback(self, request):
        actual_pose = self.get_cartesian_pose()
        print(request)
        request = self.transform(request)
        print(request)
        location_max_x = [.3,.6]
        location_max_y = [.3,-.3]

        # Verify x within safe bounds, update if not
        if request.loc.x < location_max_x[0]:
            print("Below safe x val, adjusted")
            request.loc.x = location_max_x[0]
        elif request.loc.x > location_max_x[1]:
            request.loc.x = location_max_x[1]
            print("Above safe x val, adjusted")

        # Verify x within safe bounds, update if not
        if request.loc.y < location_max_y[1]:
            print("Below safe y val, adjusted")
            request.loc.y = location_max_y[1]
        elif request.loc.y > location_max_y[0]:
            request.loc.y = location_max_y[0]
            print("Above safe y val, adjusted")

        actual_pose.position.x = request.loc.x # x positive is front
        actual_pose.position.y = request.loc.y # x positive is front
        actual_pose.position.z = .28 #request.loc.z # x positive is front # Fix to somehwere .2/.3??
        self.reach_cartesian_pose(actual_pose, tolerance=.01, constraints=None)

        actual_pose = self.get_cartesian_pose()
        print("Updated pose:")
        print(actual_pose)
        return PlantLocationResponse(1)

    def transform(self, request):
        print("hui")
        print(request.loc.y)
        temp = request.loc.x
        # .18 to shift to center of camera, .762 from center of camera to base of robot

        request.loc.x = request.loc.y + .762 -.35 + .08
        request.loc.y = temp
        return request
        
    
    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.arm_group
        
        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)

        # Plan and execute
        rospy.loginfo("Planning and going to the Cartesian Pose")
        return arm_group.go(wait=True)

def main():
    arm = ArmNode()
    arm.start()
    rospy.spin()


if __name__ == "__main__":
    main()