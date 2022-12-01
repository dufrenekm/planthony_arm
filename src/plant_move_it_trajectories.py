#!/usr/bin/env python
#roslaunch kortex_driver kortex_driver.launch gripper:=robotiq_2f_85



# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty

from plant_arm_project.srv import PlantLocation, PlantLocationResponse

class PlantMoveItTrajectories(object):
  """PlantMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(PlantMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('plant_move_it_trajectories')

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

      #self.add_box(pos=[0,0,1.1],dim=[1,1,.1],name="top")
      self.planthony_srv = rospy.Service("planthony", PlantLocation, self.planthony_callback)
    
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

    self.go_home()
    self.go_to_start()
  
    rospy.spin()

  def planthony_callback(self, request):
    actual_pose = self.get_cartesian_pose()
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
    actual_pose.position.z = .2 #request.loc.z # x positive is front # Fix to somehwere .2/.3??
    self.reach_cartesian_pose(actual_pose, tolerance=.01, constraints=None)

    actual_pose = self.get_cartesian_pose()
    print("Updated pose:")
    print(actual_pose)
    return PlantLocationResponse(1)

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

  def reach_joint_angles(self, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = pi/2
      joint_positions[1] = 0
      joint_positions[2] = pi/4
      joint_positions[3] = -pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
      joint_positions[6] = 0.2
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = 0
      joint_positions[1] = 0
      joint_positions[2] = pi/2
      joint_positions[3] = pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

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
    joint_positions[0] = 0
    # base angle
    joint_positions[1] = pi/6
    #Shoulder rotation
    joint_positions[2] = pi
    print(joint_positions[2])
    # Shoulder angle
    joint_positions[3] = -pi/2
    joint_positions[4] = 0
    joint_positions[5] = -pi/3
    joint_positions[6] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

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

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

def main():
  example = PlantMoveItTrajectories()

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass
  
  '''
  if success:
    rospy.loginfo("Reaching Cartesian Pose...")
    
    actual_pose = example.get_cartesian_pose()
    print("ho")
    print(actual_pose.position)
    print("yo")
    actual_pose.position.z -= 0.2
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)
  
  if success:
    rospy.loginfo("Reaching Named Target Home...")
    success &= example.reach_named_position("home")
    print (success)
  
  if success:
    rospy.loginfo("Reaching Named Target Vertical...")
    success &= example.reach_named_position("vertical")
    print (success)
  
  if success:
    rospy.loginfo("Reaching Named Target Home...")
    success &= example.reach_named_position("home")
    print (success)
  
  if success:
    rospy.loginfo("Reaching Joint Angles...")  
    success &= example.reach_joint_angles2(tolerance=0.01) #rad
    print (success)
    

  
  while True:
    actual_pose = example.get_cartesian_pose()
    print("Pose")
    inp = raw_input("e to exit, otherwise x,y: ")
    if inp == "e":
      break
    inp_val = inp.split(",")

    x = float(inp_val[0])
    y = float(inp_val[1])

    rospy.loginfo("Reaching Cartesian Pose...")
      
    actual_pose.position.x = x # x positive is front
    actual_pose.position.y = y # y positive is left

    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)
    '''
  

  '''
  if success:
    rospy.loginfo("Reaching Joint Angles...")  
    success &= example.reach_joint_angles(tolerance=0.01) #rad
    print (success)

  if success:
    rospy.loginfo("Reaching Joint Angles...")  
    success &= example.reach_joint_angles2(tolerance=0.01) #rad
    print (success)
  
  if success:
    rospy.loginfo("Reaching Named Target Home...")
    success &= example.reach_named_position("home")
    print (success)
  
  if success:
    rospy.loginfo("Reaching Cartesian Pose...")
    
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.z -= 0.2
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)

  if success:
    rospy.loginfo("Reaching Cartesian Pose...")
    
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.y += 0.1
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)

  if success:
    rospy.loginfo("Reaching Cartesian Pose...")
    
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.x += 0.1
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)
  
  if example.degrees_of_freedom == 7 and success:
    rospy.loginfo("Reach Cartesian Pose with constraints...")
    # Get actual pose
    actual_pose = example.get_cartesian_pose()
    actual_pose.position.y -= 0.3
    
    # Orientation constraint (we want the end effector to stay the same orientation)
    constraints = moveit_msgs.msg.Constraints()
    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    orientation_constraint.orientation = actual_pose.orientation
    constraints.orientation_constraints.append(orientation_constraint)

    # Send the goal
    success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

  if example.is_gripper_present and success:
    rospy.loginfo("Opening the gripper...")
    success &= example.reach_gripper_position(0)
    print (success)

    rospy.loginfo("Closing the gripper 50%...")
    success &= example.reach_gripper_position(0.5)
    print (success)
  '''
  # For testing purposes
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()
