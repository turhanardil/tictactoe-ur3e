"""
arm_control node.

P4 scope: hardcoded pick of blue_token_0 from the tray and place at board
cell 4 (board center). No game logic integration. Subscribers and full
parameterization come in P5.

Architecture:
  Joint-space transit       OMPL RRTConnect via /move_action.
  Cartesian descent/ascent  Pilz LIN via /move_action, with a pose-goal
                            OMPL fallback if Pilz refuses (DetachableJoint
                            forgives imprecise descents).
  Token attach / detach     std_msgs/Empty publishes on the bridged
                            /attach_token_<n> and /detach_token_<n>
                            topics, one DJ instance per token.
  Move done                 std_msgs/Bool on /move_done after the
                            sequence completes.

DetachableJoint plugins start ATTACHED at simulation start in Gazebo
Harmonic. We publish detach to all five at startup so the tokens rest
on the table.

Verbose state logging on every transition makes failures easy to localize.
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Empty


JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Pose targets, world frame. z=0.10 is the table top.
TRAY_TOKEN_0 = (0.20, -0.20)
BOARD_CELL_4 = (0.30, 0.12)
APPROACH_Z = 0.30
GRASP_Z = 0.135   # cylinder top is at 0.12, wrist offset 1.5cm above for DJ attach
PLACE_Z = 0.135


class ArmControl(Node):

    def __init__(self):
        super().__init__('arm_control')
        self._move_action = ActionClient(self, MoveGroup, '/move_action')
        self._attach_pubs = [
            self.create_publisher(Empty, '/attach_token_%d' % i, 1)
            for i in range(5)
        ]
        self._detach_pubs = [
            self.create_publisher(Empty, '/detach_token_%d' % i, 1)
            for i in range(5)
        ]
        self._done_pub = self.create_publisher(Bool, '/move_done', 10)
        self.create_subscription(JointState, '/joint_states',
                                 self._on_joint_state, 10)
        self._joint_state = None

    def _on_joint_state(self, msg):
        self._joint_state = dict(zip(msg.name, msg.position))

    def detach_all(self):
        self.get_logger().info('detaching all tokens at startup')
        for _ in range(5):
            for pub in self._detach_pubs:
                pub.publish(Empty())
            time.sleep(0.05)

    def attach_token(self, idx):
        self.get_logger().info('attaching token %d' % idx)
        self._attach_pubs[idx].publish(Empty())
        time.sleep(0.5)

    def detach_token(self, idx):
        self.get_logger().info('detaching token %d' % idx)
        self._detach_pubs[idx].publish(Empty())
        time.sleep(0.5)

    def _send_goal(self, request, timeout=30.0):
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False
        if not self._move_action.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_action server unavailable')
            return False
        send_future = self._move_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('plan goal rejected')
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if result_future.result() is None:
            self.get_logger().error('plan/execute timed out')
            return False
        err = result_future.result().result.error_code.val
        self.get_logger().info('  -> error_code=%d' % err)
        return err == 1

    def _build_joint_request(self, joints, planner='ompl'):
        req = MotionPlanRequest()
        req.group_name = 'ur_manipulator'
        if planner == 'ompl':
            req.pipeline_id = 'ompl'
            req.planner_id = 'RRTConnect'
        else:
            req.pipeline_id = 'pilz_industrial_motion_planner'
            req.planner_id = 'PTP'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        c = Constraints()
        for jname, val in zip(JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        return req

    def _build_pose_request(self, x, y, z, planner='pilz_lin'):
        req = MotionPlanRequest()
        req.group_name = 'ur_manipulator'
        if planner == 'pilz_lin':
            req.pipeline_id = 'pilz_industrial_motion_planner'
            req.planner_id = 'LIN'
        else:
            req.pipeline_id = 'ompl'
            req.planner_id = 'RRTConnect'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        c = Constraints()
        # Position constraint, 1cm tolerance box around the target.
        pc = PositionConstraint()
        pc.header.frame_id = 'world'
        pc.link_name = 'tool0'
        pc.weight = 1.0
        bv = BoundingVolume()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [0.01, 0.01, 0.01]
        bv.primitives.append(prim)
        from geometry_msgs.msg import Pose
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.w = 1.0
        bv.primitive_poses.append(p)
        pc.constraint_region = bv
        c.position_constraints.append(pc)
        # Orientation constraint, TCP z pointing world -z (180° about world x).
        oc = OrientationConstraint()
        oc.header.frame_id = 'world'
        oc.link_name = 'tool0'
        oc.orientation.x = 1.0
        oc.orientation.y = 0.0
        oc.orientation.z = 0.0
        oc.orientation.w = 0.0
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = math.pi  # don't constrain wrist roll
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        req.goal_constraints = [c]
        return req

    def go_joint(self, joints, label):
        self.get_logger().info('joint goto: %s' % label)
        return self._send_goal(self._build_joint_request(joints, 'ompl'))

    def go_pose(self, x, y, z, label):
        self.get_logger().info('pose goto: %s -> (%.3f, %.3f, %.3f)' % (label, x, y, z))
        ok = self._send_goal(self._build_pose_request(x, y, z, 'pilz_lin'))
        if ok:
            return True
        self.get_logger().warn('  Pilz LIN failed, falling back to OMPL pose')
        return self._send_goal(self._build_pose_request(x, y, z, 'ompl'))

    def run_p4(self):
        self.detach_all()
        time.sleep(1.0)

        if not self.go_joint([0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0], 'up'):
            self.get_logger().error('failed to reach up'); return

        tx, ty = TRAY_TOKEN_0
        if not self.go_pose(tx, ty, APPROACH_Z, 'above tray slot 0'): return
        if not self.go_pose(tx, ty, GRASP_Z, 'descend to grasp 0'): return
        self.attach_token(0)
        if not self.go_pose(tx, ty, APPROACH_Z, 'ascend from tray 0'): return

        bx, by = BOARD_CELL_4
        if not self.go_pose(bx, by, APPROACH_Z, 'above board cell 4'): return
        if not self.go_pose(bx, by, PLACE_Z, 'descend to place 0'): return
        self.detach_token(0)
        if not self.go_pose(bx, by, APPROACH_Z, 'ascend from board 4'): return

        if not self.go_joint([0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0], 'home (up)'):
            return

        self._done_pub.publish(Bool(data=True))
        self.get_logger().info('move_done published, P4 sequence complete')


def main():
    rclpy.init()
    node = ArmControl()
    node.get_logger().info('waiting 8s for move_group + controllers to settle')
    end = time.time() + 8.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.run_p4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
