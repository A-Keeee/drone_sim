#!/usr/bin/env python3
"""Fly a four-corner mission, advancing after a stable 0.3 m arrival."""
import argparse, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Mission(Node):
    def __init__(self, waypoints, dwell):
        super().__init__('waypoint_mission'); self.waypoints=waypoints;self.dwell=dwell;self.index=0;self.entered=None;self.done=False
        self.pub=self.create_publisher(PoseStamped,'/drone/goal',10)
        self.create_subscription(Odometry,'/drone/odom',self.odom,10)
        self.create_timer(.2,self.publish_goal)
    def publish_goal(self):
        if self.done:return
        m=PoseStamped();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='map';m.pose.position.x,m.pose.position.y,m.pose.position.z=self.waypoints[self.index];m.pose.orientation.w=1.;self.pub.publish(m)
    def odom(self,m):
        if self.done:return
        p=m.pose.pose.position;g=self.waypoints[self.index];d=math.dist((p.x,p.y,p.z),g)
        if d<.3:
            if self.entered is None:self.entered=time.monotonic()
            elif time.monotonic()-self.entered>=self.dwell:
                self.get_logger().info(f'waypoint {self.index+1}/{len(self.waypoints)} reached')
                self.index+=1;self.entered=None
                if self.index==len(self.waypoints):self.done=True
        else:self.entered=None
def main():
    p=argparse.ArgumentParser();p.add_argument('--dwell',type=float,default=1.0);p.add_argument('--timeout',type=float,default=90.0);a=p.parse_args()
    waypoints=[(0.,0.,1.5),(2.,0.,1.5),(2.,2.,2.0),(0.,2.,1.5),(0.,0.,1.5)]
    rclpy.init();node=Mission(waypoints,a.dwell);end=time.monotonic()+a.timeout
    while rclpy.ok() and not node.done and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    if not node.done:node.get_logger().error('mission timeout')
    node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
