#!/usr/bin/env python3
"""Publish a goal and record compact CSV telemetry for an already running simulation."""
import argparse, csv, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

class Runner(Node):
    def __init__(self, goal, output):
        super().__init__('experiment_runner'); self.goal=goal; self.output=output; self.rows=[]; self.rpm=[0]*4; self.start=time.monotonic()
        self.goal_publish_count=0
        self.pub=self.create_publisher(PoseStamped,'/drone/goal',10); self.create_subscription(Odometry,'/drone/ground_truth/odom',self.odom,10); self.create_subscription(Float32MultiArray,'/drone/motor_rpm',lambda m:setattr(self,'rpm',list(m.data)),10); self.goal_timer=self.create_timer(0.25,self.send)
    def send(self):
        # Wait for DDS discovery when the recorder is started immediately after launch.
        if self.pub.get_subscription_count() == 0:
            return
        m=PoseStamped();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='map';m.pose.position.x,m.pose.position.y,m.pose.position.z=self.goal;m.pose.orientation.w=1.;self.pub.publish(m)
        self.goal_publish_count += 1
        if self.goal_publish_count >= 5: self.goal_timer.cancel()
    def odom(self,m):
        p=m.pose.pose.position; self.rows.append([time.monotonic()-self.start,p.x,p.y,p.z,*self.rpm])
    def save(self):
        with open(self.output,'w',newline='') as f: w=csv.writer(f);w.writerow(['t','x','y','z','rpm1','rpm2','rpm3','rpm4']);w.writerows(self.rows)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--goal',nargs=3,type=float,default=[0,0,1.5]);ap.add_argument('--duration',type=float,default=25);ap.add_argument('--output',default='experiment.csv');a=ap.parse_args();rclpy.init();n=Runner(a.goal,a.output);end=time.monotonic()+a.duration
    while rclpy.ok() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
    n.save();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
