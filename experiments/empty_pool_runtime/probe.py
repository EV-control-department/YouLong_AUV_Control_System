import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from uv_msgs.msg import PoseInfo
from zit6_interfaces.msg import ZitSetpoint
from stonefish_ros2.msg import ThrusterState

STATE_ODOM='/auv/state/odom'
SETPOINT='/auv/hardware/zit6/cmd/setpoint'
GT='/auv/sim/ground_truth/odom'
THR='/auv/sim/actuators/thruster_state'

class Probe(Node):
    def __init__(self):
        super().__init__('empty_pool_probe')
        self.pose_pub=self.create_publisher(PoseInfo, STATE_ODOM, 10)
        self.sp_pub=self.create_publisher(ZitSetpoint, SETPOINT, 10)
        self.gt=None; self.thr=None
        self.create_subscription(Odometry, GT, self.gt_cb, 10)
        self.create_subscription(ThrusterState, THR, self.thr_cb, 10)
        self.t0=time.monotonic(); self.phase=-1; self.seq=0; self.last_print=0
        self.timer=self.create_timer(0.01,self.tick)
    def gt_cb(self,m):
        p=m.pose.pose; v=m.twist.twist
        self.gt=(p.position.x,p.position.y,p.position.z,v.linear.x,v.linear.y,v.linear.z)
    def thr_cb(self,m): self.thr=list(m.setpoint)
    def tick(self):
        now=time.monotonic()-self.t0
        p=PoseInfo(); p.robot_x=0.; p.robot_y=0.; p.robot_z=1.; p.robot_roll=0.; p.robot_pitch=0.; p.robot_yaw=0.
        self.pose_pub.publish(p)
        phase=int(now//1.5)
        if phase != self.phase:
            self.phase=phase
            names=['ACT_FX','ACT_FY','ACT_FZ','ACT_YAW','STOP','POS_X']
            self.get_logger().info('phase %d: %s' % (phase, names[phase] if phase<6 else 'DONE'))
        sp=ZitSetpoint(); sp.control_key=2; sp.type_mask=0; sp.seq=self.seq; self.seq+=1
        if phase==0: sp.x=0.2
        elif phase==1: sp.y=0.2
        elif phase==2: sp.z=0.2
        elif phase==3: sp.yaw=0.2
        elif phase==5: sp.control_key=0; sp.x=0.5
        self.sp_pub.publish(sp)
        if now-self.last_print>=0.1:
            self.last_print=now
            self.get_logger().info('t=%.2f phase=%d gt=%s thr=%s' % (now,phase,self.gt,self.thr))
        if now>=9.0: rclpy.shutdown()

def main():
    rclpy.init(); n=Probe()
    try: rclpy.spin(n)
    except (KeyboardInterrupt,): pass
    n.destroy_node()
if __name__=='__main__': main()
