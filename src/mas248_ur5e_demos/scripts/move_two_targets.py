"""C: joint-space two goal motion via Humble-compatible MoveGroup action."""
import argparse
import rclpy
from rclpy.node import Node
from .common import move_group, make_pose

def main():
    p=argparse.ArgumentParser(description='Two-goal MoveIt2 demo. Default PLAN ONLY.')
    p.add_argument('--execute',action='store_true',help='Execute ONLY on bundled mock controller launch')
    p.add_argument('--mode',choices=['joints','tcp'],default='joints',help='TCP poses may need manual adjustment')
    args=p.parse_args()
    rclpy.init();node=Node('mas248_move_two_targets')
    try:
        if args.mode=='joints':
            targets=[[0.0,-1.57,1.57,-1.57,-1.57,0.0],
                     [0.35,-1.3,1.30,-1.6,-1.57,0.20]]
        else:
            targets=[make_pose(0.53,-0.08,0.67),make_pose(0.55,0.08,0.67)]
        for i,target in enumerate(targets,1):
            node.get_logger().info(f'Target {i} of 2')
            move_group(node, joints=target if args.mode=='joints' else None,
                       pose=target if args.mode=='tcp' else None, execute=args.execute)
        if not args.execute:
            node.get_logger().warn('PLAN ONLY: the robot did not move. Run with --execute for mock motion.')
    except Exception as exc:
        node.get_logger().error(str(exc));raise
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
