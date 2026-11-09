"""A: add/move/remove collision box in MoveIt planning scene (service, persistent)."""
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from .common import wait_future

def add_box(node,name,x,y,z,sx,sy,sz,remove=False):
    client=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
    if not client.wait_for_service(timeout_sec=20):
        raise RuntimeError('MoveIt planning scene service not running; launch MoveIt first')
    scene=PlanningScene();scene.is_diff=True
    obj=CollisionObject();obj.header.frame_id='world';obj.id=name
    obj.operation=CollisionObject.REMOVE if remove else CollisionObject.ADD
    if not remove:
        primitive=SolidPrimitive();primitive.type=SolidPrimitive.BOX
        primitive.dimensions=[float(sx),float(sy),float(sz)]
        pose=Pose();pose.position.x=float(x);pose.position.y=float(y);pose.position.z=float(z)
        pose.orientation.w=1.0
        obj.primitives=[primitive];obj.primitive_poses=[pose]
    scene.world.collision_objects=[obj]
    req=ApplyPlanningScene.Request();req.scene=scene
    res=wait_future(node,client.call_async(req),30)
    if not res.success:raise RuntimeError('ApplyPlanningScene rejected '+name)
    node.get_logger().info(('Removed' if remove else 'Added')+' collision object '+name)

def main():
    ap=argparse.ArgumentParser(description='Add cube to UR5e MoveIt scene (MOCK demo)')
    ap.add_argument('--x',type=float,default=0.52);ap.add_argument('--y',type=float,default=0.0)
    ap.add_argument('--z',type=float,default=0.65);ap.add_argument('--size',type=float,default=0.12)
    ap.add_argument('--id',default='mas248_cube');ap.add_argument('--remove',action='store_true')
    ap.add_argument('--table',action='store_true',help='also add a drawing table below the workspace')
    args=ap.parse_args()
    rclpy.init();node=Node('mas248_add_cube')
    try:
        if args.table: add_box(node,'drawing_table',0.58,0,0.43,0.64,0.50,0.20,remove=args.remove)
        add_box(node,args.id,args.x,args.y,args.z,args.size,args.size,args.size,remove=args.remove)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__': main()
