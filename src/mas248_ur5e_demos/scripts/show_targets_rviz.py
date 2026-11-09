"""Missing starter preview entry point restored during submission review."""
import argparse
import json
import math


def rotate_vector_xyzw(q, vector):
    norm = math.sqrt(sum(float(v)**2 for v in q))
    if norm < 1e-12:
        raise ValueError('Zero quaternion')
    x, y, z, w = [float(v)/norm for v in q]
    a, b, c = vector
    return ((1-2*y*y-2*z*z)*a + (2*x*y-2*z*w)*b + (2*x*z+2*y*w)*c,
            (2*x*y+2*z*w)*a + (1-2*x*x-2*z*z)*b + (2*y*z-2*x*w)*c,
            (2*x*z-2*y*w)*a + (2*y*z+2*x*w)*b + (1-2*x*x-2*y*y)*c)


def target_axes(q, length):
    return tuple(rotate_vector_xyzw(q, v)
                 for v in ((length, 0, 0), (0, length, 0), (0, 0, length)))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--json', default='/tmp/mas248_targets.json')
    p.add_argument('--axis-every', type=int, default=10)
    p.add_argument('--axis-length-mm', type=float, default=18.)
    p.add_argument('--axis-lift-mm', type=float, default=0.)
    p.add_argument('--no-orientations', action='store_true')
    args = p.parse_args()
    if args.axis_every < 1 or not math.isfinite(args.axis_length_mm) or args.axis_length_mm <= 0:
        p.error('Invalid axis sampling/length')
    with open(args.json, encoding='utf-8') as f:
        data = json.load(f)
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray
    rclpy.init()
    node = Node('mas248_targets_preview')
    pub = node.create_publisher(MarkerArray, '/mas248/drawing_targets', 10)
    array = MarkerArray()
    for i, stroke in enumerate(data['strokes']):
        line = Marker()
        line.header.frame_id = 'world'
        line.ns = 'MAS248_contour_paths'
        line.id = i
        line.type = Marker.LINE_STRIP
        line.pose.orientation.w = 1.0
        line.scale.x = .002
        line.color.r, line.color.g, line.color.b, line.color.a = 1., .5, .1, 1.
        for j, pose in enumerate(stroke['poses']):
            line.points.append(Point(x=float(pose['x']), y=float(pose['y']), z=float(pose['z'])))
            if not args.no_orientations and j % args.axis_every == 0:
                q = [pose[k] for k in ('qx', 'qy', 'qz', 'qw')]
                for k, vector in enumerate(target_axes(q, args.axis_length_mm/1000)):
                    arrow = Marker()
                    arrow.header.frame_id = 'world'
                    arrow.ns = 'MAS248_TCP_orientation_' + 'XYZ'[k]
                    arrow.id = i*100000+j
                    arrow.type = Marker.ARROW
                    arrow.pose.orientation.w = 1.
                    arrow.scale.x, arrow.scale.y, arrow.scale.z = .001, .003, .004
                    arrow.color.r, arrow.color.g, arrow.color.b = [float(k == n) for n in range(3)]
                    arrow.color.a = 1.
                    start = (float(pose['x']), float(pose['y']),
                             float(pose['z'])+args.axis_lift_mm/1000)
                    end = [a+b for a, b in zip(start, vector)]
                    arrow.points = [Point(x=start[0], y=start[1], z=start[2]),
                                    Point(x=end[0], y=end[1], z=end[2])]
                    array.markers.append(arrow)
        array.markers.append(line)
    def publish():
        for marker in array.markers:
            marker.header.stamp = node.get_clock().now().to_msg()
        pub.publish(array)
    node.create_timer(1., publish)
    print('RViz: MarkerArray /mas248/drawing_targets (world). Ctrl+C stops preview.', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
