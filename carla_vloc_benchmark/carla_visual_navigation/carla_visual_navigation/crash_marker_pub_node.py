import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

import json

class CrashMarkerPublisher(Node):

    '''
    Visualize the vehicle crash locations inside Carla/Rviz
    '''

    def __init__(self):
        super().__init__('crash_marker_publisher')

        self.declare_parameter("log_file_path")
        log_file_path = self.get_parameter('log_file_path').get_parameter_value().string_value

        self.declare_parameter("render_in_carla", False)
        self.render_in_carla = self.get_parameter('render_in_carla').get_parameter_value().bool_value

        # Only needed if rendering in Carla
        self.declare_parameter("carla_render_method", 'netvlad+r2d2+NN-ratio')
        carla_render_method = self.get_parameter('carla_render_method').get_parameter_value().string_value
        self.declare_parameter("carla_render_kvalue", 0)
        carla_render_kvalue = self.get_parameter('carla_render_kvalue').get_parameter_value().integer_value

        with open(log_file_path, 'r+') as f:
            logs = json.load(f)

        if self.render_in_carla:
            # Render markers in Carla
            self.pub = self.create_publisher(MarkerArray, '/carla/debug_marker', 10)
            self.markers = None
        else:
            # Render markers in Rviz
            self.pub = []

        color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        for method, kvalues in logs.items():

                for kvalue, attributes in kvalues.items():
                    markers = []
                    for location in attributes['crash_locations']:

                        marker = Marker(header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()),
                                        scale=Vector3(x=0.2,y=0.2,z=0.2),
                                        type=8,
                                        action=0,
                                        color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))

                        marker.colors.append(color)
                        marker.points.append(Point(x=location['x'], y=location['y'], z=location['z']))
                        markers.append(marker)

                    if self.render_in_carla:
                        if method == carla_render_method:
                            if int(float(kvalue)) == carla_render_kvalue:
                                self.markers = MarkerArray(markers=markers)
                    else:
                        pub_topic = '{}_k_{}'.format(method, int(float(kvalue))).replace('+','_').replace('-','_')
                        pub = self.create_publisher(MarkerArray, pub_topic, 10)
                        self.pub.append({'publisher': pub, 'markers': MarkerArray(markers=markers)})

        self.timer = self.create_timer( 0.5 , self.timer_callback)

    def timer_callback(self):

        if self.render_in_carla:
            self.pub.publish(self.markers)
        else:
            for method_k in self.pub:
                method_k['publisher'].publish(method_k['markers'])

def main(args=None):

    rclpy.init(args=args)

    try:
        mark = CrashMarkerPublisher()
        executor = SingleThreadedExecutor()
        executor.add_node(mark)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        mark.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()