import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PoseArray, PoseStamped, Vector3, Point
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

import json

class CrashMarkerPublisher(Node):

    def __init__(self):
        super().__init__('crash_marker_publisher')
        log_file_path = '/results/town01_illumination_run_navigation_grouped_by_method.json'
        with open(log_file_path, 'r+') as f:
            logs = json.load(f)

        self.pub = []

        self.messages = []

        color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        for metod, kvalues in logs.items():

            if metod == 'netvlad+r2d2+NN-ratio':

                for kvalue, attributes in kvalues.items():
                    markers = []
                    for location in attributes['crash_locations']:
                        marker = Marker(header=Header(frame_id='map', stamp=self.get_clock().now().to_msg()), scale=Vector3(x=0.2,y=0.2,z=0.2), type=8, action=0, color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
                        marker.colors.append(color)
                        marker.points.append(Point(x=location['x'], y=location['y'], z=location['z']))
                        markers.append(marker)

                    #pub_topic = '{}_k_{}'.format(metod, int(float(kvalue))).replace('+','_').replace('-','_')
                     #self.pub.append({'publisher': pub, 'markers': MarkerArray(markers=markers)})
                    self.messages.append(MarkerArray(markers=markers))

        pub_topic = '/carla/debug_marker'
        self.pub = self.create_publisher(MarkerArray, pub_topic, 10)

        self.timer = self.create_timer( 0.5 , self.timer_callback)

        #for method_k in self.pub:

            #method_k['publisher'].publish(method_k['markers'])

       # self.pub[9]['publisher'].publish(self.pub[9]['markers'])


    def timer_callback(self):

        self.pub.publish(self.messages[5])

        # for method_k in self.pub:

        #     method_k['publisher'].publish(method_k['markers'])


        return


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