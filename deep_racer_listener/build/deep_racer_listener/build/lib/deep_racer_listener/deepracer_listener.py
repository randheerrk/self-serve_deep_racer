import time
import requests
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from deepracer_interfaces_pkg.msg import ServoCtrlMsg
        

class DeepRacerListener(Node) :
    def __init__(self) :
        super().__init__('deepracer_listener_node')
        self.get_logger().info("deepracer_listener_node started")

        self.subscriber = self.create_subscription(ServoCtrlMsg,
                "/auto_drive",
                self.listener_callback,
                10)
        self.subscriber

    def listener_callback(self, msg) :
        throttle = msg.throttle
        angle = msg.angle
        data = {
            "throttle": throttle,
            "angle": angle,
            "time": time.time()
        }
        self.get_logger().info(f'{data}')
        try :
            res = requests.post("http://52.140.123.110:10001/", json=data)
            if res.status_code != 200 : 
                raise Exception(msg="with status code, {}".format(res.status_code))
            self.get_logger().info(f'{time.time()} Saved message from navigation node')
        except Exception as e :
            self.get_logger().info(f'{time.time()} Failed to save message from navigation node, {e}')
    

def main(args=None) :
    rclpy.init(args=args)
    deepracer_listener = DeepRacerListener()
    executor = MultiThreadedExecutor()
    rclpy.spin(deepracer_listener, executor)

    deepracer_listener.destroy_node()
    rclpy.shutdown()


if __name__=="__main__" :
    main()


