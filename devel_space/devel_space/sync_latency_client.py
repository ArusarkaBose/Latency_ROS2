#!/usr/bin/env python

from devel_interfaces.srv import LatencyCheck

import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import numpy as np
import sys
from threading import Thread
import time
import json

class MinimalClientSync(Node):

    def __init__(self):
        super().__init__('minimal_client_sync')
        self.cli = self.create_client(LatencyCheck, 'latency_check')
        while not self.cli.wait_for_service(timeout_sec=0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self):
        self.req = LatencyCheck.Request()
        self.req.query_time=self.get_clock().now().to_msg()
        return self.cli.call(self.req)
        # This only works because rclpy.spin() is called in a separate thread below.
        # Another configuration, like spinning later in main() or calling this method from a timer callback, would result in a deadlock.

def main():
    num_queries=1000
    num_bursts=10

    rclpy.init()

    minimal_client = MinimalClientSync()

    spin_thread = Thread(target=rclpy.spin, args=(minimal_client,))
    spin_thread.start()

    latency_bursts=[]
    for i in range(num_bursts):
        latencies=[]
        for i in range(num_queries):
            response = minimal_client.send_request()
            
            return_time=response.return_time
            current_time=minimal_client.get_clock().now().to_msg()

            latency=(current_time.nanosec+current_time.sec*(10**9))-(return_time.nanosec+return_time.sec*(10**9))
            latency/=10**6
            latencies.append(latency)
            print("Latency obtained = ",latency)

        mean=np.mean(latencies)
        standard_dev=np.std(latencies)   

        print("Mean = ",mean)
        print("Error Bar = ",mean+standard_dev)
        print("Max = ",max(latencies))
        print("Min = ",min(latencies))

        latency_bursts.append(latencies)
        time.sleep(5)

    y=[np.mean(latencies) for latencies in latency_bursts]
    x=np.arange(1,len(y)+1,1)
    yerr=[np.std(latencies) for latencies in latency_bursts]

    """
    In order to plot both ROS2 and ROS1 latencies on the same plot, uncomment the below lines.
    Change the file open path as per your json file location.
    """



    # with open('/home/arusarka/catkin_ws/src/Latency_ROS/src/latency_ROS1.json') as f:
    #     latency_ROS1=json.load(f)
    # y_ros1=latency_ROS1["y"]
    # x_ros1=latency_ROS1["x"]
    # yerr_ros1=latency_ROS1["yerr"]

    ax=plt.subplot()

    # ax.errorbar(x_ros1,y_ros1,yerr_ros1,label='ROS1')      #Uncomment for both plots on the same graph

    ax.errorbar(x,y,yerr,label='ROS2')
    ax.set(xlabel='Query Number (at 5 sec intervals)',ylabel='Latency (in ms) ',title='Latency Plot')
    plt.xticks(np.arange(min(x), max(x)+1, 1.0))
    ax.grid()

    ax.legend(loc='upper center')

    plt.show()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
