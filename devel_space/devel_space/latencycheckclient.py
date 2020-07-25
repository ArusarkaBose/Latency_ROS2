#!/usr/bin/env python

from devel_interfaces.srv import LatencyCheck

import rclpy
import matplotlib.pyplot as plt
import numpy as np
import pickle as pl


def main():
    rclpy.init()
    node = rclpy.create_node('latency_check_client')
    cli = node.create_client(LatencyCheck, 'latency_check')

    num_queries=1000
    latencies=[]
    for i in range(num_queries):
        req = LatencyCheck.Request()
        req.query_time=node.get_clock().now().to_msg()
        while not cli.wait_for_service(timeout_sec=0.0):
            pass

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future)

        try:
            result = future.result()
            return_time=result.return_time
        except Exception as e:
            node.get_logger().info('Service call failed %r' % (e,))
        else:
            current_time=node.get_clock().now().to_msg()
            latency=(current_time.nanosec+current_time.sec*(10**9))-(return_time.nanosec+return_time.sec*(10**9))
            latency/=10**6
            latencies.append(latency)
            print("Latency obtained = ",latency)

    node.destroy_node()
    rclpy.shutdown()

    mean=sum(latencies[1:])/len(latencies[1:])
    standard_dev=sum([((x - mean) ** 2) for x in latencies[1:]]) / len(latencies[1:])    

    print("Mean = ",mean)
    print("Error Bar = ",mean+standard_dev)

    fig_handle = plt.figure()
    ax=plt.subplot()
    ax.plot(latencies)
    ax.set(xlabel='Query Number',ylabel='Latency (in ms) ',title='Latency Plot')
    plt.xticks(np.arange(0, len(latencies)+1, len(latencies)/20))
    ax.grid()

    pl.dump(fig_handle, open('ros2_latency.pickle','wb'), protocol=2)

    plt.show()


if __name__ == '__main__':
    main()
