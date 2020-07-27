#!/usr/bin/env python

from devel_interfaces.srv import LatencyCheck

import rclpy

g_node = None

def Handle_time(request,response):
    global g_node
    response.return_time=request.query_time
    response.return_load=request.query_load
    print("Incoming request ",request.query_time)
    return response

def main(args=None):
    global g_node
    rclpy.init(args=args)

    g_node = rclpy.create_node('latency_check_service')

    srv = g_node.create_service(LatencyCheck, 'latency_check', Handle_time)
    while rclpy.ok():
        rclpy.spin_once(g_node)

    g_node.destroy_service(srv)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
