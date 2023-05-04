from std_srvs.srv import Empty,Trigger

import rclpy
import time


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('drone_service_client')
    cli = node.create_client(Empty, 'takeoff')
    req = Empty.Request()
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("takeoff successful")

    time.sleep(5)

    battery = 100
    while battery > 20:
        cli = node.create_client(Trigger, 'battery')
        req = Trigger.Request()
        while not cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service not available, waiting again...')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        try:
            result = future.result()

        except Exception as e:
            node.get_logger().info('Service call failed %r' % (e,))
        else:
            node.get_logger().info("Battery successful")
            battery = int(result.message)
            node.get_logger().info(f'Battery level is: {battery}')
        time.sleep(15)
    
    # cli = node.create_client(Empty, 'up')
    # req = Empty.Request()
    # while not cli.wait_for_service(timeout_sec=1.0):
    #     node.get_logger().info('service not available, waiting again...')

    # future = cli.call_async(req)
    # rclpy.spin_until_future_complete(node, future)

    # try:
    #     result = future.result()
    # except Exception as e:
    #     node.get_logger().info('Service call failed %r' % (e,))
    # else:
    #     node.get_logger().info("up successful")

    
    cli = node.create_client(Empty, 'land')
    req = Empty.Request()
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("land successful")

        

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()