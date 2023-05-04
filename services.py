import socket
import threading
import time

import rclpy
from rclpy.node import Node



from std_srvs.srv import Empty, Trigger

class DroneServer(Node):
    drone_response = "no_response"
    battery_level=100
    def __init__(self):
        super().__init__('drone_server')
        
        
        self.local_ip = ''
        self.local_port = 8889
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd
        self.socket.bind((self.local_ip, self.local_port))

        # thread for receiving cmd ack
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        self.tello_ip = '192.168.10.1'
        self.tello_port = 8889
        self.tello_address = (self.tello_ip, self.tello_port)
        self.MAX_TIME_OUT = 15.0
         
        # self.timer = self.create_timer(15.0, self.up_callback)
        #self.last_command_time = time.time()
        
        self.srv = self.create_service(Empty, 'takeoff', self.takeoff_callback)
        #self.srv = self.create_service(Empty, 'land', self.land_callback)
        self.srv = self.create_service(Trigger, 'battery', self.battery_callback)
        
        
        
        
        
        
    def send_command(self, msg):
        command = msg  # the actual command string

        self.get_logger().info('I heard: "%s"' % msg)
        self.socket.sendto(command.encode('utf-8'), self.tello_address)
        print('sending command: %s to %s' % (command, self.tello_ip))

        
        start = time.time()
        now = time.time()
        diff = now - start
        if diff > self.MAX_TIME_OUT:
            print('Max timeout exceeded... command %s' % command)
            return
        print('Done!!! sent command: %s to %s' % (command, self.tello_ip))
    
    def takeoff_callback(self, request, response):
        self.get_logger().info('Incoming request: Takeoff')
        command = "takeoff"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        return response
        
    '''def land_callback(self, request, response):
        self.get_logger().info('Incoming request: Land')
        command = "land"
        print(command)
        self.send_command("command")
        time.sleep(2)
        self.send_command(command)
        return response'''
    
    # def up_callback(self, request, response):
    #     current_time = time.time()
    #     if (current_time - self.last_command_time) > 15.0:
    #         self.get_logger().info('Incoming request: up')
    #         command = "up"
    #         print(command)
    #         self.send_command("command")
    #         time.sleep(2)
    #         self.send_command(command)
    #         self.last_command_time = current_time
    #     return response
        
    
    def battery_callback(self, request, response):
        global drone_response
        self.get_logger().info('Incoming request: Battery')
        command="battery?"
        print(command)
        self.send_command("command")
        time.sleep(1)
        self.send_command(command)
        time.sleep(1)
        response.message=drone_response.strip("b'")[0:2]
        #battery=int(response.message)
        if(int(response.message)<20):
            self.get_logger().info('Battery low! landing')
            command = "land"
            print(command)
            self.send_command("command")
            time.sleep(2)
            self.send_command(command)
            
        else:
            self.get_logger().info('Battery level is not below than 20%')
        return response
    

    
    
    def _receive_thread(self):
        global drone_response
        #Listen to responses from the Tello.
        while True:
            try:
                self.response, ip = self.socket.recvfrom(1024)
                print('from %s: %s' % (ip, self.response))
                drone_response = str(self.response) #convert from byte string to string
            except (socket.error, exc):
                print("Caught exception socket.error : %s" % exc)

                

def main(args=None):
    rclpy.init(args=args)

    node = DroneServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - Done automatically when node is garbage collected)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()