#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from interfaces.srv import ArduinoCall  # Custom service type (ensure you have this defined in your package)

class ArduinoListener(Node):
    def __init__(self):
        super().__init__('arduino_listener')

        # Create a publisher to publish Arduino data
        self.publisher_ = self.create_publisher(String, 'arduino_data', 10)
        
        # Create a service server for handling requests
        self.srv = self.create_service(ArduinoCall, 'arduino_service', self.handle_arduino_service)
        
        # Open the serial port
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.timer = self.create_timer(0.1, self.read_serial_data)  # Timer for polling the serial port

        # Store the last state of the Electromagnet
        self.Electromagnet_state = None

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            # Read a line from the serial port
            line = self.ser.readline().decode('utf-8').rstrip()
            # self.get_logger().info(f'Received: {line}')
            # Publish the data to the ROS 2 topic
            msg = String()
            msg.data = line
            self.publisher_.publish(msg)

            # Update the Electromagnet state based on received data
            if "Electromagnet on without brick" in line:
                self.Electromagnet_state = "Electromagnet on without brick"  #30mt
            elif "Electromagnet off without brick" in line:
                self.Electromagnet_state = "Electromagnet off without brick" #0mt
            elif "Electromagnet on with brick" in line:
                self.Electromagnet_state = "Electromagnet on with brick"    #40mt
            elif "Electromagnet off with brick" in line:
                self.Electromagnet_state = "Electromagnet off with brick"   #10mt

    def handle_arduino_service(self, request, response):
        # Send serial command to the Arduino based on the request
        if request.command == "Free Brick Position Reached":
            self.ser.write(b'ON\n')  # Command to turn electromagnet ON
            response.response = "Electromagnet ON command sent."
        elif request.command == "Target Brick Position Reached":
            self.ser.write(b'OFF\n')  # Command to turn electromagnet OFF
            response.response = "Electromagnet OFF command sent."
        else:
            response.response = "Invalid command received."


        # Respond with the current Electromagnet state
        if self.Electromagnet_state:
            response.response = self.Electromagnet_state
        else:
            response.response = "No valid data received from the Arduino."

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
