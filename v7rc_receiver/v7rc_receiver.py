import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial
import socket
import threading
from collections import deque


class V7RCReceiver(Node):
    def __init__(self, type):
        super().__init__('v7rc_receiver')

        # 建立 cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 使用 deque 儲存接收到的最新控制值（CH1, CH2）
        self.control_queue = deque(maxlen=1)
        self.control_queue.append((1500, 1500))  # 初始值（中立）

        # Timer to print the status if receiving command
        self.last_receive_time = 0

        if(type == 0):
            # 啟動 UDP 接收執行緒
            self.udp_thread = threading.Thread(target=self.udp_receiver_thread, daemon=True)
            self.udp_thread.start()
        elif(type == 1):
            # Run Serial Thread
            self.uart_thread = threading.Thread(target=self.uart_receiver_thread, daemon=True)
            self.uart_thread.start()

        # 每 50ms 發布一次 /cmd_vel
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)

        # Max control value can received from arguments
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('max_linear', 0.5)
        # Pass arguments
        self.max_angular = self.get_parameter('max_angular').value
        self.max_linear = self.get_parameter('max_linear').value
        self.get_logger().warn(f"Max *Linear* speed = {self.max_linear} m/s, make sure it will not too fast!")
        self.get_logger().warn(f"Max *Angular* speed = {self.max_angular} rad/s, make sure it will not too fast!")

    # A thread to handle the UDP remote packet
    def udp_receiver_thread(self):
        # Listening IP and port (0.0.0.0 for all interface)
        udp_ip = '0.0.0.0'
        udp_port = 6188
        # Create socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((udp_ip, udp_port))
        self.get_logger().info(f"UDP listening on {udp_ip}:{udp_port}")
        # Flag to vaildate the data correct or not
        invaild = False
        # Loop for receive the signal
        while True:
            try:
                # Receiving data
                data, addr = sock.recvfrom(1024)
                # Decode data
                msg = data.decode('utf-8').strip()
                # Received data, get the wall clock
                now_time = self.get_clock().now().nanoseconds
                # Hint user if still get invaild flag
                if((now_time - self.last_receive_time) / 1e6 > 3000 and invaild):
                    self.get_logger().warn(f"Data invaild detected! Check remote software's setting or connection")
                    self.last_receive_time = now_time
                # Uncorrect header, uncorrect length, uncorrect footer, flip the flag
                if not (msg.startswith("SRV") or len(msg) < 20 or msg.endswith("#")):
                    invaild = True
                    continue
                # 解析 CH1 / CH2（格式：SRV1000200015001500#）
                ch1 = int(msg[3:7])
                ch2 = int(msg[7:11])
                # Append it to deque, to processing by main thread
                self.control_queue.append((ch1, ch2))
                # 接收期間每3秒印一次提示，讓使用者知道資料仍在接收中(Heartbeat)
                if((now_time - self.last_receive_time) / 1e6 > 3000):
                    self.get_logger().info(f"Got new control signal from {addr}!")
                    # Update timer
                    self.last_receive_time = now_time
                # Data vaild, so put down the flag
                invaild = False
            # If it have another exception
            except Exception as e:
                self.get_logger().warn(f"UDP receive error: {e}")

    # A thread to handle the UDP remote packet
    def uart_receiver_thread(self):
        # UART Port and baud
        uart_port = '/dev/v7rc_controller'
        uart_baud = 115200
        # Flag to empty data
        not_clear_data = True
        # Flag to vaildate the data correct or not
        invaild = False
        # Open port
        ser = serial.Serial(uart_port, uart_baud, timeout=0.01)
        self.get_logger().info(f"Serial port opened {uart_port}:{uart_baud}")
        # Loop for receive the signal
        while True:
            try:
                # Current timestamp
                now_time = self.get_clock().now().nanoseconds
                # Hint user if still get invaild flag
                if((now_time - self.last_receive_time) / 1e6 > 3000 and invaild):
                    self.get_logger().warn(f"Data invaild detected! Check remote software's setting or connection")
                    self.last_receive_time = now_time
                if not_clear_data:
                    # Maybe have remaining data from the MCU's buffer, clear it!
                    self.get_logger().warn(f"Clear remaining datas... Don't connect the software!")
                    in_waiting = 0
                    while(True):
                        buf = ser.read(1)
                        in_waiting += ser.in_waiting + len(buf)
                        if(in_waiting > 0):
                            ser.reset_input_buffer()
                            ser.reset_output_buffer()
                        else:
                            self.get_logger().info(f"Already clear remaining {in_waiting} bytes data in serial!")
                            not_clear_data = False
                            break
                else:
                    # Receiving data
                    # Packet format: SRV1000200015001500
                    # V7RC's header: SRV and #
                    # Block if it not have in waiting data to prevent high cpu usage
                    # Flag to vaildate the data correct or not
                    invaild = False
                    # Read first byte
                    header = ser.read(1)
                    # Not equal the packet head, skip this package
                    if header != b'S' and len(header) > 0:
                        invaild = True
                        continue
                    # Continue read the header
                    header += ser.read(2)
                    # Check header vaild
                    if header != b'SRV':
                        invaild = True
                        continue
                    # If length not match
                    if ser.in_waiting < 17:
                        invaild = True
                        continue
                    # Read remaining 17 bytes (to #)
                    msg = ser.read(17)
                    # Footer check
                    if msg[-1:] != b'#':
                        invaild = True
                        continue
                    # Decode data
                    msg = msg.decode()
                    # 解析 CH1 / CH2（格式：SRV1000200015001500#）
                    ch1 = int(msg[0:4])
                    ch2 = int(msg[4:8])
                    # Append it to deque, to processing by main thread
                    self.control_queue.append((ch1, ch2))
                    # 接收期間每3秒印一次提示，讓使用者知道資料仍在接收中(Heartbeat)
                    if((now_time - self.last_receive_time) / 1e6 > 3000):
                        self.get_logger().info(f"Got new control signal from {uart_port}!")
                        # Update timer
                        self.last_receive_time = now_time
                    # Data vaild, so put down the flag
                    invaild = False
            # If it have another exception
            except Exception as e:
                self.get_logger().warn(f"UART receive error: {e}")

    # Send command when timer fired 
    def publish_cmd_vel(self):
        # If have no new command, just finish this run
        if len(self.control_queue) == 0:
            return
        # Get the data from deque
        ch1, ch2 = self.control_queue[-1]
        # Empty deque
        self.control_queue.clear()
        # CH1：水平轉向（angular.z），CH2：前後速度（linear.x）
        # 以 1000 ~ 2000 對應到指令範圍
        # Scaling linear
        def scale_linear(val):
            # (val - 1500) / 500.0 will scale value to +1 to -1
            return max(min((val - 1500) / 500.0 * self.max_linear, self.max_linear), (self.max_linear * -1.0))
        # Scaling angular
        def scale_angular(val):
            return max(min((val - 1500) / 500.0 * self.max_angular, self.max_angular), (self.max_angular * -1.0))
        # Make twist message
        twist = Twist()
        # Put the received command in
        twist.linear.x = scale_linear(ch2)
        twist.angular.z = -scale_angular(ch1)
        # Publish message
        self.cmd_pub.publish(twist)


def main_udp(args=None):
    rclpy.init(args=args)
    node = V7RCReceiver(type=0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main_uart(args=None):
    rclpy.init(args=args)
    node = V7RCReceiver(type=1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main_udp()
