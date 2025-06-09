import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import socket
import threading
from collections import deque


class V7RCReceiver(Node):
    def __init__(self):
        super().__init__('v7rc_udp_receiver')

        # 建立 cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 使用 deque 儲存接收到的最新控制值（CH1, CH2）
        self.control_queue = deque(maxlen=1)
        self.control_queue.append((1500, 1500))  # 初始值（中立）

        # 啟動 UDP 接收執行緒
        self.udp_thread = threading.Thread(target=self.udp_receiver_thread, daemon=True)
        self.udp_thread.start()

        # 每 50ms 發布一次 /cmd_vel
        self.timer = self.create_timer(0.05, self.publish_cmd_vel)

        # Max control value can received from arguments
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('max_linear', 0.5)
        # Pass arguments
        self.max_angular = self.get_parameter('max_angular').value
        self.max_linear = self.get_parameter('max_linear').value

        # Timer to print the status if receiving command
        self.last_receive_time = 0

    # A thread to handle the UDP remote packet
    def udp_receiver_thread(self):
        # Listening IP and port (0.0.0.0 for all interface)
        udp_ip = '0.0.0.0'
        udp_port = 6188
        # Create socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((udp_ip, udp_port))
        self.get_logger().info(f"UDP listening on {udp_ip}:{udp_port}")
        # Loop for receive the signal
        while True:
            try:
                # Receiving data
                data, addr = sock.recvfrom(1024)
                # Decode data
                msg = data.decode('utf-8').strip()
                # Received data, get the wall clock
                now_time = self.get_clock().now().nanoseconds
                # Flag to vaildate the data correct or not
                invaild = False
                # Uncorrect header, flip the flag, and hint the user at heartbeat
                if not msg.startswith("SRV") or len(msg) < 11:
                    invaild = True
                # 接收期間每3秒印一次提示，讓使用者知道資料仍在接收中(Heartbeat)
                if((now_time - self.last_receive_time) / 1e6 > 3000):
                    self.get_logger().info(f"Got new control signal from {addr}!")
                    # If last data still invaild, print the warning message
                    if invaild:
                        self.get_logger().warn("Data Invaild! Ensure is it in correct mode or corrcct remote software!")
                    # Update timer
                    self.last_receive_time = now_time
                # 解析 CH1 / CH2（格式：SRV1000200015001500#）
                ch1 = int(msg[3:7])
                ch2 = int(msg[7:11])
                # Append it to deque, to processing by main thread
                self.control_queue.append((ch1, ch2))
            # If it have another exception
            except Exception as e:
                self.get_logger().warn(f"UDP receive error: {e}")
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
            return max(min((val - 1500) / 500.0, self.max_linear), (self.max_linear * -1.0))
        # Scaling angular
        def scale_angular(val):
            return max(min((val - 1500) / 500.0, self.max_angular), (self.max_angular * -1.0))
        # Make twist message
        twist = Twist()
        # Put the received command in
        twist.linear.x = scale_linear(ch2)
        twist.angular.z = -scale_angular(ch1)
        # Publish message
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = V7RCReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
