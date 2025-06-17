# ROS 2 V7RC Receiver
Use V7RC app as the ROS car's controller!  
Supports WiFi UDP direct connect or BLE -> Serial bridged connection

* It's a typical `ament_python` ROS2 package, ensure your Python have basic `UDP socket` and `deque` support, just clone, build and launch it!
* 這是一個標準的`ament_python` ROS2 套件，只要確認你的 Python 支援基本的 `UDP Socket` 以及 `deque` ， clone 這個套件與編譯，即可執行與使用！

## Build and Run it

1. Enter your workspace
    ```
    cd ~/ros2_ws/src
    ```
2. Clone this repo under `src` folder!
    ```
    git clone https://github.com/Bob-YsPan/v7rc_receiver_ros2.git v7rc_receiver
    ```
3. Build package
    ```
    cd ~/ros2_ws
    colcon build --packages-select v7rc_receiver
    ```
4. Run it!
    ```
    # Start by UDP protocol
    ros2 run v7rc_receiver v7rc_udp_receiver
    # Start by UART protocol (Need a MCU do the convertion, make PC can receive control protocol like BLE)
    ros2 run v7rc_receiver v7rc_uart_receiver
    ```
   You can run it by pass the maxinum speed! (Default: `0.5 m/s linear, 1.0 rad/s angular`)
    ```
    ros2 run v7rc_receiver v7rc_udp_receiver --ros-args -p max_angular:=2.5 -p max_linear:=1.0
    ```
5. (If you are using UDP mode) Find your PC's IP that run this node, connect your phone under same subnet, and then set your V7RC app's connect mode to Wi-Fi, then fill in the setting:
    * IP: `Your PC's IP`
    * Port(連接埠): `6188` (Default)
6. (If you are using UART mode) UDEV rules for Raspberry Pi Pico W to receives the controls, you can replace vendor id and product id to meet your needs:
    `/etc/udev/rules.d/v7rc_receiver.rules`
    ```
    KERNEL=="ttyACM*", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0005", MODE:="0666", SYMLINK+="v7rc_controller"
    ```
    Sample files can be found at https://github.com/Bob-YsPan/v7rc_receiver_ros2/tree/main/Pico_W_V7RC_ROS_micropython, you can directly upload to your micropython board, if you want to write your own, **make sure you just prints the V7RC's command at serial (no line break at each command), or you will got lot's of the warning message!**
7. It will auto connect, back to the remote screen, the log will like this:
    ```
    [INFO] [1749478043.049440469] [v7rc_receiver]: UDP listening on 0.0.0.0:6188
    [WARN] [1750146930.766355076] [v7rc_receiver]: Max *Linear* speed = 0.5 m/s, make sure it will not too fast!
    [WARN] [1750146930.767324150] [v7rc_receiver]: Max *Angular* speed = 1.0 rad/s, make sure it will not too fast!
    [INFO] [1749478064.075538138] [v7rc_receiver]: Got new control signal from ('192.168.1.33', 6188)!
    [INFO] [1749478067.096427642] [v7rc_receiver]: Got new control signal from ('192.168.1.33', 6188)!
    [INFO] [1749478070.100631637] [v7rc_receiver]: Got new control signal from ('192.168.1.33', 6188)!
    [INFO] [1749478073.101192725] [v7rc_receiver]: Got new control signal from ('192.168.1.33', 6188)!    < Note: This log will print each 3 seconds if app is running at remote screen!
    ```
    Or like this when at UART mode  
    **Note: It will got some warning message if uses my sample micropython program because the message print by micropython or other message not belongs to V7RC, just ignore it if the control signal can receive correctly!**
    ```
    [INFO] [1750146930.765994766] [v7rc_receiver]: Serial port opened /dev/v7rc_controller:115200
    [WARN] [1750146930.766355076] [v7rc_receiver]: Max *Linear* speed = 0.5 m/s, make sure it will not too fast!
    [INFO] [1750146930.767139088] [v7rc_receiver]: Clear remaining datas...
    [WARN] [1750146930.767324150] [v7rc_receiver]: Max *Angular* speed = 1.0 rad/s, make sure it will not too fast!
    [INFO] [1750146931.769270955] [v7rc_receiver]: Already clear remaining 0 bytes data in serial!
    [INFO] [1750146936.803052883] [v7rc_receiver]: Got new control signal from /dev/v7rc_controller!
    [INFO] [1750146939.832484817] [v7rc_receiver]: Got new control signal from /dev/v7rc_controller!
    [INFO] [1750146942.862727636] [v7rc_receiver]: Got new control signal from /dev/v7rc_controller!    < Note: This log will print each 3 seconds if app is running at remote screen!
    ```
   If you got this error, make sure your V7RC is in `CAR (車輛)` mode!
    ```
    [WARN] [1749478077.281304232] [v7rc_udp_receiver]: Data Invaild! Ensure is it in correct mode or corrcct remote software!
    [WARN] [1749478080.300307321] [v7rc_udp_receiver]: Data Invaild! Ensure is it in correct mode or corrcct remote software!
    [WARN] [1749478083.330704208] [v7rc_udp_receiver]: Data Invaild! Ensure is it in correct mode or corrcct remote software!
    ```
8. Run others node to launch the robot, try to control and enjoy it!
