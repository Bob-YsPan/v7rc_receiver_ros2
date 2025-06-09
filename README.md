# ROS 2 V7RC Receiver
Use V7RC app's WiFi-UDP mode as the ROS car's controller!

* It's a typical `ament_python` ROS2 package, ensure your Python have basic `UDP socket` and `deque` support, just clone, build and launch it!
* 這是一個標準的`ament_python` ROS2 套件，只要確認你的 Python 支援基本的 `UDP Socket` 以及 `deque` ， clone 這個套件與編譯，即可執行與使用！

## Build and Run it

1. Enter your workspace
    ```
    cd ~/ros2_ws/src
    ```
2. Clone this repo under `src` folder!
3. Build package
    ```
    cd ~/ros2_ws
    colcon build --packages-select v7rc_udp_receiver
    ```
4. Run it!
    ```
    ros2 run v7rc_udp_receiver v7rc_udp_receiver
    ```
   You can run it by pass the maxinum speed! (Default: `0.5 m/s linear, 1.0 rad/s angular`)
    ```
    ros2 run v7rc_udp_receiver v7rc_udp_receiver --ros-args -p max_angular:=2.5 -p max_linear:=1.0
    ```
5. Find your PC's IP that run this node, connect your phone under same subnet, and then set your V7RC app's connect mode to Wi-Fi, then fill in the setting:
    * IP: `Your PC's IP`
    * Port(連接埠): `6188` (Default)
6. It will auto connect, back to the remote screen, make sure the log like this:
    ```
    [INFO] [1749478043.049440469] [v7rc_udp_receiver]: UDP listening on 0.0.0.0:6188
    [INFO] [1749478064.075538138] [v7rc_udp_receiver]: Got new control signal from ('192.168.1.33', 6188)!
    [INFO] [1749478067.096427642] [v7rc_udp_receiver]: Got new control signal from ('192.168.1.33', 6188)!
    [INFO] [1749478070.100631637] [v7rc_udp_receiver]: Got new control signal from ('192.168.1.33', 6188)!
    [INFO] [1749478073.101192725] [v7rc_udp_receiver]: Got new control signal from ('192.168.1.33', 6188)!    < Note: This log will print each 3 seconds if app is running at remote screen!
    ```
   If you got this error, make sure your V7RC is in `CAR (車輛)` mode!
    ```
    [INFO] [1749478077.280239942] [v7rc_udp_receiver]: Got new control signal from ('192.168.1.33', 6188)!
    [WARN] [1749478077.281304232] [v7rc_udp_receiver]: Data Invaild! Ensure is it in correct mode or corrcct remote software!
    [INFO] [1749478080.299411753] [v7rc_udp_receiver]: Got new control signal from ('192.168.1.33', 6188)!
    [WARN] [1749478080.300307321] [v7rc_udp_receiver]: Data Invaild! Ensure is it in correct mode or corrcct remote software!
    [INFO] [1749478083.329775143] [v7rc_udp_receiver]: Got new control signal from ('192.168.1.33', 6188)!
    [WARN] [1749478083.330704208] [v7rc_udp_receiver]: Data Invaild! Ensure is it in correct mode or corrcct remote software!
    ```
7. Run others node to launch the robot, try to control and enjoy it!
