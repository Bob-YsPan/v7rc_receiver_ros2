import ble_uart_v7rc
import bluetooth
from machine import Pin

# 初始化LED
led = Pin('LED', Pin.OUT)

# 剖析資料
def data_process(data: str):
    print(f"{data}", end="")
        
# 定義接收回呼函數，僅印出接收到的字串
def rx_callback():
    led.on()
    data = uart.read().decode().strip()  # 讀取並解碼接收到的 BLE 資料
    #print('Received:', str(data))
    data_process(data=data)
    led.off()

# 初始化 BLE 和 UART 類別
ble = bluetooth.BLE()
print(f"V7RC Receiver Starting...")
uart = ble_uart_v7rc.ble_uart(ble=ble, rx_callback=rx_callback, name="PicoW ROS Controller")
