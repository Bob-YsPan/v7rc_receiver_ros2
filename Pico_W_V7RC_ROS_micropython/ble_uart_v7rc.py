# 由 Mason 更新，用於 Android 裝置的 BLE UART 程式碼
import bluetooth
from ble_advertising import advertising_payload
from ble_advertising import advertising_resp_payload
from micropython import const
from machine import UART, Pin

# 定義 BLE 特徵標誌
_FLAG_WRITE = const(0x0008)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_NOTIFY = const(0x0010)

led = Pin('LED', Pin.OUT)

# 定義 UART 服務和特徵 UUID
_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"), _FLAG_NOTIFY,)
_UART_RX = (bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"), _FLAG_WRITE_NO_RESPONSE,)
_UART_SERVICE = (_UART_UUID, (_UART_TX, _UART_RX),)

# 定義 BLE 事件常數
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)
_IRQ_CONNECTION_UPDATE = const(27)

# BLE 外觀設定
_ADV_APPEARANCE_GENERIC_COMPUTER = const(128)
_ADV_APPEARANCE_GENERIC_PHONE = const(64)

class ble_uart:
    def __init__(self, ble, rx_callback=None, name="ble-uart", rxbuf=100):
        # 初始化 BLE UART 類別
        self._ble = ble
        self._ble.active(True)
        self._ble.config(gap_name=name)  # 設定 GAP 名稱
        print("BLE Started!")

        # 設定 BLE 讀寫和通知方法
        self._write = self._ble.gatts_write
        self._read = self._ble.gatts_read
        self._notify = self._ble.gatts_notify

        # 註冊 UART 服務
        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._ble.gatts_set_buffer(self._rx_handle, rxbuf, True)
        self._connections = set()
        self._rx_buffer = bytearray()
        self._handler = None
        self._ble.irq(self._irq)
        self._payload = advertising_payload(services=(_UART_UUID,), appearance=_ADV_APPEARANCE_GENERIC_PHONE)
        self._resp_payload = advertising_resp_payload(name=name)
        self._advertise()
        
        # 初始化其他變數
        self.irq(handler=rx_callback)

    def irq(self, handler):
        # 設定 IRQ 處理器
        self._handler = handler

    def _irq(self, event, data):  # BLE 中斷處理函數
        if event == _IRQ_CENTRAL_CONNECT:
            # 當中央裝置連接時
            conn_handle, addr_type, addr, = data
            self._connections.add(conn_handle)
            print("Connected!")
            led.off()
        elif event == _IRQ_CENTRAL_DISCONNECT:
            # 當中央裝置斷開連接時
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)
            self._advertise()  # 重新開始廣播
        elif event == _IRQ_GATTS_WRITE:
            # 當有資料寫入時
            conn_handle, attr_handle = data
            if conn_handle in self._connections:
                self._rx_buffer += self._ble.gatts_read(self._rx_handle)
                if self._handler:
                    self._handler()

    def read(self, sz=None):
        # 讀取接收到的資料
        if not sz:
            sz = len(self._rx_buffer)
        result = self._rx_buffer[0:sz]
        self._rx_buffer = self._rx_buffer[sz:]
        return result

    def write(self, data):
        # 寫入資料並通知所有連接的裝置
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._tx_handle, data)

    def close(self):
        # 關閉所有連接
        for conn_handle in self._connections:
            self._ble.gap_disconnect(conn_handle)
        self._connections.clear()

    def _advertise(self, interval_us=500000):
        # 開始 BLE 廣播
        self._ble.gap_advertise(None)  # 停止之前的廣播
        self._ble.gap_advertise(interval_us, adv_data=self._payload, resp_data=self._resp_payload)
        print("Advertising...")
        led.on()
