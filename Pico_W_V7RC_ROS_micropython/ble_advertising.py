# 書中第 6.1 節的程式碼
# 用於產生 BLE 廣告封包的輔助程式。

from micropython import const
import struct
import bluetooth

# 廣告封包的結構為：
#   1 個位元組的資料長度 (N + 1)
#   1 個位元組的類型 (參見下列常數)
#   N 個位元組的類型相關資料

# 定義廣告類型常數
_ADV_TYPE_FLAGS = const(0x01)
_ADV_TYPE_NAME = const(0x09)
_ADV_TYPE_UUID16_COMPLETE = const(0x03)
_ADV_TYPE_UUID32_COMPLETE = const(0x05)
_ADV_TYPE_UUID128_COMPLETE = const(0x07)
_ADV_TYPE_UUID16_MORE = const(0x02)
_ADV_TYPE_UUID32_MORE = const(0x04)
_ADV_TYPE_UUID128_MORE = const(0x06)
_ADV_TYPE_APPEARANCE = const(0x19)

# 產生廣告封包，供 gap_advertise(adv_data=...) 使用
def advertising_payload(limited_disc=False, br_edr=False, name=None, services=None, appearance=0):
    payload = bytearray()

    def _append(adv_type, value):
        # 將資料附加至廣告封包
        nonlocal payload
        payload += struct.pack("BB", len(value) + 1, adv_type) + value

    # 添加基本的廣告標誌
    _append(
        _ADV_TYPE_FLAGS,
        struct.pack("B", (0x01 if limited_disc else 0x02) + (0x18 if br_edr else 0x04)),
    )

    # 添加名稱至廣告封包
    if name:
        _append(_ADV_TYPE_NAME, name)

    # 添加服務 UUID 至廣告封包
    if services:
        for uuid in services:
            b = bytes(uuid)
            if len(b) == 2:
                _append(_ADV_TYPE_UUID16_COMPLETE, b)
            elif len(b) == 4:
                _append(_ADV_TYPE_UUID32_COMPLETE, b)
            elif len(b) == 16:
                _append(_ADV_TYPE_UUID128_COMPLETE, b)

    # 設定外觀 (appearance)
    if appearance:
        _append(_ADV_TYPE_APPEARANCE, struct.pack("<h", appearance))

    return payload

# 產生回應封包，供 gap_advertise(adv_data=...) 使用
def advertising_resp_payload(name=None, services=None):
    # 產生掃描回應的封包
    payload = bytearray()

    def _append(adv_type, value):
        # 將資料附加至回應封包
        nonlocal payload
        payload += struct.pack("BB", len(value) + 1, adv_type) + value

    # 添加名稱至回應封包
    if name:
        _append(_ADV_TYPE_NAME, name)

    # 添加服務 UUID 至回應封包
    if services:
        for uuid in services:
            b = bytes(uuid)
            if len(b) == 2:
                _append(_ADV_TYPE_UUID16_COMPLETE, b)
            elif len(b) == 4:
                _append(_ADV_TYPE_UUID32_COMPLETE, b)
            elif len(b) == 16:
                _append(_ADV_TYPE_UUID128_COMPLETE, b)

    return payload

# 解碼指定類型的廣告資料欄位
def decode_field(payload, adv_type):
    i = 0
    result = []
    while i + 1 < len(payload):
        if payload[i + 1] == adv_type:
            result.append(payload[i + 2 : i + payload[i] + 1])
        i += 1 + payload[i]
    return result

# 解碼廣告封包中的名稱
def decode_name(payload):
    n = decode_field(payload, _ADV_TYPE_NAME)
    return str(n[0], "utf-8") if n else ""

# 解碼廣告封包中的服務 UUID
def decode_services(payload):
    services = []
    for u in decode_field(payload, _ADV_TYPE_UUID16_COMPLETE):
        services.append(bluetooth.UUID(struct.unpack("<h", u)[0]))
    for u in decode_field(payload, _ADV_TYPE_UUID32_COMPLETE):
        services.append(bluetooth.UUID(struct.unpack("<d", u)[0]))
    for u in decode_field(payload, _ADV_TYPE_UUID128_COMPLETE):
        services.append(bluetooth.UUID(u))
    return services

# 範例測試函數
def demo():
    # 產生一個廣告封包範例
    payload = advertising_payload(
        name="micropython",
        services=[bluetooth.UUID(0x181A), bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")],
    )
    print(payload)
    print(decode_name(payload))
    print(decode_services(payload))

if __name__ == "__main__":
    demo()
