from machine import Pin, SPI, Timer
from umqtt.simple import MQTTClient
from neopixel import NeoPixel
import network, time, ujson, _thread

led = Pin(25, Pin.OUT)

MOSI    = 19
MISO    = 16
SCK     = 18
CS      = 17
RST     = 20
BROKER  = "demo.thingsboard.io"
TOKEN   = "KzkFxGtN076bsPJz3Lvw"

def subcribe_callback(topic, msg):
    try:
        data = ujson.loads(msg)
        for k, v in data.items():
            print("[MQTT SUB] topic ", topic, " attribute:", k, "value:", v)
            if k == "value":
                Pin(25, Pin.OUT).value(v)
            elif k == "rgb":
                rgb = NeoPixel(Pin(23, Pin.OUT), 1)
                if v == "R":
                    rgb.fill((255, 0, 0))
                elif v == "G":
                    rgb.fill((0, 255, 0))
                elif v == "B":
                    rgb.fill((0, 0, 255))
                elif v == "T":
                    rgb.fill((0, 0, 0))
                rgb.write()
    except Exception as e:
        print("[MQTT SUB ERROR]", e)

def mqtt_connect():
    global mqtt_client
    while True:
        try:
            mqtt_client = MQTTClient(client_id="w55000", server=BROKER, port=1883, user=TOKEN, password="")
            mqtt_client.connect()
            mqtt_client.set_callback(subcribe_callback)
            mqtt_client.subscribe(b"v1/devices/me/attributes")
            time.sleep(1)
            print("[MQTT] Connected and subscribed")
            break
        except Exception as e:
            print("[MQTT] Connect failed, retrying in 3s:", e)
            time.sleep(3)

def w5x00_init():
    spi=SPI(0, 80_000_000, mosi = Pin(MOSI), miso = Pin(MISO), sck = Pin(SCK))
    Pin(RST, Pin.OUT).value(0)
    time.sleep_ms(500)
    Pin(RST, Pin.OUT).value(1)
    time.sleep_ms(500)
    Pin(RST, Pin.OUT).value(0)
    time.sleep_ms(500)
    nic = network.WIZNET5K(spi, Pin(CS), Pin(RST)) 
    nic.active(True)
    try:
        nic.ifconfig('dhcp')
        print(f"DHCP: {nic.status()}")
    except:
        nic.ifconfig(('10.42.0.2', '255.255.255.0', '10.42.0.1', '8.8.8.8'))
        print("Config static dynamic")
    while not nic.isconnected():
        time.sleep(1)
        print(nic.regs())
        print("Wating connect internet...")
    print("W5500 connect internet success:", nic.ifconfig())
    print("IP Address:",nic.ifconfig()[0])
    print("Subnet Mask:",nic.ifconfig()[1])
    print("Gateway:",nic.ifconfig()[2])
    print("DNS:",nic.ifconfig()[3])
    return nic
    

def listen(log: str):
    while True:
        try:
            mqtt_client.check_msg()
        except Exception as e:
            print(f"[{log}] MQTT check_msg error, reconnecting:", e)
            mqtt_connect()

def publish(timer):
    global value
    try:
        payload = '{"temperature":%.2f}' % value
        mqtt_client.publish(topic, payload)
        print(f"[TIMER] topic {topic} payload: {payload}")
        value += 10.0
        if value > 100.0:
            value = 10.0
    except Exception as e:
        print(f"[TIMER] MQTT publish error, reconnecting:", e)
        mqtt_connect()


if __name__ == "__main__":
    nic = w5x00_init()
    mqtt_connect()
    _thread.start_new_thread(listen, ("LISTEN", ))
    topic = b"v1/devices/me/telemetry"
    value = 10.0
    tim = Timer()
    tim.init(period=5000, mode=Timer.PERIODIC, callback=publish)