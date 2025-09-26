"""
Mode TX register map pin SPI0
CE      GPIO6
CSN     GPIO5
SCK     GPIO2
MOSI    GPIO3
MISO    GPIO4
IRQ     None
"""

"""
Mode RX register map pin SPI1
CE      GPIO9
CSN     GPIO13
SCK     GPIO10
MOSI    GPIO11
MISO    GPIO12
IRQ     None
"""

"""
Config SPI  and test SPI
spi0 = SPI(0, baudrate=80_000_000, polarity=0, phase=0, bits=8, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
csn0 = Pin(5, Pin.OUT)
csn0.value(1)
csn0.value(0)
spi0.write(bytearray([0x00 | 0x00]))
buf = spi0.read(1)
csn0.value(1)
print("Received 0:", buf[0]))spi1 = SPI(1, baudrate=80_000_000, polarity=0, phase=0, bits=8, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
csn1 = Pin(13, Pin.OUT)
csn1.value(1)
csn1.value(0)
spi1.write(bytearray([0x00 | 0x00]))
buf = spi1.read(1)
csn1.value(1)
print("Received 0:", buf[0]))
"""
 
from machine import SPI, Pin
from time import sleep, sleep_us, sleep_ms
from utime import ticks_ms, ticks_diff
import uasyncio

LED_USER = 25
BUTTON_USER = 24
WS2812_USER = 23

class REGISTER:
    @staticmethod
    def register_config(b7_reserved: bool, b6_mask_rx_dr: bool, b5_mask_tx_ds: bool, b4_mask_max_rt: bool, b3_en_crc: bool, b2_crco: bool, b1_pwr_up: bool, b0_prim_rx: bool) -> int:
        if b7_reserved:
            raise ValueError("Only 0 allowed for reserved bits 7")
        register = (b7_reserved << 7) | (b6_mask_rx_dr << 6) | (b5_mask_tx_ds << 5) | (b4_mask_max_rt << 4) | (b3_en_crc << 3) | (b2_crco << 2) | (b1_pwr_up << 1) | (b0_prim_rx << 0)
        return register
    @staticmethod
    def register_en_aa(b7_reseerved: bool, b6_reseerved: bool, b5_enaa_p5: bool, b4_enaa_p4: bool, b3_enaa_p3: bool, b2_enaa_p2: bool, b1_enaa_p1: bool, b0_enaa_p0: bool) -> int:
        if b7_reseerved or b6_reseerved:
            raise ValueError("Only 00 allowed for reserved bits 7:6")
        register = (b7_reseerved << 7) | (b6_reseerved << 6) | (b5_enaa_p5 << 5) | (b4_enaa_p4 << 4) | (b3_enaa_p3 << 3) | (b2_enaa_p2 << 2) | (b1_enaa_p1 << 1) | (b0_enaa_p0 << 0)
        return register
    @staticmethod
    def register_en_rxaddr(b7_reseerved: bool, b6_reseerved: bool, b5_erx_p5: bool, b4_erx_p4: bool, b3_erx_p3: bool, b2_erx_p2: bool, b1_erx_p1: bool, b0_erx_p0: bool) -> int:
        if b7_reseerved or b6_reseerved:
            raise ValueError("Only 00 allowed for reserved bits 7:6")
        register = (b7_reseerved << 7) | (b6_reseerved << 6) | (b5_erx_p5 << 5) | (b4_erx_p4 << 4) | (b3_erx_p3 << 3) | (b2_erx_p2 << 2) | (b1_erx_p1 << 1) | (b0_erx_p0 << 0)
        return register
    @staticmethod
    def register_setup_aw(b7_reserved: bool, b6_reserved: bool, b5_reserved: bool, b4_reserved: bool, b3_reserved: bool, b2_reserved: bool, b1_aw: bool, b0_aw: bool) -> int:
        if b7_reserved or b6_reserved or b5_reserved or b4_reserved or b3_reserved or b2_reserved:
            raise ValueError("Only 000000 allowed for reserved bits 7:2")
        register = (b7_reserved << 7) | (b6_reserved << 6) | (b5_reserved << 5) | (b4_reserved << 4) | (b3_reserved << 3) | (b2_reserved << 2) | (b1_aw << 1) | (b0_aw << 0)
        return register
    @staticmethod
    def register_rf_ch(b7_reserved: bool, b6_rf_ch: bool, b5_rf_ch: bool, b4_rf_ch: bool, b3_rf_ch: bool, b2_rf_ch: bool, b1_rf_ch: bool, b0_rf_ch: bool) -> int:
        if b7_reserved:
            raise ValueError("Only 0 allowed for reserved bits 7")
        register = (b7_reserved << 7) | (b6_rf_ch << 6) | (b5_rf_ch << 5) | (b4_rf_ch << 4) | (b3_rf_ch << 3) | (b2_rf_ch << 2) | (b1_rf_ch << 1) | (b0_rf_ch << 0)
        return register
    @staticmethod
    def register_rf_setup(b7_cont_wave: bool, b6_reserved: bool, b5_rf_dr_low: bool, b4_pll_lock: bool, b3_rf_dr_high: bool, b2_rf_pwr: bool, b1_rf_pwr: bool, b0_obsolete: bool) -> int:
        if b6_reserved:
            raise ValueError("Only 0 allowed for reserved bits 6")
        register = (b7_cont_wave << 7) | (b6_reserved << 6) | (b5_rf_dr_low << 5) | (b4_pll_lock << 4) | (b3_rf_dr_high << 3) | (b2_rf_pwr << 2) | (b1_rf_pwr << 1) | (b0_obsolete << 0)
        return register
    @staticmethod
    def register_status(b7_reserved: bool, b6_rx_dr: bool, b5_tx_ds: bool, b4_max_rt: bool, b3_rx_p_no: bool, b2_rx_p_no: bool, b1_rx_p_no: bool, b0_tx_full: bool) -> int:
        if b7_reserved:
            raise ValueError("Only 0 allowed for reserved bits 7")
        register = (b7_reserved << 7) | (b6_rx_dr << 6) | (b5_tx_ds << 5) | (b4_max_rt << 4) | (b3_rx_p_no << 3) | (b2_rx_p_no << 2) | (b1_rx_p_no << 1) | (b0_tx_full << 0)
        return register
    
class SPEED(object):
    _1MBPS_     = REGISTER.register_rf_setup(0, 0, 0, 0, 0, 1, 1, 0)
    _2MBPS_     = REGISTER.register_rf_setup(0, 0, 0, 0, 1, 1, 1, 0)
    _250KBPS_   = REGISTER.register_rf_setup(0, 0, 1, 0, 0, 1, 1, 0)
    _RESERVED_  = REGISTER.register_rf_setup(0, 0, 1, 0, 1, 1, 1, 0)

class NRF24L01(object):
    """Command Address"""
    _R_REGISTER_            = 0X00
    _W_REGISTER_            = 0X20
    _R_RX_PAYLOAD_          = 0X61
    _W_TX_PAYLOAD_          = 0XA0
    _FLUSH_TX_              = 0XE1
    _FLUSH_RX_              = 0XE2
    _REUSE_RTX_PL           = 0XE3
    _ACTIVATE_              = 0X50
    _R_RX_RL_WID_           = 0X60
    _W_ACK_PAYLOAD_         = 0XA8
    _W_TX_PAYLOAD_NO_ACK_   = 0XB0
    _NOP_                   = 0XFF
    """Register Address"""
    _CONFIG_                = 0x00
    _EN_AA_                 = 0x01
    _EN_RXADDR_             = 0x02
    _SETUP_AW_              = 0x03
    _SETUP_RETR_            = 0x04
    _RF_CH_                 = 0x05
    _RF_SETUP_              = 0x06
    _STATUS_                = 0x07
    _OBSERVE_TX             = 0x08
    _CD_                    = 0x09
    _RX_ADDR_P0_            = 0x0A
    _RX_ADDR_P1_            = 0x0B
    _RX_ADDR_P2_            = 0x0C
    _RX_ADDR_P3_            = 0x0D
    _RX_ADDR_P4_            = 0x0E
    _RX_ADDR_P5_            = 0x0F
    _TX_ADDR_               = 0x10
    _RX_PW_P0_              = 0x11
    _RX_PW_P1_              = 0x12
    _RX_PW_P2_              = 0x13
    _RX_PW_P3_              = 0x14
    _RX_PW_P4_              = 0x15
    _RX_PW_P5_              = 0x16
    _FIFO_STATUS_           = 0x17
    _DYNPD_                 = 0x1C
    _FEATURE_               = 0x1D

    def __init__(self, spi: SPI, ce: int, csn: int, baud: int = 4000000, channel: int = 46, payload_size: int = 16, auto_ack: bool = False):
        assert payload_size <= 32
        self.buffer = bytearray(1)
        self.spi = spi
        self.spi.init(baudrate = baud, polarity = 0, phase = 0)
        self.ce = Pin(ce, Pin.OUT, value = 0)
        self.csn = Pin(csn, Pin.OUT, value = 1)
        self.payload_size = payload_size
        self.__speed: int = None
        self.pipe0_read_address = None
        sleep_ms(5)
        self.w_register(self._CONFIG_, REGISTER.register_config(0,0,0,0,1,0,0,0))
        self.w_register(self._EN_AA_, REGISTER.register_en_aa(0, 0, 1, 1, 1, 1, 1, 1) if auto_ack else REGISTER.register_en_aa(0, 0, 0, 0, 0, 0, 0, 0))
        self.w_register(self._EN_RXADDR_, REGISTER.register_en_rxaddr(0, 0, 0, 0, 0, 0, 0, 1))
        self.w_register(self._SETUP_AW_, REGISTER.register_setup_aw(0, 0, 0, 0, 0, 0, 1, 1)) 
        self.w_register(self._RF_CH_, channel & REGISTER.register_rf_ch(0, 1, 1, 1, 1, 1, 1, 1))
        self.w_register(self._RF_SETUP_, SPEED._2MBPS_)
        self.w_register(self._STATUS_, REGISTER.register_status(0, 1, 1, 1, 0, 0, 0, 0))
        self.flush_rx()
        self.flush_tx()

    def r_register(self, reg: int) -> int:
        self.csn.value(0)
        self.spi.readinto(self.buffer, reg)
        self.spi.readinto(self.buffer)
        self.csn.value(1)
        return self.buffer[0]
    
    def w_register(self, reg: int, buf) -> int:
        self.csn.value(0)
        status = self.spi.write_readinto(bytearray([self._W_REGISTER_ | reg]), self.buffer)
        if isinstance(buf, int):
            self.spi.write(bytearray([buf]))
        else:
            self.spi.write(buf)
        self.csn.value(1)
        return self.buffer[0]

    def status(self) -> int:
        self.csn.value(0)
        self.spi.readinto(self.buffer, self._NOP_)
        self.csn.value(1)
        return self.buffer[0]

    def flush_rx(self):
        self.csn.value(0)
        self.spi.readinto(self.buffer, self._FLUSH_RX_)
        self.csn.value(1)

    def flush_tx(self):
        self.csn.value(0)
        self.spi.readinto(self.buffer, self._FLUSH_TX_)
        self.csn.value(1)

    @property
    def speed(self) -> int:
        return self.__speed

    @speed.setter
    def speed(self, _speed: int):
        self.w_register(self._RF_SETUP_, _speed)
        self.__speed = _speed

    def open_tx_pipe(self, address: int):
        assert len(address) == 5
        self.w_register(self._RX_ADDR_P0_, address)
        self.w_register(self._TX_ADDR_, address)
        self.w_register(self._RX_PW_P0_, self.payload_size)

    def open_rx_pipe(self, id: int, address: int):
        assert len(address) == 5
        assert 1 <= id <= 5
        if id == 0:
            self.pipe0_read_address = address
        if id < 2:
            self.w_register(self._RX_ADDR_P0_ + id, address)
        else:
            self.w_register(self._RX_ADDR_P0_ + id, address[0])
        self.w_register(self._RX_PW_P0_ + id, self.payload_size)
        self.w_register(self._EN_RXADDR_, self.r_register(self._EN_RXADDR_) | (1 << id))

    def rx_mode(self):
        self.w_register(self._CONFIG_, REGISTER.register_config(0, 0, 0, 0, 1, 1, 1, 1))
        self.w_register(self._STATUS_, REGISTER.register_status(0, 1, 1, 1, 0, 0, 0, 0))
        if self.pipe0_read_address is not None:
            self.w_register(self._RX_ADDR_P0_, self.pipe0_read_address)
        self.flush_rx()
        self.flush_tx()
        self.ce.value(1)
        sleep_us(30)

    def rx_stop(self):
        self.ce.value(0)
        self.flush_tx()
        self.flush_rx()

    def any(self):
        return not bool(self.r_register(self._FIFO_STATUS_) & 0x01)
    
    def recv(self):
        self.csn.value(0)
        self.spi.readinto(self.buffer, self._R_RX_PAYLOAD_)
        buf = self.spi.read(self.payload_size)
        self.csn.value(1)
        self.w_register(self._STATUS_, REGISTER.register_status(0, 1, 0, 0, 0, 0, 0, 0))
        return buf
    
    def start(self, buf):
        self.w_register(self._CONFIG_, REGISTER.register_config(0, 0, 0, 0, 1, 1, 1, 0))
        sleep_us(1500)
        self.csn.value(0)
        self.spi.readinto(self.buffer, self._W_TX_PAYLOAD_)
        self.spi.write(buf)
        if len(buf) < self.payload_size:
            self.spi.write(b"\x00" * (self.payload_size - len(buf)))
        self.csn.value(1)
        self.ce.value(1)
        sleep_us(15)
        self.ce.value(0)

    def done(self):
        _status = self.status()
        if not (_status & (0x20 | 0x10)):
            return None
        _status = self.w_register(self._STATUS_, REGISTER.register_status(0, 1, 1, 1, 0, 0, 0, 0))
        cfg = self.r_register(self._CONFIG_)
        self.w_register(self._CONFIG_, cfg & ~0x02)
        return 1 if _status & 0x20 else 2
    
    def tx_send(self, buf: bytes, timeout:int = 500):
        self.start(buf)
        _start = ticks_ms()
        result = None
        while result is None and ticks_diff(ticks_ms(), _start) < timeout:
            result = self.done()
        if result is None:
            self.flush_tx()
            self.w_register(self._CONFIG_, self.r_register(self._CONFIG_) & ~0x02)
            raise OSError("timed out")
        if result == 2:
            raise OSError("tx_send failed")

async def tx_task(nrf: NRF24L01):
    addr = b"1Node"
    nrf.open_tx_pipe(addr)
    print("[TX] Start mode TX.")
    button = Pin(BUTTON_USER, Pin.IN, Pin.PULL_UP)
    state = 0
    last = button.value()
    pressed = False
    while True:
        now = button.value()
        if last == 1 and now == 0:
            pressed = True
        if last == 0 and now == 1 and pressed:
            pressed = False
            state ^= 1
            msg = b"1" if state else b"0"
            try:
                nrf.tx_send(msg)
                print(f"[TX] Sent: {msg}")
            except Exception as e:
                print(f"[TX] Error: {e}")
            await uasyncio.sleep_ms(200) 
        last = now
        await uasyncio.sleep_ms(20)

async def rx_task(nrf: NRF24L01):
    addr = b"1Node"
    nrf.open_rx_pipe(1, addr)
    nrf.rx_mode()
    print("[RX] Start mode RX.")
    led = Pin(LED_USER, Pin.OUT)
    while True:
        if nrf.any():
            data = nrf.recv().rstrip(b"\x00")
            print(f"[RX] Received: {data}")
            if data == b"1":
                led.value(1)
            elif data == b"0":
                led.value(0)
        await uasyncio.sleep_ms(50)

if __name__ == "__main__":
    # nrf1 = NRF24L01(SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4)), 6, 5)
    nrf2 = NRF24L01(SPI(1, sck=Pin(10), mosi=Pin(11), miso=Pin(12)), 9, 13)
    loop = uasyncio.get_event_loop()
    loop.create_task(rx_task(nrf2))
    # loop.create_task(tx_task(nrf2))
    loop.run_forever()