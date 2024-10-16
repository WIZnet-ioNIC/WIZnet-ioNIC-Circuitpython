import board
import rp2pio
import adafruit_pioasm
import digitalio
import time

from wiznet5k_pio import WIZNET5K
from adafruit_ticks import ticks_ms, ticks_diff
from micropython import const

from wiznet5k_pio import (
    SNSR_SOCK_ESTABLISHED,
    SNSR_SOCK_CLOSE_WAIT,
    SNSR_SOCK_LISTEN,
    SNSR_SOCK_CLOSED,
)

# PIO assembly code: SPI master implementation
spi_master = """
.program spi_master
    pull block
    set x, 7
bitloop:
    out pins, 1
    set pins, 1
    nop [1]
    in pins, 1
    set pins, 0
    jmp x-- bitloop
    push block
"""

mac_address = [0x00, 0x08, 0xDC, 0x01, 0x02, 0x03]
ip_address = [192, 168, 11, 110]
gateway_ip = [192, 168, 11, 1]
subnet_mask = [255, 255, 255, 0]

# PIO and State Machine setup
assembled = adafruit_pioasm.assemble(spi_master)
sm = rp2pio.StateMachine(
    assembled,
    frequency=1_000_000,
    first_out_pin=board.GP23,  # mosi
    first_in_pin=board.GP22,  # miso
    first_set_pin=board.GP21,  # clk
    out_pin_count=1,
    in_pin_count=1,
    set_pin_count=1,
    in_shift_right=False,
    out_shift_right=False,
    push_threshold=8,
    pull_threshold=8,
)

# CS and RST pin setup
cs_pin = digitalio.DigitalInOut(board.GP20)
rst_pin = digitalio.DigitalInOut(board.GP25)

# Initialize WIZNET5K
wiznet = WIZNET5K(
    sm,
    cs_pin,
    rst_pin,
    mac_address=mac_address,
    ip_address=ip_address,
    gateway_ip=gateway_ip,
    subnet_mask=subnet_mask,
)
# After creating the WIZNET5K instance
version = wiznet.read_version()
print(f"W5500 Version: 0x{version:02X}")
print("Network information :", wiznet.ifconfig)

while True:
    pass
