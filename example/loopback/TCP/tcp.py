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

## TCP
# Initialize socket and start listening (MAX socket < 8)
sock_num_tcp = 0
port_tcp = 5000

print("Socket Open (TCP)")
wiznet.socket_init(sock_num_tcp)
wiznet.socket_listen(sock_num_tcp, port_tcp)

print(f"Listening on TCP port {port_tcp}")

while True:
    # Handle TCP socket
    status_tcp = wiznet.socket_status(sock_num_tcp)
    if status_tcp == SNSR_SOCK_ESTABLISHED:  # SOCK_ESTABLISHED
        # Receive data from the client
        data = wiznet.socket_recv(sock_num_tcp)
        if data:
            remote_port = wiznet.remote_port(sock_num_tcp)
            print(f"[TCP] Dest P: {remote_port} Received: {data}")
            # Echo the data back to the client
            wiznet.socket_send(sock_num_tcp, data)
    elif status_tcp == SNSR_SOCK_CLOSE_WAIT:  # SOCK_CLOSE_WAIT
        print("[TCP] Client disconnected, closing socket")
        wiznet.socket_close(sock_num_tcp)
        wiznet.socket_init(sock_num_tcp)
        wiznet.socket_listen(sock_num_tcp, port_tcp)
    elif status_tcp == SNSR_SOCK_LISTEN:  # SOCK_LISTEN
        pass  # Listening for incoming connections
    elif status_tcp == SNSR_SOCK_CLOSED:  # SOCK_CLOSED
        wiznet.socket_init(sock_num_tcp)
        wiznet.socket_listen(sock_num_tcp, port_tcp)
