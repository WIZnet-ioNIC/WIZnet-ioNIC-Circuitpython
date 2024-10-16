from __future__ import annotations
import rp2pio
import digitalio
import time

from adafruit_ticks import ticks_ms, ticks_diff
from micropython import const

# *** Wiznet Common Registers ***
_REG_MR = const(0x0000)
_REG_GAR = const(0x0001)
_REG_SUBR = const(0x0005)
_REG_SHAR = const(0x0009)
_REG_SIPR = const(0x000F)
_REG_VERSIONR = const(0x0039)
_REG_LINK_FLAG = const(0x002E)
_REG_RTR = const(0x0019)
_REG_RCR = const(0x001B)
_REG_PHYCFGR = const(0x002E)

# *** Wiznet Socket Registers ***
_REG_SNMR = const(0x0000)
_REG_SNCR = const(0x0001)
_REG_SNIR = const(0x0002)
_REG_SNSR = const(0x0003)
_REG_SNPORT = const(0x0004)
_REG_SNTX_FSR = const(0x0020)
_REG_SNTX_WR = const(0x0024)
_REG_SNRX_RSR = const(0x0026)
_REG_SNRX_RD = const(0x0028)
_REG_SNDIPR = const(0x000C)
_REG_SNDPORT = const(0x0010)

# Socket Commands (Sn_CR values)
_CMD_SOCK_OPEN = const(0x01)
_CMD_SOCK_LISTEN = const(0x02)
_CMD_SOCK_CONNECT = const(0x04)
_CMD_SOCK_DISCON = const(0x08)
_CMD_SOCK_CLOSE = const(0x10)
_CMD_SOCK_SEND = const(0x20)
_CMD_SOCK_RECV = const(0x40)

# Socket Modes (Sn_MR values)
_SNMR_CLOSE = const(0x00)
_SNMR_TCP = const(0x01)
_SNMR_UDP = const(0x02)

# Socket Status Register values (Sn_SR)
SNSR_SOCK_CLOSED = const(0x00)
_SNSR_SOCK_INIT = const(0x13)
SNSR_SOCK_LISTEN = const(0x14)
SNSR_SOCK_ESTABLISHED = const(0x17)
SNSR_SOCK_CLOSE_WAIT = const(0x1C)
_SNSR_SOCK_UDP = const(0x22)

# Other constants
_MR_RST = const(0x80)
_DEFAULT_MAC = "DE:AD:BE:EF:FE:ED"
_MAX_SOCK_NUM = const(0x08)
_SOCKET_INVALID = const(0xFF)
_SOCK_SIZE = const(0x800)
_SOCK_MASK = const(0x7FF)
_CH_SIZE = const(0x100)

# Buffer size registers
_REG_Sn_TXBUF_SIZE = const(0x001E)
_REG_Sn_RXBUF_SIZE = const(0x0022)


def debug_msg(message: str, debug: bool):
    if debug:
        print(message)


class WIZNET5K:
    def __init__(
        self,
        spi_sm: rp2pio.StateMachine,
        cs: digitalio.DigitalInOut,
        reset=None,
        is_dhcp: bool = True,
        mac=_DEFAULT_MAC,
        hostname=None,
        debug: bool = False,
        mac_address: Union[bytes, list] = None,
        ip_address: Union[bytes, list] = None,
        gateway_ip: Union[bytes, list] = None,
        subnet_mask: Union[bytes, list] = None,
    ) -> None:
        self._debug = debug
        self._chip_type = 0  # Assuming only W5500 is used
        self._sm = spi_sm

        # CS pin initialization
        self._cs = cs
        self._cs.direction = digitalio.Direction.OUTPUT
        self._cs.value = True

        # Reset WIZnet module
        if reset:
            debug_msg("* Resetting WIZnet chip", self._debug)
            reset.switch_to_output()
            reset.value = False
            time.sleep(0.1)
            reset.value = True
            time.sleep(0.1)
        self._rst = reset

        # SPI communication buffer initialization
        self._pbuff = bytearray(8)
        self._rxbuf = bytearray(2048)  # Maximum packet size (example: 2048 bytes)

        # Set MAC address
        if mac_address is not None:
            if isinstance(mac_address, list):
                self._mac_address = bytes(mac_address)
            elif isinstance(mac_address, bytes):
                self._mac_address = mac_address
            else:
                raise TypeError("mac_address must be a list or bytes")
        else:
            # 기본 MAC 주소 사용
            if isinstance(mac, str):
                self._mac_address = bytes(int(x, 16) for x in mac.split(":"))
            elif isinstance(mac, (bytes, bytearray, list)):
                self._mac_address = bytes(mac)
            else:
                raise TypeError("mac must be a string, list, or bytes-like object.")

        # 네트워크 설정 값 저장
        self._ip_address = bytes(ip_address) if ip_address else None
        self._gateway_ip = bytes(gateway_ip) if gateway_ip else None
        self._subnet_mask = bytes(subnet_mask) if subnet_mask else None

        self._dns = b"\x00\x00\x00\x00"

        # Set hostname (if needed)
        self._hostname = hostname

        # Set DHCP usage
        self._is_dhcp = is_dhcp

        # Additional initialization
        self._ch_base_msb = 0
        self._src_ports_in_use = []
        self.max_sockets = 8

        # UDP initialization
        self.udp_from_ip = [b"\x00\x00\x00\x00"] * self.max_sockets
        self.udp_from_port = [0] * self.max_sockets

        # Initialize WIZnet chip
        self.init()

        # Ethernet link initialization
        start_time = time.monotonic()
        timeout = 5  # 5 seconds timeout
        while time.monotonic() - start_time < timeout:
            if self.link_status():
                break
            debug_msg("Ethernet link is down...", self._debug)
            time.sleep(0.5)
        self._dhcp_client = None

        # DHCP setup
        if is_dhcp:
            self.set_dhcp(hostname)

    def link_status(self):
        # Check link status using PHYCFGR register
        phycfgr = self.read_reg(_REG_PHYCFGR, 0)
        return bool(phycfgr & 0x01)  # Check LNK bit

    def set_dhcp(self, hostname=None):
        # Implement DHCP client or use external library
        debug_msg("Setting up DHCP...", self._debug)
        # Actual DHCP implementation needs to be added
        pass

    def _select(self):
        self._cs.value = False

    def _deselect(self):
        self._cs.value = True

    def _transfer(self, data_out):
        data_in = bytearray(len(data_out))
        self._sm.write_readinto(data_out, data_in)
        return data_in

    def _read(self, addr, block, length=1):
        self._select()
        command = bytearray([(addr >> 8) & 0xFF, addr & 0xFF, block & 0xFF])
        data_out = command + bytearray([0x00] * length)
        data_in = self._transfer(data_out)
        self._deselect()
        return data_in[3:]

    def _write(self, addr: int, callback: int, data) -> None:
        self._select()
        # Construct command
        command = bytearray([(addr >> 8) & 0xFF, addr & 0xFF, (callback | 0x04) & 0xFF])

        # Data type handling
        if isinstance(data, int):
            try:
                data = data.to_bytes(1, "big")
            except OverflowError:
                data = data.to_bytes(2, "big")
        elif isinstance(data, bytes) or isinstance(data, bytearray):
            pass  # Already bytes-like
        else:
            raise TypeError("Data must be an integer or bytes-like object")

        # Transfer data
        data_out = command + data
        self._transfer(data_out)
        self._deselect()

    def _write_socket_register(self, sock: int, address: int, data: int) -> None:
        """Write to W5500 socket register."""
        cntl_byte = (sock << 5) + 0x0C  # Set write bit
        self._write(address, cntl_byte, data)

    def _read_socket_register(self, sock: int, address: int) -> int:
        """Read from W5500 socket register."""
        cntl_byte = (sock << 5) + 0x08  # Read operation
        register = self._read(address, cntl_byte)
        return int.from_bytes(register, "big")

    def read_reg(self, addr, block):
        data = self._read(addr, block, 1)
        return data[0]

    def write_reg(self, addr, block, data):
        self._write(addr, block, data)
        time.sleep(0.01)

    def reset(self):
        if self._rst is not None:
            self._rst.value = False
            time.sleep(0.1)
            self._rst.value = True
            time.sleep(0.1)

    def init(self):
        self.reset()
        # Soft reset
        self.write_reg(_REG_MR, 0, _MR_RST)
        time.sleep(0.1)
        # Initialize Common Register
        self.write_reg(_REG_MR, 0, 0x00)
        # Set MAC Address
        for i, val in enumerate(self._mac_address):
            self.write_reg(_REG_SHAR + i, 0, val)
        # Set IP Address
        if self._ip_address:
            for i, val in enumerate(self._ip_address):
                self.write_reg(_REG_SIPR + i, 0, val)
        # Set Gateway IP
        if self._gateway_ip:
            for i, val in enumerate(self._gateway_ip):
                self.write_reg(_REG_GAR + i, 0, val)
        # Set Subnet Mask
        if self._subnet_mask:
            for i, val in enumerate(self._subnet_mask):
                self.write_reg(_REG_SUBR + i, 0, val)
        print("W5500 Initialization complete")

    def socket_set_buffer_size(self, txsize, rxsize):
        tx_total = sum(txsize)
        rx_total = sum(rxsize)

        if any(size not in [0, 1, 2, 4, 8, 16] for size in txsize + rxsize):
            raise ValueError("Buffer size must be one of [0, 1, 2, 4, 8, 16] KB")

        if tx_total > 16 or rx_total > 16:
            raise ValueError("Total TX or RX buffer size cannot exceed 16KB")

        for i in range(len(txsize)):
            self.write_reg(_REG_Sn_TXBUF_SIZE + i, 0, txsize[i])
            self.write_reg(_REG_Sn_RXBUF_SIZE + i, 0, rxsize[i])
            # print(f"Socket {i} buffer sizes set: TX={txsize[i]}KB, RX={rxsize[i]}KB")

    def socket_init(self, sock_num):
        print("Initializing socket")
        # Close and initialize the socket
        self._write_socket_register(
            sock_num, _REG_SNCR, _CMD_SOCK_CLOSE
        )  # CLOSE command
        time.sleep(0.01)

        status = self._read_socket_register(sock_num, _REG_SNSR)
        print(f"Socket {sock_num} status after close: 0x{status:02X}")

        # Set to TCP mode
        self._write_socket_register(sock_num, _REG_SNMR, _SNMR_TCP)  # Set to TCP mode
        time.sleep(0.01)

        # Open the socket
        self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_OPEN)  # OPEN command
        time.sleep(0.1)

        # Check socket status
        status = self._read_socket_register(sock_num, _REG_SNSR)
        print(f"Socket {sock_num} status after open: 0x{status:02X}")
        if status != _SNSR_SOCK_INIT:
            raise RuntimeError(
                f"Socket {sock_num} failed to initialize, status: 0x{status:02X}"
            )

    def socket_listen(self, sock_num, port):
        print(f"Setting socket {sock_num} to listen on port {port}")
        # Set port (Sn_PORT is 0x0004, two bytes)
        self._write_socket_register(sock_num, _REG_SNPORT, (port >> 8) & 0xFF)
        self._write_socket_register(sock_num, _REG_SNPORT + 1, port & 0xFF)
        time.sleep(0.01)

        self._write_socket_register(
            sock_num, _REG_SNCR, _CMD_SOCK_LISTEN
        )  # LISTEN command
        time.sleep(0.1)

        # Wait for command to complete
        while self._read_socket_register(sock_num, _REG_SNCR):
            time.sleep(0.01)

        status = self._read_socket_register(sock_num, _REG_SNSR)
        print(f"Socket {sock_num} status after listen: 0x{status:02X}")

    def socket_status(self, sock_num):
        return self._read_socket_register(sock_num, _REG_SNSR)

    def _read_data(self, sock_num, addr, length):
        # Read data from RX buffer
        cntl_byte = (sock_num << 5) + 0x18  # 0x18 for RX buffer read
        data = self._read(addr, cntl_byte, length)
        return data

    def _write_data(self, sock_num, addr, data):
        # Write data to TX buffer
        cntl_byte = (sock_num << 5) + 0x14  # 0x14 for TX buffer write
        self._write(addr, cntl_byte, data)

    def socket_recv(self, sock_num):
        rx_size = self._read_socket_register(
            sock_num, _REG_SNRX_RSR
        ) << 8 | self._read_socket_register(sock_num, _REG_SNRX_RSR + 1)
        if rx_size > 0:
            # Read RX read pointer
            rx_rd = self._read_socket_register(
                sock_num, _REG_SNRX_RD
            ) << 8 | self._read_socket_register(sock_num, _REG_SNRX_RD + 1)

            # Calculate the physical address
            addr = rx_rd & _SOCK_MASK

            # Read data from RX buffer
            data = self._read_data(sock_num, addr, rx_size)

            # Update RX read pointer
            rx_rd = (rx_rd + rx_size) & 0xFFFF
            self._write_socket_register(sock_num, _REG_SNRX_RD, (rx_rd >> 8) & 0xFF)
            self._write_socket_register(sock_num, _REG_SNRX_RD + 1, rx_rd & 0xFF)

            # Issue RECV command
            self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_RECV)
            return data
        return None

    def socket_send(self, sock_num, data):
        data_length = len(data)

        # Read TX write pointer
        tx_wr = self._read_socket_register(
            sock_num, _REG_SNTX_WR
        ) << 8 | self._read_socket_register(sock_num, _REG_SNTX_WR + 1)

        # Calculate the physical address
        addr = tx_wr & _SOCK_MASK

        # Write data to TX buffer
        self._write_data(sock_num, addr, data)

        # Update TX write pointer
        tx_wr = (tx_wr + data_length) & 0xFFFF
        self._write_socket_register(sock_num, _REG_SNTX_WR, (tx_wr >> 8) & 0xFF)
        self._write_socket_register(sock_num, _REG_SNTX_WR + 1, tx_wr & 0xFF)

        # Issue SEND command
        self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_SEND)

        # Wait for SEND command to complete
        while self._read_socket_register(sock_num, _REG_SNCR):
            time.sleep(0.001)

    def socket_open_udp(self, sock_num, port):
        print(f"Opening UDP socket {sock_num} on port {port}")
        # Close the socket first
        self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_CLOSE)
        time.sleep(0.01)

        # Set to UDP mode
        self._write_socket_register(sock_num, _REG_SNMR, _SNMR_UDP)
        time.sleep(0.01)

        # Set the local port for the UDP socket
        self._write_socket_register(sock_num, _REG_SNPORT, (port >> 8) & 0xFF)
        self._write_socket_register(sock_num, _REG_SNPORT + 1, port & 0xFF)
        time.sleep(0.01)

        # Open the socket
        self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_OPEN)
        time.sleep(0.1)

        # Check socket status
        status = self._read_socket_register(sock_num, _REG_SNSR)
        if status != _SNSR_SOCK_UDP:
            raise RuntimeError(
                f"Failed to open UDP socket {sock_num}, status: 0x{status:02X}"
            )
        print(f"UDP socket {sock_num} opened on port {port}")

    def udp_sendto(self, sock_num, dest_ip, dest_port, data):
        # Set destination IP
        for i in range(4):
            self._write_socket_register(sock_num, _REG_SNDIPR + i, dest_ip[i])

        # Set destination port
        self._write_socket_register(sock_num, _REG_SNDPORT, (dest_port >> 8) & 0xFF)
        self._write_socket_register(sock_num, _REG_SNDPORT + 1, dest_port & 0xFF)

        # Send data
        self.socket_send(sock_num, data)

    def udp_recvfrom(self, sock_num):
        rx_size = self._read_socket_register(
            sock_num, _REG_SNRX_RSR
        ) << 8 | self._read_socket_register(sock_num, _REG_SNRX_RSR + 1)
        if rx_size > 0:
            # Read RX read pointer
            rx_rd = self._read_socket_register(
                sock_num, _REG_SNRX_RD
            ) << 8 | self._read_socket_register(sock_num, _REG_SNRX_RD + 1)

            # Calculate the physical address
            addr = rx_rd & _SOCK_MASK

            # Read data from RX buffer
            data = self._read_data(sock_num, addr, rx_size)

            # Update RX read pointer
            rx_rd = (rx_rd + rx_size) & 0xFFFF
            self._write_socket_register(sock_num, _REG_SNRX_RD, (rx_rd >> 8) & 0xFF)
            self._write_socket_register(sock_num, _REG_SNRX_RD + 1, rx_rd & 0xFF)

            # Issue RECV command
            self._write_socket_register(sock_num, _REG_SNCR, _CMD_SOCK_RECV)

            # Remove header (8 bytes: source IP, source port, and length)
            return data[8:]
        return None

    def socket_close(self, sock_num):
        self._write_socket_register(
            sock_num, _REG_SNCR, _CMD_SOCK_CLOSE
        )  # CLOSE command
        while self._read_socket_register(sock_num, _REG_SNCR):
            time.sleep(0.01)  # Wait for command to complete

    def print_socket_status(self, sock_num):
        status = self.socket_status(sock_num)
        print(f"Socket {sock_num} status: 0x{status:02X}")
        print(
            f"Socket {sock_num} mode: 0x{self._read_socket_register(sock_num, _REG_SNMR):02X}"
        )
        print(
            f"Socket {sock_num} command: 0x{self._read_socket_register(sock_num, _REG_SNCR):02X}"
        )
        print(
            f"Socket {sock_num} interrupt: 0x{self._read_socket_register(sock_num, _REG_SNIR):02X}"
        )
        port_high = self._read_socket_register(sock_num, _REG_SNPORT)
        port_low = self._read_socket_register(sock_num, _REG_SNPORT + 1)
        print(f"Socket {sock_num} port: {port_high << 8 | port_low}")

    def print_network_info(self):
        print("Network Information:")
        print(
            "IP Address:",
            ".".join(str(self.read_reg(_REG_SIPR + i, 0)) for i in range(4)),
        )
        print(
            "Gateway IP:",
            ".".join(str(self.read_reg(_REG_GAR + i, 0)) for i in range(4)),
        )
        print(
            "Subnet Mask:",
            ".".join(str(self.read_reg(_REG_SUBR + i, 0)) for i in range(4)),
        )
        print(
            "MAC Address:",
            ":".join(f"{self.read_reg(_REG_SHAR + i, 0):02X}" for i in range(6)),
        )

    def read_version(self):
        version_reg_addr = _REG_VERSIONR
        version = self._read(version_reg_addr, 0x00)[0]
        return version

    @property
    def rcr(self) -> int:
        """Retry count register."""
        addr = _REG_RCR
        return int.from_bytes(self._read(addr, 0x00), "big")

    @rcr.setter
    def rcr(self, retry_count: int) -> None:
        """Retry count register."""
        addr = _REG_RCR
        if not (0 <= retry_count <= 255):
            raise ValueError("Retries must be from 0 to 255.")
        self._write(addr, 0x04, retry_count)

    @property
    def rtr(self) -> int:
        """Retry time register."""
        addr = _REG_RTR
        return int.from_bytes(self._read(addr, 0x00, 2), "big")

    @rtr.setter
    def rtr(self, retry_time: int) -> None:
        """Retry time register."""
        addr = _REG_RTR
        if not (0 <= retry_time < 2**16):
            raise ValueError("Retry time must be from 0 to 65535")
        self._write(addr, 0x00, retry_time)

    @property
    def ip_address(self) -> bytes:
        """
        Configured IP address for the WIZnet Ethernet hardware.

        :return bytes: IP address as four bytes.
        """
        return self._read(_REG_SIPR, 0x00, 4)

    @property
    def ipv4_address(self) -> str:
        """
        Configured IP address for the WIZnet Ethernet hardware.

        :return str: The IP address (a string of the form '255.255.255.255')
        """
        return self.pretty_ip(self.ip_address)

    @staticmethod
    def pretty_ip(ipv4: bytes) -> str:
        """
        Convert a 4 byte IP address to a dotted-quad string for printing.

        :param bytearray ipv4: A four byte IP address.

        :return str: The IP address (a string of the form '255.255.255.255').

        :raises ValueError: If IP address is not 4 bytes.
        """
        if len(ipv4) != 4:
            raise ValueError("Wrong length for IPv4 address.")
        return ".".join(f"{byte}" for byte in ipv4)

    @staticmethod
    def unpretty_ip(ipv4: str) -> bytes:
        """
        Convert a dotted-quad string to a four byte IP address.

        :param str ipv4: IPv4 address (a string of the form '255.255.255.255') to be converted.

        :return bytes: IPv4 address in four bytes.

        :raises ValueError: If IPv4 address is not 4 bytes.
        """
        return WIZNET5K._unprettyfy(ipv4, ".", 4)

    @property
    def mac_address(self) -> bytes:
        """
        The WIZnet Ethernet hardware MAC address.

        :return bytes: Six byte MAC address.
        """
        return self._read(_REG_SHAR, 0x00, 6)

    @mac_address.setter
    def mac_address(self, address: Union[bytes, str]) -> None:
        """
        Set the WIZnet hardware MAC address.

        :param Union[bytes, str] address: A hardware MAC address.

        :raises ValueError: If the MAC address is invalid
        """
        if isinstance(address, str):
            try:
                address = bytes(int(x, 16) for x in address.split(":"))
            except ValueError:
                raise ValueError("Invalid MAC address format.")
        elif isinstance(address, (bytes, bytearray)):
            address = bytes(address)
        else:
            raise TypeError("MAC address must be a string or bytes-like object.")

        if len(address) != 6:
            raise ValueError("MAC address must be 6 bytes long.")

        self._write(_REG_SHAR, 0x04, address)

    @staticmethod
    def pretty_mac(mac: bytes) -> str:
        """
        Convert a bytes MAC address to a ':' separated string for display.

        :param bytes mac: The MAC address.

        :return str: Mac Address in the form 00:00:00:00:00:00

        :raises ValueError: If MAC address is not 6 bytes.
        """
        if len(mac) != 6:
            raise ValueError("Incorrect length for MAC address.")
        return ":".join(f"{byte:02X}" for byte in mac)

    def remote_port(self, socket_num: int) -> int:
        """
        Port number of the host which sent the current incoming packet.

        :param int socket_num: ID number of the socket to check.

        :return int: The incoming port number of the socket connection.

        :raises ValueError: If the socket number is out of range.
        """
        self._sock_num_in_range(socket_num)
        return self._read_two_byte_sock_reg(socket_num, _REG_SNDPORT)

    @staticmethod
    def _unprettyfy(ipv4: str, delimiter: str, expected_length: int) -> bytes:
        """
        Helper method to convert a delimited string to bytes.

        :param str ipv4: The IPv4 address string.
        :param str delimiter: The delimiter used in the string.
        :param int expected_length: Expected number of bytes.

        :return bytes: Converted bytes.

        :raises ValueError: If the format is incorrect.
        """
        parts = ipv4.split(delimiter)
        if len(parts) != expected_length:
            raise ValueError(
                f"IP address must have {expected_length} parts separated by '{delimiter}'."
            )
        try:
            return bytes(int(part) for part in parts)
        except ValueError:
            raise ValueError("IP address parts must be integers between 0 and 255.")

    def _sock_num_in_range(self, sock_num: int):
        """
        Check if the socket number is within the valid range.

        :param int sock_num: Socket number to check.

        :raises ValueError: If socket number is out of range.
        """
        if not (0 <= sock_num < self.max_sockets):
            raise ValueError(
                f"Socket number {sock_num} is out of range (0-{self.max_sockets - 1})."
            )

    def _read_two_byte_sock_reg(self, sock_num: int, reg_addr: int) -> int:
        """
        Read two bytes from a socket register and return as an integer.

        :param int sock_num: Socket number.
        :param int reg_addr: Register address.

        :return int: Combined integer value of two bytes.
        """
        high = self._read_socket_register(sock_num, reg_addr)
        low = self._read_socket_register(sock_num, reg_addr + 1)
        return (high << 8) | low

    @property
    def ifconfig(self) -> Tuple[str, str, str, str]:
        """
        Network configuration information.

        :return Tuple[str, str, str, str]: The IP address, subnet mask, gateway
            address and DNS server address as strings.
        """
        ip = self.ipv4_address
        subnet = self.pretty_ip(self._read(_REG_SUBR, 0x00, 4))
        gateway = self.pretty_ip(self._read(_REG_GAR, 0x00, 4))
        dns = self._dns
        return (ip, subnet, gateway, dns)

    @ifconfig.setter
    def ifconfig(self, params: Tuple[bytes, bytes, bytes, bytes]) -> None:
        """
        Set network configuration.

        :param Tuple[bytes, bytes, bytes, bytes]: Configuration
            settings - (ip_address, subnet_mask, gateway_address, dns_server).
        """
        if len(params) != 4:
            raise ValueError("ifconfig requires a tuple of four byte sequences.")

        ip_address, subnet_mask, gateway_address, dns_server = params

        for param in [ip_address, subnet_mask, gateway_address, dns_server]:
            if len(param) != 4:
                raise ValueError("Each IPv4 address must be 4 bytes.")

        # Set IP Address
        for i in range(4):
            self._write(_REG_SIPR + i, 0, ip_address[i])
        # Set Subnet Mask
        for i in range(4):
            self._write(_REG_SUBR + i, 0, subnet_mask[i])
        # Set Gateway IP
        for i in range(4):
            self._write(_REG_GAR + i, 0, gateway_address[i])
        # Set DNS Server
        self._dns = bytes(dns_server)
