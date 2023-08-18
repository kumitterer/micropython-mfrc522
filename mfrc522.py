from machine import Pin, SPI
from os import uname


class MFRC522:
	""" MicroPython class to communicate with the MFRC522 RFID module """

	# Constants

	OK = 0
	NOTAGERR = 1
	ERR = 2

	REQIDL = 0x26
	REQALL = 0x52
	AUTHENT1A = 0x60
	AUTHENT1B = 0x61

	def __init__(self, sck: int = None, mosi: int = None, miso: int = None, rst: int = None, cs: int = None, spi: SPI = None):
		""" Initialize the MFRC522 module

		For compatibility with the original library, the pins can be specified 
		as arguments for WiPy, LoPy, FiPy and ESP8266. For other platforms, an
		initialized SPI object must be passed as argument.

		```python
		# For ESP8266:

		mfrc522_esp8266 = MFRC522(0, 2, 4, 5, 14)

		# For ESP32:

		from machine import SoftSPI, Pin

		sck = Pin(18, Pin.OUT)
		mosi = Pin(23, Pin.OUT)
		miso = Pin(19, Pin.OUT)
		spi = SoftSPI(baudrate=100000, polarity=0, phase=0, sck=sck, mosi=mosi, miso=miso)
		spi.init()

		sda = Pin(5, Pin.OUT)

		mfrc522_esp32 = MFRC522(spi=spi, cs=sda)
		```

		:param sck: SPI clock pin
		:param mosi: SPI MOSI pin
		:param miso: SPI MISO pin
		:param rst: Reset pin
		:param cs: Chip select pin
		:param spi: Initialized SPI object 
		"""

		if not spi:
			# Initialize SPI bus on given pins

			self.sck = Pin(sck, Pin.OUT)
			self.mosi = Pin(mosi, Pin.OUT)
			self.miso = Pin(miso)
			
			board = uname()[0]

			# Due to differences in the SPI initialization, only the following
			# platforms are supported

			if board == 'WiPy' or board == 'LoPy' or board == 'FiPy':
				self.spi = SPI(0)
				self.spi.init(SPI.MASTER, baudrate=1000000, pins=(self.sck, self.mosi, self.miso))
			elif board == 'esp8266':
				self.spi = SPI(baudrate=100000, polarity=0, phase=0, sck=self.sck, mosi=self.mosi, miso=self.miso)
				self.spi.init()
			else:
				raise RuntimeError("Unsupported platform - please provide an initialized SPI object")

		else:
			self.spi = spi

		if rst:
			# Initialize reset pin

			if isinstance(rst, int):
				self.rst = Pin(rst, Pin.OUT)

			self.rst.value(1)
		else:
			self.rst = None

		# Initialize chip select pin

		if isinstance(cs, int):
			self.cs = Pin(cs, Pin.OUT)
			
		self.cs.value(1)
		
		# Continue in dedicated function

		self.init()

	def _wreg(self, reg: bytes, val: bytes):
		""" Write a value to a register

		:param reg: Register address
		:param val: Value to write
		"""

		self.cs.value(0)
		self.spi.write(b'%c' % int(0xff & ((reg << 1) & 0x7e)))
		self.spi.write(b'%c' % int(0xff & val))
		self.cs.value(1)

	def _rreg(self, reg: bytes):
		""" Read a value from a register

		:param reg: Register address
		:return: Value read
		"""

		self.cs.value(0)
		self.spi.write(b'%c' % int(0xff & (((reg << 1) & 0x7e) | 0x80)))
		val = self.spi.read(1)
		self.cs.value(1)

		return val[0]

	def _sflags(self, reg, mask):
		self._wreg(reg, self._rreg(reg) | mask)

	def _cflags(self, reg, mask):
		self._wreg(reg, self._rreg(reg) & (~mask))

	def _tocard(self, cmd, send):

		recv = []
		bits = irq_en = wait_irq = n = 0
		stat = self.ERR

		if cmd == 0x0E:
			irq_en = 0x12
			wait_irq = 0x10
		elif cmd == 0x0C:
			irq_en = 0x77
			wait_irq = 0x30

		self._wreg(0x02, irq_en | 0x80)
		self._cflags(0x04, 0x80)
		self._sflags(0x0A, 0x80)
		self._wreg(0x01, 0x00)

		for c in send:
			self._wreg(0x09, c)
		self._wreg(0x01, cmd)

		if cmd == 0x0C:
			self._sflags(0x0D, 0x80)

		i = 2000
		while True:
			n = self._rreg(0x04)
			i -= 1
			if ~((i != 0) and ~(n & 0x01) and ~(n & wait_irq)):
				break

		self._cflags(0x0D, 0x80)

		if i:
			if (self._rreg(0x06) & 0x1B) == 0x00:
				stat = self.OK

				if n & irq_en & 0x01:
					stat = self.NOTAGERR
				elif cmd == 0x0C:
					n = self._rreg(0x0A)
					lbits = self._rreg(0x0C) & 0x07
					if lbits != 0:
						bits = (n - 1) * 8 + lbits
					else:
						bits = n * 8

					if n == 0:
						n = 1
					elif n > 16:
						n = 16

					for _ in range(n):
						recv.append(self._rreg(0x09))
			else:
				stat = self.ERR

		return stat, recv, bits

	def _crc(self, data):

		self._cflags(0x05, 0x04)
		self._sflags(0x0A, 0x80)

		for c in data:
			self._wreg(0x09, c)

		self._wreg(0x01, 0x03)

		i = 0xFF
		while True:
			n = self._rreg(0x05)
			i -= 1
			if not ((i != 0) and not (n & 0x04)):
				break

		return [self._rreg(0x22), self._rreg(0x21)]

	def init(self):

		self.reset()
		self._wreg(0x2A, 0x8D)
		self._wreg(0x2B, 0x3E)
		self._wreg(0x2D, 30)
		self._wreg(0x2C, 0)
		self._wreg(0x15, 0x40)
		self._wreg(0x11, 0x3D)
		self.antenna_on()

	def reset(self):
		self._wreg(0x01, 0x0F)

	def antenna_on(self, on=True):

		if on and ~(self._rreg(0x14) & 0x03):
			self._sflags(0x14, 0x03)
		else:
			self._cflags(0x14, 0x03)

	def request(self, mode):

		self._wreg(0x0D, 0x07)
		(stat, recv, bits) = self._tocard(0x0C, [mode])

		if (stat != self.OK) | (bits != 0x10):
			stat = self.ERR

		return stat, bits

	def anticoll(self):

		ser_chk = 0
		ser = [0x93, 0x20]

		self._wreg(0x0D, 0x00)
		(stat, recv, bits) = self._tocard(0x0C, ser)

		if stat == self.OK:
			if len(recv) == 5:
				for i in range(4):
					ser_chk = ser_chk ^ recv[i]
				if ser_chk != recv[4]:
					stat = self.ERR
			else:
				stat = self.ERR

		return stat, recv

	def select_tag(self, ser):

		buf = [0x93, 0x70] + ser[:5]
		buf += self._crc(buf)
		(stat, recv, bits) = self._tocard(0x0C, buf)
		return self.OK if (stat == self.OK) and (bits == 0x18) else self.ERR

	def auth(self, mode, addr, sect, ser):
		return self._tocard(0x0E, [mode, addr] + sect + ser[:4])[0]

	def stop_crypto1(self):
		self._cflags(0x08, 0x08)

	def read(self, addr):

		data = [0x30, addr]
		data += self._crc(data)
		(stat, recv, _) = self._tocard(0x0C, data)
		return recv if stat == self.OK else None

	def write(self, addr, data):

		buf = [0xA0, addr]
		buf += self._crc(buf)
		(stat, recv, bits) = self._tocard(0x0C, buf)

		if not (stat == self.OK) or not (bits == 4) or not ((recv[0] & 0x0F) == 0x0A):
			stat = self.ERR
		else:
			buf = []
			for i in range(16):
				buf.append(data[i])
			buf += self._crc(buf)
			(stat, recv, bits) = self._tocard(0x0C, buf)
			if not (stat == self.OK) or not (bits == 4) or not ((recv[0] & 0x0F) == 0x0A):
				stat = self.ERR

		return stat
