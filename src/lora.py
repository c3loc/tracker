# based on https://github.com/Wei1234c/SX127x_driver_for_MicroPython_on_ESP8266

import time
import machine
import ubinascii
import esp
import sx127x
from machine import Pin, SPI, reset

def init():
	controller = Controller()
	l = None
	count = 0
	while not l and count < 3:
		try:
			l = controller.add_transceiver(sx127x.SX127x(name = 'LoRa'),pin_id_ss = controller.PIN_ID_FOR_LORA_SS,pin_id_RxDone = controller.PIN_ID_FOR_LORA_DIO0)
		except Exception as e:
			count += 1
			if count > 2:
				raise e
	return l

def mac2eui(mac):
	mac = mac[0:6] + 'fffe' + mac[6:]
	return hex(int(mac[0:2], 16) ^ 2)[2:] + mac[2:]

uuid = ubinascii.hexlify(machine.unique_id()).decode()
NODE_EUI = mac2eui(uuid)
NODE_NAME = uuid
millisecond = time.ticks_ms

class BaseController:

	class Mock:
		pass

	ON_BOARD_LED_PIN_NO = None
	ON_BOARD_LED_HIGH_IS_ON = True
	GPIO_PINS = []
	PIN_ID_FOR_LORA_RESET = None
	PIN_ID_FOR_LORA_SS = None
	PIN_ID_SCK = None
	PIN_ID_MOSI = None
	PIN_ID_MISO = None
	PIN_ID_FOR_LORA_DIO0 = None
	PIN_ID_FOR_LORA_DIO1 = None
	PIN_ID_FOR_LORA_DIO2 = None
	PIN_ID_FOR_LORA_DIO3 = None
	PIN_ID_FOR_LORA_DIO4 = None
	PIN_ID_FOR_LORA_DIO5 = None


	def __init__(self,
				 pin_id_led = ON_BOARD_LED_PIN_NO,
				 on_board_led_high_is_on = ON_BOARD_LED_HIGH_IS_ON,
				 pin_id_reset = PIN_ID_FOR_LORA_RESET,
				 blink_on_start = (2, 0.5, 0.5)):

		self.pin_led = self.prepare_pin(pin_id_led)
		self.on_board_led_high_is_on = on_board_led_high_is_on
		self.pin_reset = self.prepare_pin(pin_id_reset)
		self.reset_pin(self.pin_reset)
		self.spi = self.prepare_spi(self.get_spi())
		self.transceivers = {}

	def add_transceiver(self,
						transceiver,
						pin_id_ss = PIN_ID_FOR_LORA_SS,
						pin_id_RxDone = PIN_ID_FOR_LORA_DIO0,
						pin_id_RxTimeout = PIN_ID_FOR_LORA_DIO1,
						pin_id_ValidHeader = PIN_ID_FOR_LORA_DIO2,
						pin_id_CadDone = PIN_ID_FOR_LORA_DIO3,
						pin_id_CadDetected = PIN_ID_FOR_LORA_DIO4,
						pin_id_PayloadCrcError = PIN_ID_FOR_LORA_DIO5):

		transceiver.transfer = self.spi.transfer
		transceiver.blink_led = self.blink_led

		transceiver.pin_ss = self.prepare_pin(pin_id_ss)
		transceiver.pin_RxDone = self.prepare_irq_pin(pin_id_RxDone)
		transceiver.pin_RxTimeout = self.prepare_irq_pin(pin_id_RxTimeout)
		transceiver.pin_ValidHeader = self.prepare_irq_pin(pin_id_ValidHeader)
		transceiver.pin_CadDone = self.prepare_irq_pin(pin_id_CadDone)
		transceiver.pin_CadDetected = self.prepare_irq_pin(pin_id_CadDetected)
		transceiver.pin_PayloadCrcError = self.prepare_irq_pin(pin_id_PayloadCrcError)

		transceiver.init()
		self.transceivers[transceiver.name] = transceiver
		return transceiver


	def prepare_pin(self, pin_id, in_out = None):
		reason = '''
			# a pin should provide:
			# .pin_id
			# .low()
			# .high()
			# .value()# read input.
			# .irq()	# (ESP8266/ESP32 only) ref to the irq function of real pin object.
		'''
		raise NotImplementedError('reason')


	def prepare_irq_pin(self, pin_id):
		reason = '''
			# a irq_pin should provide:
			# .set_handler_for_irq_on_rising_edge()# to set trigger and handler.
			# .detach_irq()
		'''
		raise NotImplementedError('reason')


	def get_spi(self):
		reason = '''
			# initialize SPI interface
		'''
		raise NotImplementedError('reason')


	def prepare_spi(self, spi):
		reason = '''
			# a spi should provide:
			# .close()
			# .transfer(pin_ss, address, value = 0x00)
		'''
		raise NotImplementedError('reason')


	def led_on(self, on = True):
		self.pin_led.high() if self.on_board_led_high_is_on == on else self.pin_led.low()


	def blink_led(self, times = 1, on_seconds = 0.1, off_seconds = 0.1):
		for i in range(times):
			self.led_on(True)
			time.sleep(on_seconds)
			self.led_on(False)
			time.sleep(off_seconds)


	def reset_pin(self, pin, duration_low = 0.05, duration_high = 0.05):
		pin.low()
		time.sleep(duration_low)
		pin.high()
		time.sleep(duration_high)


	def __exit__(self):
		self.spi.close()

class Controller(BaseController):

	# LoRa config
	PIN_ID_FOR_LORA_RESET = 14

	PIN_ID_FOR_LORA_SS = 18
	PIN_ID_SCK = 5
	PIN_ID_MOSI = 27
	PIN_ID_MISO = 19

	PIN_ID_FOR_LORA_DIO0 = 26
	PIN_ID_FOR_LORA_DIO1 = None
	PIN_ID_FOR_LORA_DIO2 = None
	PIN_ID_FOR_LORA_DIO3 = None
	PIN_ID_FOR_LORA_DIO4 = None
	PIN_ID_FOR_LORA_DIO5 = None


	# ESP config
	ON_BOARD_LED_PIN_NO = 2
	ON_BOARD_LED_HIGH_IS_ON = True
	GPIO_PINS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 34, 35, 36, 37, 38, 39)


	def __init__(self,
			pin_id_led = ON_BOARD_LED_PIN_NO,
			on_board_led_high_is_on = ON_BOARD_LED_HIGH_IS_ON,
			pin_id_reset = PIN_ID_FOR_LORA_RESET,
			blink_on_start = (2, 0.5, 0.5)):

		super().__init__(pin_id_led,
			on_board_led_high_is_on,
			pin_id_reset,
			blink_on_start)


	def prepare_pin(self, pin_id, in_out = Pin.OUT):
		if pin_id is not None:
			pin = Pin(pin_id, in_out)
			new_pin = Controller.Mock()
			new_pin.pin_id = pin_id
			new_pin.value = pin.value

			if in_out == Pin.OUT:
				new_pin.low = lambda : pin.value(0)
				new_pin.high = lambda : pin.value(1)
			else:
				new_pin.irq = pin.irq

			return new_pin


	def prepare_irq_pin(self, pin_id):
		pin = self.prepare_pin(pin_id, Pin.IN)
		if pin:
			pin.set_handler_for_irq_on_rising_edge = lambda handler: pin.irq(handler = handler, trigger = Pin.IRQ_RISING)
			pin.detach_irq = lambda : pin.irq(handler = None, trigger = 0)
			return pin


	def get_spi(self):
		spi = None
		id = 1

		try:
			id = -1
			spi = SPI(id, baudrate = 10000000, polarity = 0, phase = 0, bits = 8, firstbit = SPI.MSB,
					sck = Pin(self.PIN_ID_SCK, Pin.OUT, Pin.PULL_DOWN),
					mosi = Pin(self.PIN_ID_MOSI, Pin.OUT, Pin.PULL_UP),
					miso = Pin(self.PIN_ID_MISO, Pin.IN, Pin.PULL_UP))
			spi.init()

		except Exception as e:
			print(e)
			if spi:
				spi.deinit()
				spi = None
			reset()# in case SPI is already in use, need to reset.

		return spi


	def prepare_spi(self, spi):
		if spi:
			new_spi = Controller.Mock()

			def transfer(pin_ss, address, value = 0x00):
				response = bytearray(1)

				pin_ss.low()

				spi.write(bytes([address]))
				spi.write_readinto(bytes([value]), response)

				pin_ss.high()

				return response

			new_spi.transfer = transfer
			new_spi.close = spi.deinit
			return new_spi


	def __exit__(self):
		self.spi.close()
