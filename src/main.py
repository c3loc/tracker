import machine
import time
import struct
import network
import sys
import lora
import hmac
import ubinascii
import random

from config import *

def parseLonLat(data):
	try:
		sdata = data.split('.')
		sec = float('0.'+sdata[-1])*60
		min = int(sdata[0][-2:])
		deg = int(sdata[0][:-2])
	except:
		return None
	return deg + (min/60.0) + (sec/3600.0)

def parseNMEA(line):
	info = {}
	if len(line) < 6:
		return info
	if line[0] != '$':
		return info
	line = line.lstrip('$').rstrip('\r\n').split(',')
	info['paramcount'] = len(line) - 1
	if len(line) > 0:
		tmp = line[0]
		if len(tmp) < 5:
			return info
		info['provider'] = '{}{}'.format(tmp[0], tmp[1])
		info['type'] = '{}{}{}'.format(tmp[2], tmp[3], tmp[4])

	if info.get('type') == 'GLL' and  len(line) == 8 and line[6] == 'A':
		info['lat'] = line[1]
		info['lato'] = line[2]
		info['lon'] = line[3]
		info['lono'] = line[4]
		info['fixtime'] = line[5]
		info['checksum'] = line[7]
	elif info.get('type') == 'RMC' and len(line) == 13:
		info['fixtime'] = line[1]
		info['lat'] = line[3]
		info['lato'] = line[4]
		info['lon'] = line[5]
		info['lono'] = line[6]
		info['speed'] = line[7]
		info['time'] = line[9]
		info['checksum'] = line[12]
	elif info.get('type') == 'GGA' and  len(line) == 16:
		info['fixtime'] = line[1]
		info['lat'] = line[2]
		info['lato'] = line[3]
		info['lon'] = line[4]
		info['lono'] = line[5]
		info['quality'] = line[6]
		info['satcount'] = line[7]
		info['checksum'] = line[15]
	elif info.get('type') == 'GSA' and  len(line) == 18:
		info['sats'] = [x for x in line[3:14] if x != '']
		info['PDOP'] = line[15]
		info['HDOP'] = line[16]
		info['VDOP'] = line[17]
	elif info.get('type') == 'ZDA' and  len(line) == 8:
		info['time'] = line[1]
		info['checksum'] = line[7]
	
	for i in ['lon', 'lat']:
		if i in info:
			info[i] = parseLonLat(info[i])

	return info

def getBattery():
	pin = machine.Pin(BAT_PIN)
	adc = machine.ADC(pin)
	adc.atten(machine.ADC.ATTN_11DB)
	adc.width(adc.WIDTH_10BIT)
	result = []
	for i in range(8):
		result.append(adc.read())
		time.sleep_ms(100//8)
	tmp = 0
	for i in result:
		tmp += i
	return tmp // len(result)

def sendDataLora(buf):
	print('started sending lora data...', end='')
	timebefore = time.ticks_ms()
	l = lora.init()
	l.aquire_lock(False)
	l.beginPacket()
	l.write(buf)
	l.endPacket();
	l.aquire_lock(False)
	print('done (took {}ms)'.format(time.ticks_ms() - timebefore))

def encodeData(data):
	mac = struct.unpack("<L", network.WLAN().config('mac')[-4:])[0]
	bat = getBattery()
	run = struct.pack("<ffBL", data['lat'], data['lon'], bat, mac)
	runhmac = hmac.new(HMAC_KEY, msg=run).digest()[:HMAC_LENGTH]
	print('data: {}, length (+ {} bytes hmac): {}, hmac: {}'.format(run, HMAC_LENGTH, len(run), ubinascii.hexlify(runhmac).decode()))
	print('{}, {} (bat: {}, mac: {})'.format(data['lat'], data['lon'], bat, mac))
	return run + runhmac

def connectWifi():
	import network
	wifi = network.WLAN(network.STA_IF)
	if not wifi.isconnected():
		print('connecting to network...')
		wifi.active(True)
		wifi.connect(SSID, SSID_PW)
		while not wifi.isconnected():
			pass
		print('network config:', wifi.ifconfig())

def sendDataWifi(data, rssi=0, snr=0):
	import urequests
	import base64
	encodeddata = base64.b64encode(data)
	r = urequests.get(URL, json={ 'apikey': APIKEY, 'gateway': GWNAME, 'data': encodeddata, 'rssi': rssi, 'snr': snr})
	try:
		print(r.text)
	finally:
		r.close()

def receiveLora(chip, rawdata):
	try:
		data = rawdata[:-HMAC_LENGTH]
		datahmac = rawdata[-HMAC_LENGTH:]
		wantedhmac = hmac.new(HMAC_KEY, msg=data).digest()[:HMAC_LENGTH]
		rssi = chip.packetRssi()
		snr = chip.packetSnr()
		print('got data {}, hmac: {}, rssi: {}, snr: {}'.format(data, datahmac, rssi, snr))
		if datahmac == wantedhmac:
			sendDataWifi(bytes(data), rssi, snr)
		else:
			print('broken hmac on packet, was {}, should be {}'.format(ubinascii.hexlify(datahmac).decode(), ubinascii.hexlify(wantedhmac).decode()))
	except Exception as e:
		sys.print_exception(e)
		machine.reset()

def mainTracker():
	uart = machine.UART(2, 9600) # 17: tx, 16: rx
	start = time.ticks_ms()
	while (time.ticks_ms() - start) < GPS_WAIT_TIME :
		try:
			line = uart.readline().decode()
		except Exception:
			line = None
		if not line or line == '':
			time.sleep_ms(100)
			continue

		info = parseNMEA(line)
		print(info)
		if info.get('lat') and info.get('lon'):
			buf = encodeData(info)
			sendDataLora(buf)
			break
	sleeptime = random.randint(int(SEND_INTERVALL*0.9), int(SEND_INTERVALL*1.1))
	machine.deepsleep(max(1, sleeptime))

def mainGateway():
	connectWifi()
	l = lora.init()
	l.onReceive(receiveLora)
	l.receive()
	while True:
		time.sleep(1)

try:
	if TYPE == 'tracker':
		mainTracker()
	elif TYPE == 'gateway':
		mainGateway()
except Exception as e:
	sys.print_exception(e)
	machine.reset()
