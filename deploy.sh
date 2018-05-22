#!/bin/sh

port=/dev/ttyUSB0
base=`dirname $0`

transfere() {
	echo -n "transfering $1 ..."
	ampy --port $port put $base/src/$1 $1
	echo done
}

transfere lora.py
transfere base64.py
transfere sx127x.py
transfere urequests.py
transfere hmac.py
transfere main.py
