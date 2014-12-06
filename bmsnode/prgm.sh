#!/bin/bash

set -e
while [ 1 ]
do
	set -x
	./eepromgen eep.raw
	# hfuse = 0x76 for EESAVE, 7e for no eesave
	sudo avrdude -q -q -p attiny25 -c stk500hvsp -P /dev/ttyUSB1 -U lfuse:w:0xe2:m -U hfuse:w:0x7e:m -U efuse:w:0xff:m
	sudo avrdude -q -q -p attiny25 -c stk500hvsp -P /dev/ttyUSB1 -U flash:w:bmsnode.hex -U eeprom:w:eep.raw:r
	sudo avrdude -q -q -p attiny25 -c stk500hvsp -P /dev/ttyUSB1 -U hfuse:w:0x76:m
	{ set +x; } 2>/dev/null
	echo "Press enter for next"
	read
done
