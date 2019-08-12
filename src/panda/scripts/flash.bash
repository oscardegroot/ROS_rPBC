sudo avrdude -pm2560 -P /dev/ttyUSB0 -b 57600 -c stk500v2 -D -Uflash:w:Elisa3-firmware-advanced-rev240-13.03.18.hex:i -v
