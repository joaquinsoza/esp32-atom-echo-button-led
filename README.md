Steps to flash and monitor

```
idf.py fullclean
idf.py set-target esp32
idf.py build

# check which srial usb
ls /dev/cu.*

# flashing more slowly works
idf.py -p /dev/cu.usbserial-5552A2D146 -b 115200 flash

idf.py -p /dev/cu.usbserial-5552A2D146 monitor
```

Erase flash on the device
```
python -m esptool --port /dev/cu.usbserial-5552A2D146 --chip esp32 erase_flash
```