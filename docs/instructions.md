# Getting the robot to work

This is mostly a random collection of things.
Not complete by any means.

## Install platformio
I couldn't get platformio to work normally, so instead I used a virtual environment:

```
python -m venv venv
. venv/bin/activate
pip install platformio
```

Then I can do
```
platformio run
```

The top level Makefile already assumes that the venv is used.

# Fixing GPIO12 conflicting with the distance sensor

By default, the board cannot be flashed correctly when the distance sensor is connected, because GPIO12 is a strapping pin (MTDI).
I followed instructions in [ESP32 docs](https://docs.espressif.com/projects/esptool/en/latest/esp32/espefuse/set-flash-voltage-cmd.html) and permantently set the VDD_SDIO regulator to 3.3V.
This disables the use of GPIO12 as a strapping pin, but it's an irreversible change.

Platformio doesn't let me directly use the `espfuse.py`, has to be accessed through a package.
In addition, there are some dependencies missing, so I installed them in the virtual env.
I also had to specify port for connecting to the ESP32.

Final commands:
```
pip install ecdsa reedsolo bitstring
pio pkg exec --package "platformio/tool-esptoolpy" -- espefuse.py --port /dev/ttyUSB1 set_flash_voltage 3.3V
```
