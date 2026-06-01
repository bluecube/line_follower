# HACKS.md

Known workarounds for hardware and driver issues.

## esp-hal `Adc::new` ignores attenuation for ADC2

`Adc::new` hardcodes `ADC1::set_attenuation` even when called with an `ADC2` peripheral, so ADC2
channels are silently left at the register default instead of the requested attenuation.

Workaround in `lf-hal/src/line_sensor.rs`: after both `Adc` instances drop, manually call
`ADC2::set_attenuation` for every ADC2 channel.

TODO: We have since upgraded esp-hal, maybe it's not necessary any more.

## BLE PHY ISR corrupts ADC2 reads

The BLE stack uses ADC2 for TX power detection. After each measurement the PHY ISR leaves
`SAR2_PWDET_FORCE=1` in `SAR_READ_CTRL2`, routing the ADC2 input mux to the internal PWDET
signal instead of the GPIO pin. Reading ADC2 without clearing this bit returns garbage.

Workaround in `lf-hal/src/line_sensor.rs`: `start_adc2()` clears `sar2_pwdet_force` before every
conversion, and every ADC2 read is wrapped in a `critical_section` to prevent the PHY ISR from
re-setting the bit between the clear and the read.

Note: the next hardware revision should route all phototransistor ADC lines to ADC1, eliminating
both workarounds.

## GPIO 12 strapping pin conflict with distance sensor

The IR distance sensor is connected to GPIO 12, which is the MTDI strapping pin on the ESP32. At
boot, the ESP32 samples this pin to determine the VDD_SDIO regulator voltage: HIGH selects 1.8V,
LOW selects 3.3V. With the sensor connected the pin was driven HIGH, causing the chip to boot with
VDD_SDIO at 1.8V and preventing correct operation and flashing.

Fix: permanently burned the VDD_SDIO efuse to force 3.3V, disabling GPIO 12's role as a strapping
pin. This is irreversible. Command used (via PlatformIO's bundled esptool):

```
pip install ecdsa reedsolo bitstring
pio pkg exec --package "platformio/tool-esptoolpy" -- espefuse.py --port /dev/ttyUSB1 set_flash_voltage 3.3V
```

See also: `docs/instructions.md`.

## Button noise

Deck button seems to have problems with EMI when mtors are spinning at full.
So far this only appears in motor_test.rs.
Either we will require multiple samples from the button to agree, or an electrical solution
(caps on the motors, caps on the button). Perhaps both.

TODO: Verify and implement the fixes
