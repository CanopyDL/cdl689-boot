# cdl689-boot
OpenBLT bootloader for the CDL689 Industrial IMU tailored for RS-485 communications on a STM32F411CE.

## Uploading New Firmware
Use Feaser's microboot utility to upload new firmware to the CDL-689.  It can be obtained from the Feaser website below:
https://www.feaser.com/openblt/doku.php?id=manual:microboot
1. Connect the CDL-689 to a power supply and a desktop PC using a USB->RS-485 converter.
2. Using a small jumper wire, connect the AUX pin of the CDL-689 to ground.  Restart the device by cycling power, and it will enter bootloader mode as indicated by the quickly blinking LED (once every quarter second).
3. Start the microboot utility.
4. Click the settings button, and choose the appropriate COM port and baud rate of 115,200.
5. Click the Browse button and select an SREC file containing the firmware to be uploaded.  The progress bar should advance as the new firmware is transferred to the device.
6. Cycle power on the CDL-689 again, and the new firmware should run.
