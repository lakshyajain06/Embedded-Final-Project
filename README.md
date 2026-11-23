# SynapSense

Classroom prototype for detecting sustained, rhythmic wrist motion associated with the clonic phase of generalized or focal-to-bilateral tonic–clonic seizures. The Arduino firmware uses the accelerometer, gyroscope, RGB LED and Bluetooth on the Seeed Studio XIAO nRF52840 Sense. This prototype is for demonstration only. It has no clinical validation.

## Run the demo

1. Connect the XIAO nRF52840 **Sense** via USB and upload with PlatformIO or run `pio run -t upload`. If the upload port is missing, double-press the reset button for bootloader mode and retry.
2. In **nRF Connect for Mobile**, scan for **SynapSense**, connect, and open the **Nordic UART Service**. Enable notifications on the TX characteristic ending in `0003` (full UUID `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`). Keep the app open for the demo.
3. Hold the board still, move it normally, and try a single quick movement. Then, while holding it securely, move it back and forth briskly with some rotation about four times per second for several seconds. The onboard red LED lights for ten seconds and the TX characteristic receives text such as `ALERT t=12450ms`. The time is milliseconds since startup. The detector rearms after six consecutive quiet window evaluations. A blinking blue LED indicates an IMU startup fault or sustained invalid readings.
4. Open the PlatformIO serial monitor at 115200 and send lowercase `s` to check IMU and BLE startup. Send lowercase `d` to review the most recent event. The output gives the event number, trigger time and 150 CSV rows from oldest to newest with nominal offsets ending at zero. Multiply raw acceleration by 0.000244 for g, and raw angular velocity by 0.035 for degrees per second.

The detector evaluates overlapping three-second windows every half second at a 50 Hz sampling rate. It subtracts each axis mean and checks acceleration RMS, gyro RMS and acceleration autocorrelation. Two consecutive qualifying windows trigger an event. The board keeps the latest event window and one pending BLE alert in RAM, including while the phone is disconnected. A newer event replaces that snapshot and any pending alert. Restarting clears both. The phone receives a GATT notification in the BLE app.

## Enclosure

Print [the base](enclosure/base.stl) and [lid](enclosure/lid.stl) as oriented. Use slicer supports under the internal board shelves and check the spring-bar holes after printing. The lugs accept a standard 22 mm watch strap with removable spring bars. Install a headerless board with the USB port facing the opening and secure it to the shelves with thin nonconductive tape. The lid has an LED viewing slot. Use removable tape on the lid edge if needed.

The enclosure assumes a battery body up to 26 × 22 × 4.5 mm with room for leads, and a headerless XIAO. Check the actual battery, connector, and USB plug clearance before printing. Secure the battery to the floor of the case with nonconductive mounting tape and keep the leads clear of the lid. Verify the battery and charger against the [Seeed board documentation](https://wiki.seeedstudio.com/XIAO_BLE/). The STL files can be regenerated with `python enclosure/generate.py` in a Python environment with `manifold3d` and `numpy` installed.
