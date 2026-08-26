# ESP32-S3(or variant) CAN bus Ethanol Content Analyzer

This will read the frequency(Ethanol %) and low pulse time(C Temp) of a standard GM Ethanol content sensor, print it to serial, and then transmit a CAN message every 250ms if we are receiving a certain CAN bus ID(i.e. RPM). It currently only checks if the ID is received, not any values. On startup it will transmit regardless for 15 seconds. The code averages 8 samples before transmitting and includes a basic spike filter for occasional noise. Basic addressable LED status, green for good, red for bad, purple for waiting for CAN RX.

CAN bus transmit message format can be set to whatever. Default is set to mirror the Zeitronix format. It will transmit 50% Ethanol, max temperature, and a faulted sensor state for any error condition. The error conditions being a disconnected or failed sensor, too low of a frequency(below 0%), and too high of a frequency(above 100%).

![Wiring Diagram.png]()

I was getting random noise spikes where the ethanol percentage would randomly spike 20-30% above what it should be. Adding a 10nF capacitor across the GPIO reduced it to maybe once or twice in a 30 minute period. The basic spike filter code eliminated those being an issue. I also added a diode for basic protection of the GPIO.

Tested with:
* VDO / CONTINENTAL SE1004S or GM 13577429. Connector for the sensor is a Delphi 13511132. Cheap connectors on AliExpress work great.
* ESP32-S3 Super Mini Board - https://www.espboards.dev/esp32/esp32-s3-super-mini
* Texas Instruments SN65HVD230 CANBUS transceiver. Again, cheap breakout boards on AliExpress. Depending on your application you may not may not need a 120 Ohm termination resistor. The boards I bought from AliExpress included them.
* QS-1205CME-3A buck converter, from AliExpress, to step down the vehicle 12V to 5V needed for the ESP32.
* 10nF Ceramic Capacitor
* 3.9K Ohm Resistor (1/8 Watt is fine)
* 1N5817 Schottky Diode
* 3D Printed Box - https://www.printables.com/model/72839-customizable-parametric-stable-and-waterproof-elec

![Completed Module1.png]()
![Completed Module2.png]()

```
Output Ethanol Content: % (Frequency Hz - 50)
Fuel temperature °C: [41.25 x low pulsewidth (ms)] - 81.25
Self diagnostics: Embedded (fuel contamination more likely as readings increase above 150Hz)
```
