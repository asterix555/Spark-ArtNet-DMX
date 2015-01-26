# Spark Core UDP to DMX
This is a simple application for the Spark Core that receives DMX-Values over WiFi and transmits it over DMX.

The received DMX data is stored and transmitted over DMX with a refresh rate of 25Hz.

The color of the RGB LED is determined by the first 3 channels and the small blue LED toggles every time the DMX universe is sent. The Outputs D0-D7 are set to DMX Values 1-8 (D0 and D1 as PWM, rest as digital).

This code integrates great to home-automation-servers such as openHAB, Loxone or many others as you can use simple commands for one specific channel. If you use ArtNet you have to send all 512 channels at once, using this code you can set or fade any channel with a simple packet.

## Usage

Send the command(s) to Spark Core IP, Port 3200 (TCP or UDP):
__< Mode >< DMX-Channel >[,< DMX-Value >,[< Fading-Time >]][;< Next Command >]__

Where mode is __S__ for set or __F__ for fade, DMX-Channel as a single Channel from 1-512 and Value from 0-255.

On unix for example:
<pre>
# echo "S1,255;F2,255;F3,128" | nc -u sparkcore.lan 3200
</pre>

This will set Channel 1 instantly to 255 (=100%) and softly fade Channel 2 to 255 (100%) and Channel 3 to 128 (50%).


## Compiling
__This code will not work in Sparks online IDE!__
You have to compile it yourself or upload the firmware without changes.

Check out Sparks repository for information on compiling: https://github.com/spark/core-firmware

## Transmitting DMX
To transmit DMX you need an RS485-transceiver chip. Hooking up a MAX485 is simple:

<p align="center" >
  <img src="MAX485-Schematic.png" alt="Schematic" width="768" height="412">
</p>

## Flashing

Use the spark-cli tools to upload the firmware (typically stored in core-firmware.bin) to your spark core. Press and hold "MODE"-Button (also called USER-Button) while pressing "RESET". Release "MODE" button as soon as the core flashes yellow (takes approx 3 seconds).

<pre># spark flash --usb core-firmware.bin</pre>

The Spark Core should boot automatically the new firmware.

## Spark Cloud Features

To improve the stability of the firmware I disabled the Spark Cloud. To revert to any other firmware using Spark Cloud IDE you need to set SYSTEM_MODE to AUTOMATIC or flash any other Cloud-enabled firmware like Tinker:

<pre># spark flash --usb tinker</pre>

## Sources
- Spark Core port of DMX transmission is based on this project: https://github.com/KyotoFox/Spark-ArtNet-DMX
- DMX transmission is based on this repo: https://code.google.com/p/stm32-dmx512/
