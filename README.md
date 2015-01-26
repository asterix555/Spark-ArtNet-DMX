# Spark Core UDP to DMX
This is a simple application for the Spark Core that receives DMX-Values over WiFi and transmits it over DMX.

The received DMX data is stored and transmitted over DMX with a refresh rate of 25Hz.

The color of the RGB LED is determined by the first 3 channels and the small blue LED toggles every time the DMX universe is sent. The Outputs D0-D7 are set to DMX Values 1-8 (D0 and D1 as PWM, rest as digital).

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

## Sources
- ArtNet reception is based on this gist: https://gist.github.com/deftx/9377546
- DMX transmission is based on this repo: https://code.google.com/p/stm32-dmx512/
