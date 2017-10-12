# Self Driving Car Playgroung

## Hardware

I use this [arduino vehicle](https://www.amazon.com/gp/product/B074BSW3QW/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1) as a base for my experiments. It seems to be a pretty solid platform for the money for the instant start. Although in general it is good, there were some issues with the kit:
* power wires were not set correctly so I had to swap them to make it work;
* bluetooth module is connected to pins 0 and 1 by default what makes it impossible to upload the program with bluetooth module connectd. I simply moved bluetooth to pins 11 and 12. Also bluetooth module in the kit is not BLE, so no way to connect it to iPhone... But replacement is coming :)

## Software

Right now there is only some simple functionality and self driving is not implemented yet, however the vehicle now can be controlled over bluetooth and there is a small portion of self-driving like behavior when the vehicle is staying on the sertain distance from a wall.
