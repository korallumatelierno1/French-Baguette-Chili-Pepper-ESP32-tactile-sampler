# French-Baguette-Chili-Pepper-ESP32-tactile-sampler
Example code for the French baguette sampler based on the Chili Pepper ESP32 devkit by Korallum Atelier No.1

How to add samples to your French Baguette : 

Insert a formatted SD card (ideal is MS DOS - FAT32)

Power on then wait 1 minute, the software will create empty files on the SD. 

Insert your SD in your computer then you just have to paste some WAVs to manually map your sampler, works with various formats. 

If too long, the WAVs are cut. 

How to upload code on Chili Pepper : 

Board name on Arduino : ESP32S3 Dev Module

Upload profile (midi enabled) :

->tools 

PSRAM : OPI PSRAM 

USB mode : USB-OTG (tiny usb)

Play these sequence with buttons if not recognized : Hold boot, clic reset, release reset, release boot

*** To use midi function, please connect a data compatible usb C câble to the USB direct port ***

Control : 

Oct - : Eiffel Tower; 
Oct + : Vine bottle; 
Volume - : Cloud; 
Volume + : Flower

Sampler mode : Combo Eiffel + Vine bottle 
Return to synth mode : Same combo 
Use the line 64 in the config.h file to change the synthesizer scale. 
