GNSS DATA LOGGER FOR UAV TO SOLVE DELAY PROBLEM CAUSED BY OVERLOAD OF AUTOPILOT


Intruduction:

Usually when people use old and weak autopilots based on AVR processor, they face problems with overloaded processor.
This is caused by controlling UAV and doing another actions as taking photos at the same time. Autopilots have not enough 
of computational power and there main problems starts. Sometimes it skips planned photo taking and logging data.


Solution:

I made a Data Logger for UAV to solve this problem. Firstly, i connected additional Arduino microcontroller to Pixhawk autopilot,
to get GPS data and Pitch, Roll, Yaw angles. I decided to take this data for making an orthoimages later ( you can use another).
So all the data was taken from autopilot using MAVLink protocol via UART. Later for saving data, i connected sd card reader
to my Arduino. Its was important for me to have all data stored somewhere for later research and use. Also to save problem with 
triggering camera i made a camera shutter via USB. Sending 5V impulse to camera to remote control. I used CanonS110, with CHDK 
scripts (Its allows you controll camera via USB remotelly). When i understant when should i take a photo? I decided to take a photo 
every 3 sec. But my system starts only when im getting PWM pulse more than 1500, and its stops when i get PWM less than 1500

Components:

Autopilot Pixhawk 2Cube
Arduino Due
Canon S110
FRSKY reciever
SD Card Module 
microflash card

