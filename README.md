# Tunnel-Vision
Rapid mapping and state estimation through a fixed-wing UAS in a GPS-denied and potentially featureless environment.


Acronyms:
- FV: Flight Vehicle
- CAD: Computer Aided Design
- UAS: Unmanned aerial system
- GPS: Global Positioning System 
- ToF: Time of flight

TODO:

Fit gears, axle and mounts, push forward?, reinforce surrounding structure?, rotation structure interface with wing

1. Determine mapping sensors, model them, and place in their desired configurations
2. Create the structure around FV + mapping components
     - Shrink holes for plastic tapping
     - Replace front facing SDM15 with higher range lidar?
3. Create PCB for microcontrollers and JST connectors
     - Start with circuit diagram
     - Prototype with perf board
     - Design in KiCAD/Altium and order it
4. Make outside streamlined structure
5. Design the aero structures
     - And deployment methods later on
6. Test components and create calibration or initial setup procedures?
7. Test Telemetry radio with all other components
        - Radio works with the ToF data
8. Test all sensing devices, recording all data, and see overall sampling rate
9. Check out redundant IMUs if needed?
10. Print, cut, assemble parts for first ground tests and then first flight
11. Add parachute and parachute release for quickly stopping
12. Simulate flight dynamics and program them into avionics
13. Create LiDAR based state estimation and hallway centering algorithm 

Notes:
-- To build in PlatformIO: pio run -e total-system
-- To Upload: pio run -e total-system -t upload

1. Finished CADing potential FV control parts and layed most of them out in a possible configuration
2. Finished initial calcuations for baseline wing area and lift aerodynamics based on 4412 airfoil
3. Finished testing Holybro Telemetry modules where we are able to receive data from the teensy on my windows laptop
4. Finished intercepting the PPM commands from the RC receiver with the Teensy. Can use the RC controller to change values, have the teensy intercept them, and then transmit them back to us with the Holybro radio.
5. Finished testing multiple SDM15 ToF Sensors at one time through multiple hardware serials
     - Pin numbers for Teensy Hardware Serials: https://www.pjrc.com/teensy/td_uart.html
     - SDM15 Datasheet: https://github.com/May-DFRobot/DFRobot/blob/master/SEN0588-Data_Sheet_V0.1.2_(230228).pdf
     - SDM15 Driver/Library: https://github.com/being24/YDLIDAR-SDM15_arduino/tree/main
6. Optimizing SD Card Data logging for speed
     - https://hackingmajenkoblog.wordpress.com/2016/03/25/fast-efficient-data-storage-on-an-arduino/
     - https://arduino.stackexchange.com/questions/28540/how-to-increase-sd-card-write-speed-in-arduino#comment111165_34857
     - Write in binary, use FAT32, use Sdfat library, use FIFO SDIO
     - 
