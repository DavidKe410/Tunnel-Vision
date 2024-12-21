# Tunnel-Vision
Rapid mapping and state estimation through a fixed-wing UAS in a GPS-denied and featureless environment.


Acronyms:
- FV: Flight Vehicle
- CAD: Computer Aided Design
- UAS: Unmanned aerial system
- GPS: Global Positioning System
- ToF: Time of flight

TODO:
1. Create the structure around FV components
2. Test components and create calibration or initial setup procedures?
3. Test Telemetry radio with all other components
        - Radio works with the ToF data
5. Test all sensing devices, recording all data, and see sampling rate

Notes:
1. Finished CADing potential FV control parts and layed most of them out in a possible configuration
2. Finished initial calcuations for baseline wing area and lift aerodynamics based on 4412 airfoil
3. Finished testing Holybro Telemetry modules where we are able to receive data from the teensy on my windows laptop
4. Finished intercepting the PPM commands from the RC receiver with the Teensy. Can use the RC controller to change values, have the teensy intercept them, and then transmit them back to us with the Holybro radio.
5. Finished testing multiple SDM15 ToF Sensors at one time through multiple hardware serials
     - Pin numbers for Teensy Hardware Serials: https://www.pjrc.com/teensy/td_uart.html
     - SDM15 Datasheet: https://github.com/May-DFRobot/DFRobot/blob/master/SEN0588-Data_Sheet_V0.1.2_(230228).pdf
     - SDM15 Driver/Library: https://github.com/being24/YDLIDAR-SDM15_arduino/tree/main
