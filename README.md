This is a custom PCB and software design I created to compete in the APEC Micromouse 
competition in 2024. The design is based on the ESP32-S3 which is a dual core 
microprocessor. The pcb design was done in Kicad 7.0 and the MCU was programmed using 
the esp-idf utilizing RTOS for time dsensitive task scheduling.

Here is a link to an earlier hardware design running in a maze setting with 
control software:
[![mouse demo](https://img.youtube.com/vi/heloENtCQyA/0.jpg)](https://www.youtube.com/watch?v=heloENtCQyA)

The PCB schematic consists of IR emitters and receivers, power modules, the MCU, 
gyro, encoders, and buttons.
<img src="https://github.com/kyletyni/mercury/blob/main/images/pcb_schematic.png" width="800">

This is what the PCB looked like when it was delivered.
<img src="https://github.com/kyletyni/mercury/blob/main/images/PCB1.jpg" width="800">

Assembly complete! Software is still a work in progress, but all modules on the board
function individually as expected.

<div style="display:flex; justify-content:space-between;">
  <img src="https://github.com/kyletyni/mercury/blob/main/images/mouse1.jpg" width="400">
  <img src="https://github.com/kyletyni/mercury/blob/main/images/mouse2.jpg" width="400">
</div>

Layout of front and back layers of the board.
<div style="display:flex; justify-content:space-between;">
  <img src="https://github.com/kyletyni/mercury/blob/main/images/pcb_front.png" width="300">
  <img src="https://github.com/kyletyni/mercury/blob/main/images/pcb_back.png" width="300">
</div>