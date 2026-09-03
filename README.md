# 3D Spatial Mapping System
A stepper-driven Time-of-Flight scanner that builds a real-time 3D point cloud of a room, streamed live to MATLAB.


<img width="861" height="553" alt="assembled_device" src="https://github.com/user-attachments/assets/f0da4524-03ef-411f-8f87-bb4298e1ff8f" />


## About Me
Computer Engineering student at McMaster University, focused on embedded systems and hardware/firmware integration. Built for COMPENG 2DX3 (Microprocessor Systems).

## What It Does
Full 3D scanning normally needs expensive LIDAR or vision hardware. This project does it with a $70 parts list: a Time-of-Flight sensor spins through 360° on a stepper motor, taking 32 distance readings per sweep. Each reading streams over UART in real time, and a MATLAB script turns the readings into a live 3D point cloud as the device is moved through a space. Tested against a real hallway — the scan matches the room.

| Real hallway | Reconstructed 3D scan |
|---|---|
| <img width="412" height="433" alt="campus_location_photo_2" src="https://github.com/user-attachments/assets/4939f0e5-71d2-42c1-a57c-60b877eaa539" /> | <img width="582" height="426" alt="campus_location_scan_2" src="https://github.com/user-attachments/assets/e06744c3-7d47-4794-bd57-5ed27a1dad44" /> |<img width="545" height="399" alt="matlab_pointcloud" src="https://github.com/user-attachments/assets/67f0d9c0-4c8e-415e-a795-75745f889a80" />



## Key Features
- 360° scanning, 32 points at 11.25° resolution, per position
- Real-time I2C distance sensing (VL53L1X), accurate to ±20–25 mm
- 115200 baud UART streaming straight into MATLAB
- Two-button control: arm/disarm, trigger scan
- Live status LEDs for measurement, transmit, and motor state
- Live-updating 3D point cloud (MATLAB `scatter3`)
- Auto-unwind after each scan — no wire tangling

## Technologies / Skills

**Hardware**
ARM Cortex-M4F (TI TM4C1294) · I2C · UART · GPIO register-level programming · Stepper motor control · Oscilloscope verification

**Software**
C (bare-metal) · Keil µVision · MATLAB · Git

## My Contributions
- Wrote the full firmware — button handling, scan sequencing, arm/disarm state machine (`Final_Spatial_Mapping_2dx3.c`)
- Built the I2C porting layer connecting ST's VL53L1X driver to the TM4C1294's I2C peripheral (`vl53l1_platform_2dx4.c/.h`)
- Designed the UART protocol streaming scan data to MATLAB
- Wrote the MATLAB-side coordinate conversion (polar → Cartesian) and live 3D plotting script
- Diagnosed the system's speed bottleneck through direct measurement, and verified timing against an oscilloscope
- Validated the full system against a real physical space

Not my own work: `PLL.c/.h`, `SysTick.c/.h`, `uart.c/.h`, and `onboardLEDs.c/.h` were provided as COMPENG 2DX3 course starter code. `VL53L1X_api.c/.h`, `vl53l1_types.h`, and `vl53l1_platform.h` are ST's official (unmodified) VL53L1X driver. `CMSIS/` and `Device/` are TI/ARM vendor files required to build the project.

## Design / Architecture
<img width="1428" height="629" alt="circuit_schematic" src="https://github.com/user-attachments/assets/b60f050e-fc7c-4781-92ff-ba911014a040" />


```
[Buttons] → Arm / Trigger scan
     |
     v
Stepper Motor --11.25° step--> ToF Sensor (I2C)
     |                              |
     v                              v
Status LEDs               Distance Reading
     |                              |
     +---------> UART ------------->+
                   |
                   v
      MATLAB: polar → Cartesian → 3D plot
```

## Results
- 32 readings per 360° scan, 11.25° resolution
- ~52–55 s per scan position (motor stepping is the bottleneck — measured, not guessed)
- UART baud error: 0.08%, well inside the reliability threshold
- Bus speed independently confirmed at ~32 MHz on an oscilloscope
- Real hallway reconstructed as a matching 3D point cloud

<img width="545" height="399" alt="matlab_pointcloud" src="https://github.com/user-attachments/assets/a7f25585-a8ba-44f5-8dbe-15b3e8f927c2" />


## Challenges + How I Solved Them
**Wire twisting** after repeated rotations → added an automatic counter-rotation reset after every scan.

**No hardware sin/cos support**, and on-device single-precision math would compound error across many readings → moved all trig to MATLAB's double-precision math instead, keeping error well under the sensor's own accuracy.

## What I Learned
- Porting a vendor sensor driver to new hardware
- Real-time sensor/actuator coordination on a microcontroller
- Serial protocol design for external tools
- Isolating a real bottleneck through measurement instead of assumption
- Oscilloscope-based hardware verification

## Documentation / Links
- 📄 [Full technical report](https://github.com/user-attachments/files/31766131/khalio9_2DX3_Final_Report_Project.pdf)
- 🖥️ Firmware: this repository
- 📸 More photos: `images/`
<p align="center">
  <img width="607" height="455" alt="campus_location_photo" src="https://github.com/user-attachments/assets/01eb6a9d-c13a-40a5-8b15-eba441e678df" />
  <br>
  <em>A second campus hallway used for validation</em>
</p>

<p align="center">
  <img width="516" height="419" alt="campus_location_scan" src="https://github.com/user-attachments/assets/bfb957b9-f86d-4e3d-865c-58651ac13646" />
  <br>
  <em>Reconstructed 3D scan of the same hallway</em>
</p>

<p align="center">
  <img width="490" height="416" alt="matlab_single_scan" src="https://github.com/user-attachments/assets/0a86da2b-755c-45c6-a6a5-5b3172effd22" />
  <br>
  <em>A single 360° scan slice before combining into the full point cloud</em>
</p>
