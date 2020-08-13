# fan_speed_controller
Using a STM32L476 MCU to control fan speed based on Temperature readings.

Uses on board pushbuttons to increase/decrease setpoint

## Software
Program control based on the following FSM

![image](https://user-images.githubusercontent.com/6884645/90182000-48315880-dd7f-11ea-85e9-86788b4b526a.png)

#### Required
- STM32Cube_FW_L4_V1.8.0 
- STM32 ST-LINK Utility

## Hardware
- LM35 Temperature sensor
- LM358 OpAmp
- 0.1 uF Capacitor
- Optoisolator
- Diode
- Transistor
- Resistors
- PWM Controlled DC Fan
- Power Supply

![image](https://user-images.githubusercontent.com/6884645/90182101-6f882580-dd7f-11ea-80e3-82f5d71ef220.png)

![image](https://user-images.githubusercontent.com/6884645/90182173-8af33080-dd7f-11ea-9add-ea5475fe575b.png)
