#finished

# Home Module 

This project made for increasing comfort life in rooms/small apartments, namely automatisation. Due to the control of household appliances and lighting with the help of the Internet. Management will take place using HTTPS request (APP)/MQTT (APP) ~~or a bot in popular messengers~~ (has lost its meaning in this version of the project. will be implemented in the next project.)

## Requirements:

- Thernostat control
- Lighting control (AC dimming)
- Sun mode ( imitation sun in the morning )
- Backlight control or different low voltage devices
- Motion sensor 
- Measuring temperature and humidity 
- Configuring and swishing network with HTTPS
- HTTPS server / MQTT client
- ~~Status display~~ has lost its meaning in this version of the project. will be implemented in the next project.

## Technical solution:

- Рeating, lighting, sun mode control   - TRIAC with extrenaly detection cross zero
- Backlight control                     - MOSFET with PWM  
- Motion sensor                         - Extrenaly doppler sensor
- Temperature sensor                    - BME280
- HTTPS server + MCU                    - ESP32

## Implemented hardware:

- Design PCB sixe 10x10cm
- 3 TRIACs with 16A current
- Circuit for detection cross zero
- 3 high current MOSFET 
- Quick-release connectors
- Connector for motion sensor 
- Connector for sensor temperature
- Wiring of the microcontroller
- ~~Connector for LCD~~ has lost its meaning in this version of the project. will be implemented in the next project.

## Implemented software:

- Enable WIFI access point during first time enables
- POST Request for choice network   
- Restart in Station mode and connect to the swlwcted network. In the event of failure to return to the access point
- POST request for change the status of output pins in manual mode 
- POST request for enable and setting Sun mode
- Automatic shutdown Sun mode when using manual mode
- Automatic clock setting using SNTP
- PWM for MOSFETs
- Detection cross zero and dimming ac outputs 

## Project completion and design change

Сompletion of version 2.0 of the project showed an ineffective and unsafe design.
In practice, it turned out to be inconvenient to combine all the above functions on one board. For example, controlling the heater in one part of the room and controlling the light in another part requires long wires. It is also necessary to mount the temperature sensor at a greater distance from the board, which is located directly next to the heater.

The development of this project will be the next project with smaller functional boards, but more convenient. and possible cost reduction due to the absence of long wires.

The future project will include:
low-power temperature, motion and smoke sensor (tested in the project with esp32-c3)
AC control board
board with relay
LED control board
As well as the transition from HTTPS to MQTT and control of the system from popular messengers


## API:

### Available POST requests:

WI-FI settings:
```
https://192.168.4.1/setwifi // def address esp32
```
Body:
```
{
    "SSID":"SSID_WIFI",
    "PASS":"PASS_WIFI",
    "RES" : true
}

```
Device settings:
```
https://192.168.112.60/setstate 
```
Body:
```
{
    "TRIAC":
        {
            "T_1": 1,
            "T_2": 0,
            "T_3": 8
        },
    "MOS":
        {
            "M_1": 25,
            "M_2": 0,
            "M_3": 57

        },
    "Secur"   : true,
    "ADC_VCC" : false,
    "OneWire" : false,
    "BMP280"  : true,
    "LCD"     : true
}
```
SUN Mode setting:
```
https://192.168.112.60/setsunmode
```
Body:
```
{
    "Chanal":1,
    "Hour":9,
    "Min":20,
    "Dur":30
}
```
## Hardware 2.0v:

## Hardware 1.0v:

Schematics:

![Alt Text](https://github.com/mrGrodzki/EspHome/blob/main/HW/esphome_schem_par1.png)

![Alt Text](https://github.com/mrGrodzki/EspHome/blob/main/HW/esphome_schem_par2.png)

PCB:

Unfortunately the PCB was overheating during soldering, thus turning yellow:

![Alt Text](https://github.com/mrGrodzki/EspHome/blob/main/HW/20220512_132227.jpg)

