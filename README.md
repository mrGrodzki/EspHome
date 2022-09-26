# Home Module 

This project made for increasing comfort life in rooms/small apartments, namely automatisation

## Requirements:

- Thernostat control
- Lighting control (AC dimming)
- Sun mode ( imitation sun in the morning )
- Backlight control or different low voltage devices
- Motion sensor 
- Measuring temperature and humidity 
- Configuring and swishing network with HTTPS
- HTTPS server / MQTT client
- ~~Status display~~

## Technical solution:

- heating, lighting, sun mode control   - TRIAC with extrenaly detection cross zero
- Backlight control                     - MOSFET with PWM  
- Motion sensor                         - Extrenaly doppler sensor
- Temperature sensor                    - BME280
- HTTPS server + MCU                    - ESP32

This is a sample IoT project, using esp32 with a dedicated SDK, for control in an intelligent apartment.
The device can measure the temperature / humidity / pressure from various sensors, recognize human activity within its range, notice vibrations, and output information to the display.
It includes 3 reliably controlled 230V output and 3 low voltage outputs.

The device also supports HTTPS server, switching between WI-FI AP and WI-FI STA.

## SW:

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
            "T_3": 14
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
## HW:

Schematics:

![Alt Text](https://github.com/mrGrodzki/EspHome/blob/main/HW/esphome_schem_par1.png)

![Alt Text](https://github.com/mrGrodzki/EspHome/blob/main/HW/esphome_schem_par2.png)

PCB:

![Alt Text](https://github.com/mrGrodzki/EspHome/blob/main/HW/PCBEspHome.png)

Unfortunately the PCB was overheating during soldering, thus turning yellow:

![Alt Text](https://github.com/mrGrodzki/EspHome/blob/main/HW/20220512_132227.jpg)

