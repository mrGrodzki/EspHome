# ESPHome Module 


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
