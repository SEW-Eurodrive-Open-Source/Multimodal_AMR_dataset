# Ultrasonic Array


## Multi Sensor Specifications

The USSM array consists of **4 ultrasonic sensors**, each based on the TDK USSM1.0 PLUS-FS. These sensors are triggered in a synchronized sequence to allow precise distance measurements from multiple perspectives using a **pitch-and-catch** mode.

## Communication and Triggering

- A **ROS 1 node** has been implemented to cyclically trigger the USSM array every **25 ms** using **UDP**.
- The trigger is sent to the USSM array module, which consists of:
  - An **STM32 microcontroller**
  - A **Hard-wired TCP/IP stack**, connected via **SPI**, for UDP communication

## Triggering Sequence

- On each 25 ms interval, a new sensor is used as the **transmitter**, while all others act as **receivers**:
  - At **0 ms**: `us0` transmits, `us0`, `us1`, `us2`, `us3` receive
  - At **25 ms**: `us1` transmits, others receive
  - At **50 ms**: `us2` transmits, others receive
  - At **75 ms**: `us3` transmits, others receive

- The triggering is handled precisely on the STM32 via **capture/compare units** for synchronized activation.

## ROS Integration

- A **custom ROS message** (`ussm.msg`) is published **every 25 ms** after each individual sensor transmission.
- A full cycle involving all 4 sensors completes every **100 ms**.


## Custom ROS Message: `ussm.msg`

The `ussm.msg` defines the structure of the message published every 25 ms after each sensor's transmission step.

| Field Type | Field Name | Description |
|------------|-------------|-------------|
| Header | header | ROS standard header including timestamp and frame ID |
| uint8 | us_src | ID of the currently transmitting ultrasonic sensor (0 to 3) |
| uint32 | pkg_num | Sequence number of the UDP packet |
| uint16[] | mm_us0 | Array of target distances (in mm) detected by sensor us0 |
| uint8 | echo_height_0_us0 | Echo amplitude (height) of the closest target detected by sensor us0 |
| uint8 | echo_height_1_us0 | Echo amplitude (height) of the second closest target detected by sensor us0 |
| uint8 | nfd_flag_us0 | Near Field Detection (NFD) flag for sensor us0 indicating close-range target |
| uint8 | noise_detected_us0 | Indicates whether noise/interference was detected on sensor us0 |
| uint8 | none_default_active_us0 | Flag indicating non-default configuration active on sensor us0 |
| uint8 | freq_dev_us0 | Frequency deviation of the internal oscillator of sensor us0 |
| uint16[] | mm_us1 | Array of target distances (in mm) detected by sensor us1 |
| uint8 | echo_height_0_us1 | Echo amplitude (height) of the closest target detected by sensor us1 |
| uint8 | echo_height_1_us1 | Echo amplitude (height) of the second closest target detected by sensor us1 |
| uint8 | nfd_flag_us1 | Near Field Detection (NFD) flag for sensor us1 indicating close-range target |
| uint8 | noise_detected_us1 | Indicates whether noise/interference was detected on sensor us1 |
| uint8 | none_default_active_us1 | Flag indicating non-default configuration active on sensor us1 |
| uint8 | freq_dev_us1 | Frequency deviation of the internal oscillator of sensor us1 |
| uint16[] | mm_us2 | Array of target distances (in mm) detected by sensor us2 |
| uint8 | echo_height_0_us2 | Echo amplitude (height) of the closest target detected by sensor us2 |
| uint8 | echo_height_1_us2 | Echo amplitude (height) of the second closest target detected by sensor us2 |
| uint8 | nfd_flag_us2 | Near Field Detection (NFD) flag for sensor us2 indicating close-range target |
| uint8 | noise_detected_us2 | Indicates whether noise/interference was detected on sensor us2 |
| uint8 | none_default_active_us2 | Flag indicating non-default configuration active on sensor us2 |
| uint8 | freq_dev_us2 | Frequency deviation of the internal oscillator of sensor us2 |
| uint16[] | mm_us3 | Array of target distances (in mm) detected by sensor us3 |
| uint8 | echo_height_0_us3 | Echo amplitude (height) of the closest target detected by sensor us3 |
| uint8 | echo_height_1_us3 | Echo amplitude (height) of the second closest target detected by sensor us3 |
| uint8 | nfd_flag_us3 | Near Field Detection (NFD) flag for sensor us3 indicating close-range target |
| uint8 | noise_detected_us3 | Indicates whether noise/interference was detected on sensor us3 |
| uint8 | none_default_active_us3 | Flag indicating non-default configuration active on sensor us3 |
| uint8 | freq_dev_us3 | Frequency deviation of the internal oscillator of sensor us3 |
| int16 | temperature | - |

The ultrasonic array consists of four equally space ultrasonic sensors, that are mounted at the front of the AMR, tilted upwards. 
The first sensor sends out a chirp and all four sensors listen for the echoes. If the returning signal is higher than the dynamic threshold, the first distance of the first object is saved. 
When the signal falls back under the threshold, the second distance for the first object is saved. These two distances contain information about the object form and size. 
The distances for the further 5 objects are saved, resulting in a total of 12 distances. If no reflection returns for one or more objects, the sensor saves a distance of 65535mm (out of range). For the first two objects, the amplitude of the returning signal is saved.
This measuring cycle is repeated, with the second, third and fourth sensor sending out a chirp. Therefore, after four measuring cycles, every sensor has sent out a chirp.
In the pub_1 message, only one measuring cycle is saved. In the pub_4 message, the same message as in in pub_1 is saved, but also includes the three last pub_1 messages. Therefore, the pub_1 and pub_4 messages are both generated with 40 Hz.
The pub_4 message has a greater information density, but the same object is scattered over a greater area, since the robot and object probably moved between these measuring cycles. 
In [1], the pub_4 message had a great increase in mAP over the pub_1 message. We therefore recommend using the pub_4.

## Message pub_1

The pub_1 message is the original ROS topic written in a `.json`file. 
It contains the ID of the sending sensor `us_src: `. 


## Message pub_4 
The `pub_4` message consists of three `.csv` lists with the near field detection flag (nfd), the amplitude (amp), and the distance (dist) from the transmitter to the receiver. 

The pub_4 `dist.csv` is structured as follows: the first line is the sensor number sens0 to sen3. 
Line 2 to 13 are the received distance r1_0 to r12_0 in mm for each sensor, when sensor 0 is sending.
And respectively line 14 to 25 when sensor 1 is sending, 26 to 37 when sensor 2 is sending, and line 38 to 49 when sensor 3 is sending.

| line | sens0 | sens1 | sens2 | sens3 |
|------|-------|-------|-------|-------|
|   2  |  r1_0 |  r1_0 |  r1_0 |  r1_0 |
|   ❝   |   ❝   |   ❝    |   ❝   |   ❝   |
|  13  | r12_0 | r12_0 | r12_0 | r12_0 |
|  14  |  r1_1 |  r1_1 |  r1_1 |  r1_1 |
|   ❝   |   ❝   |   ❝    |   ❝   |   ❝   |
|  25  | r12_1 | r12_1 | r12_1 | r12_1 |
|  26  |  r1_2 |  r1_2 |  r1_2 |  r1_2 |
|   ❝   |   ❝   |   ❝    |   ❝   |   ❝   |
|  37  | r12_2 | r12_2 | r12_2 | r12_2 |
|  38  |  r1_3 |  r1_3 |  r1_3 |  r1_3 |
|   ❝   |   ❝   |   ❝    |   ❝   |   ❝   |
|  49  | r12_3 | r12_3 | r12_3 | r12_3 |


The pub_4 `nfd.csv` has in the first column the Boolean flag fx for the sensors 0 to 3 in row 2 to 5.

|line  |flag |
|------|-----|
| 2    | f0  |
| 3    | f1  |
| 4    | f2  |
| 5    | f3  |

The pub_4 `amp.csv` has the first and second amplitude `a1_x` and `a2_x` for the four sensors sens0 to sens3 for the four measuring cycles `ax_0` to `ax_3`.

| line | sens0 | sens1 | sens2 | sens3 |
|------|-------|-------|-------|-------|
|   2  | a1_0  | a1_0  | a1_0  | a1_0  |
|   3  | a2_0  | a2_0  | a2_0  | a2_0  |
|   4  | a1_1  | a1_1  | a1_1  | a1_1  |
|   5  | a2_1  | a2_1  | a2_1  | a2_1  |
|   6  | a1_2  | a1_2  | a1_2  | a1_2  |
|   7  | a2_2  | a2_2  | a2_2  | a2_2  |
|   8  | a1_3  | a1_3  | a1_3  | a1_3  |
|   9  | a2_3  | a2_3  | a2_3  | a2_3  |



## Calibration  
To calibrate the ultrasonic sensors, the AMR is positioned in front of a wall, with the wall parallel to the ultrasonic array. The distance error was measured from 500mm to 2250mm in 250mm increments. 
After applying the offsets, the maximum absolute distance error for all sensors is less than ±12mm. The calibration is at 20°C air temperature. 
To compensate for the temperature dependent speed of sound, and therefore range measurement, the air temperature is saved in each pub_1 message in the `.json` as `"temperature":` in °C.  

The following table displays the sensor number and offset for each sensor. 
`offset = measured distance - ground truth distance`

| Sensor Number | Offset (mm) |
|---------------|-------------|
| 0             | 252         |
| 2             | 263         |
| 1             | 256         |
| 3             | 257         |


## Single Sensor Specifications

| Parameter                  | Value                        |
|----------------------------|------------------------------|
| Sensor Model               | TDK USSM1.0 PLUS-FS          |
| Measurement Range (Solo)   | 180 mm – 5000 mm             |
| Measurement Range (P&C)    | 40 mm – 5000 mm              |
| Accuracy                   | Centimeter-level             |
| Measurement Rate           | Up to 50 Hz                  |
| Operating Voltage          | 8 V – 18 V (typ. 12 V)       |
| Current Consumption        | 5.5 mA (active), <1 mA (PD)  |
| Operating Temperature      | –40°C to +85°C               |
| Directivity                | ±35°                         |
| Communication Interface    | PWM-based digital IO         |
| Connector Type             | SWB 1002WVS-03E-LPSW         |
| Water/Dust Resistance      | IP65/IP67 (front side only)  |
| Acoustic Frequency          | 74.5 kHz                    |
| Wavelength (in air @ 20°C)  | 4.6 mm               |

## Reference

- [USSM1.0 PLUS-FS Datasheet (TDK)](https://www.tdk-electronics.tdk.com/inf/55/ds/B59110W2111W032.pdf)
