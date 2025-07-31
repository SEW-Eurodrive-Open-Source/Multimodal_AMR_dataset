# Thermal Camera
## General Information

| Parameter                             | Value                     |
|---------------------------------------|---------------------------|
| Sensor Model                          | Flir Boson 640            |
| Vertical resolution                   | 512                       |
| Horizontal resolution                 | 640                       |
| Focus length                          | 4.9 mm                        |
| Vertical FOV (after distortion)       | 90.9° (77.1°)             |
| Horizontal FOV (after distortion)     | 102° (86.5°)              |
| Update rate                           | 60 Hz                     |

## Calibration
The thermal camera is calibrated with an asymmetric point pattern with 8 x 11 dots with a size of 40 mm. The board consists of polished aluminum with a very low emissivity and the dots of black plastic foil with a high emissivity.
The pattern is heated to ~ 70°C with a heat gun and the image intensity is inverted. The inversion is necessary, for the extrinsic calibration of the rgb and thermal camera, that the black dots are darker than the aluminum plate in both modalities.
The extrinsic calibration between the RGB and thermal camera is done in a distance of 1.5m. For longer and closer distances, the labels of the RGB camera will have an offset to the object in the thermal image.
This misalignment is unavoidable for a sensor setup without a beam splitter. 

camera matrix  
```
420.766869 0.000000 313.349534
0.000000 419.885461 249.008955
0.000000 0.000000 1.000000
```

distortion  
```
-0.328408 0.073691 0.000796 0.001284 0.000000
```

rectification  
```
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000
```

projection  
```
309.299744 0.000000 309.851598 0.000000
0.000000 356.470306 246.227255 0.000000
0.000000 0.000000 1.000000 0.000000
```