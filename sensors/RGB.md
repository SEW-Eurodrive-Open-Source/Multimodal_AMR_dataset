# RGB Camera
## General Information
| Parameter                             | Value                     |
|---------------------------------------|---------------------------|
| Sensor Model                          | Baumer VCXG-51C           |
| Vertical resolution (after binning)   | 2048 (1024)               |
| Horizontal resolution (after binning) | 2448 (1224)               |
| Focus length                          | 4.2 mm                    |
| Vertical FOV (after distortion)       | 82.7° (77.1°)             |
| Horizontal FOV (after distortion)     | 92.8° (86.5°)             |
| Update rate                           | 10-30 Hz                  |

## Calibration

The RGB camera is calibrated using a 7x9 black and white checkerboard pattern, with each square measuring 108 mm.

camera matrix  
```
674.404614 0.000000 594.385232
0.000000 675.289580 529.478017
0.000000 0.000000 1.000000
```


distortion  
```
-0.257535 0.045380 -0.000075 0.000038 0.000000
```

rectification  
```
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000
```

projection  
```
470.149933 0.000000 576.991215 0.000000
0.000000 545.176575 539.383421 0.000000
0.000000 0.000000 1.000000 0.000000
```