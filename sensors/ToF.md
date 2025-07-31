# ToF Camera
## General Information
Beside the 3D point cloud in .bin and .pcd, the camera outputs depth images and active illuminated monochrome images in 16 bit in the `.tiff` format.  

| Parameter           | Value                |
|---------------------|----------------------|
| Sensor Model        | Espros TOFcam-660-WF |
| Wave length         | 940 nm               |
| Vertical resolution | 240                  |
| Horizontal resolution| 320                 |
| Vertical FOV        | 77°                  |
| Horizontal FOV      | 108°                 |
| Perception range    | 5-18 m               |
| Update rate         | 8-10 Hz              |

## Calibration
The ToF camera is calibrated using a 7x5 black and white checkerboard pattern with each square measuring 110 mm. 

camera matrix  
```
184.06065513369512 0.0 160.16030711699992
0.0 184.07423018567775 120.2924664780963
0.0 0.0 1.0
```

distortion  
```
-0.34004968473041836 0.07787829624140852 -0.00027895403873718727 0.00044137332989315403 0.0
```

rectification  
```
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000
```

projection  
```
184.06065513369512 0.0 160.16030711699992 0.0
0.0 184.07423018567775 120.2924664780963 0.0
0.0 0.0 1.0 0.0
```