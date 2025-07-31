## 2D Laser Scanner
One laser scanner is mounted in the left front and the other in the right back. Therefore, they have a combined field of view of 360° without any blind spots.  
⚠ **Note:** Since the AMR has no preferred driving direction, the *right back* (rb) scanner scans the same area as the other sensors, while the *left front* (lf) scanner scans the rear of all our scenes.

| Parameter           | Value                |
|---------------------|----------------------|
| Sensor Model        | Sick MicroScan3      |
| Wave length         | 845 nm               |
| Angular resolution  | 0.1°                 |
| Scanning angle      | 275°                 |
| Perception range    | 64 m                 |
| Performance level   | PL d (EN ISO 13849)  |
| Update Rate         | 25 Hz                |

The rotational offset of the origin of the scan to the direction of driving of the AMR is 3.19 radian for the right back scanner and 0 radian for the front left scanner. 

The Ros message of the laser scanner is saved as a `.json` with the following structure:

```
{
    "header": {
        "seq": 57600,
        "stamp": {
            "secs": 1710774889,
            "nsecs": 621889319
        },
        "frame_id": "scan_lb_laser"
    },
    "angle_min": -2.399827718734741,
    "angle_max": 2.3998003005981445,
    "angle_increment": 0.00672220578417182,
    "time_increment": 4.3000000005122274e-05,
    "scan_time": 0.03999999910593033,
    "range_min": 0.10000000149011612,
    "range_max": 40.0,
    "ranges": [...],
    "intensities": [...]
}
```  
