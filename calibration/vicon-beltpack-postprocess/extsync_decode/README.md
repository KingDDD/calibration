# Payload decoder tool

```
$ mkdir build
$ cd build
$ cmake ..
$ make
$ cat ../tests/fake_extsync_pin.log | ./extsync_decode 
{ "ktime": 289500, "mtime": 289500, "payload_present": true, "frame_number": 521414, "mean_squared_error": 0 }
{ "ktime": 1293500, "mtime": 1293500, "payload_present": true, "frame_number": 582974, "mean_squared_error": 0.0147059 }
```
