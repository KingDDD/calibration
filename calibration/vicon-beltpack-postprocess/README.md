# vicon-beltpack-postprocess
postprocess vicon pose data, vicon sync data, sensor data and prepare into either a calibration dataset or MLX recording

## Usage

```python process.py vicon_calib --vicon data/data--20170508-165413.txt --sync data/output.decode.txt --calib data/calib/calibration/vicon_chessboard_data --output output --rig_object lidar_align_brown```

```python process.py vicon_timestamp_align --vicon data/20170516/verification2/data--20170516-125811.txt --sync data/20170516/verification2/output.decoded.txt --from_time 542115282000  --to_time 556992413000 --output output_verification```
