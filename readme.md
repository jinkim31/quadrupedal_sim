### Launch sim environment
```shell
ros2 launch quadrupedal_sim launch.py 
```

### Publish dummy joint commands
```shell
ros2 run quadrupedal_sim controller
```

### Generate dataset
- ns: number of samples
- fs: sampling frequency in Hz
- dir: directory to save dataset file

```shell
ros2 run quadrupedal_sim dataset_gatherer --ros-args -p ns:=300 -p fs:=30.0 -p dir:=~/Desktop
```