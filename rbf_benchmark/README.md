## Benchmarks using Google Benchmark

Results were obtained on Ubuntu 18.04 with ROS Melodic using an i7-6700K and a 128GB Samsung 830 SSD.  

## Requirements

Google Benchmark  
For Ubuntu: `sudo apt install libbenchmark-dev`

#### Optional
[ros_type_introspection](https://github.com/facontidavide/ros_type_introspection) If you want to benchmark RosBabelFish against RTI

## Results

### Message Look Up
The times for the creation of the description provider and the lookup were obtained separetely.
To run this benchmark you have to compile ros_babel_fish with `-DENABLE_EMBEDDED_PYTHON=ON`.

```
$ sudo cpupower frequency-set --governor performance && rosrun rbf_benchmark message_lookup && sudo cpupower frequency-set --governor powersave
Run on (8 X 4200 MHz CPU s)
2019-09-18 14:58:59
***WARNING*** Library was built as DEBUG. Timings may be affected.
-------------------------------------------------------------------------------
Benchmark                                        Time           CPU Iterations
-------------------------------------------------------------------------------
EmbeddedPythonDescriptionProviderCreate         53 ms         52 ms         14
EmbeddedPythonDescriptionProviderLookUp       3753 us       3747 us        190
IntegratedDescriptionProviderCreate            117 ms        117 ms          6
IntegratedDescriptionProviderLookUp            927 us        927 us        756
```

### Message Translation
The time it takes to translate a raw received message.
Separated into the initial step of parsing the message definition which has to be done only once and subsequent message parsing.

```
$ sudo cpupower frequency-set --governor performance && rosrun rbf_benchmark message_translation && sudo cpupower frequency-set --governor powersave
Run on (8 X 4200 MHz CPU s)
2019-09-18 14:37:05
***WARNING*** Library was built as DEBUG. Timings may be affected.
----------------------------------------------------------------------
Benchmark                               Time           CPU Iterations
----------------------------------------------------------------------
RTI_ParseMessageDefinitionPose         85 us         85 us       8237
RBF_ParseMessageDefinitionPose       1232 us       1231 us        569
RTI_ParseMessagePose                  469 ns        469 ns    1527047
RBF_ParseMessagePose                  710 ns        710 ns     997311
RTI_ParseMessagePointcloud          18244 ns      18244 ns      38100
RBF_ParseMessagePointcloud           1153 ns       1153 ns     632874
RTI_ParseMessageFullHDImage        137480 ns     137337 ns       4723
RBF_ParseMessageFullHDImage           694 ns        693 ns    1025783
```
