# The Botmark Benchmark v1
The Botmark benchmark was developed in the Robotics And Innovation Lab (RAIL), Trinity College Dublin as a tool for comparing and benchmarking computer platforms for their use in service robotics. This is done by running 8 workloads representative of the types of worklaods typically found in service robotics. The benchmark is available on Github under the Creative Commons (Attribution-NonCommercial-ShareAlike 4.0 International) License. Each of the workloads are compiled as a seperate executable and the results are appended to the results file at `res/results.txt`.


## Workloads
1. Person tracking.

2. Slam.

3. Facial recognition.

4. Speech synthesis.

5. Voice recognition.

6. Object recognition.

7. Motion planning.

8. Path planning.





## Installing and running the benchmark
The following comands will achieve this:

1. Get the source code.

`git clone https://github.com/AndrewMurtagh/BotmarkV1.git`

2. Check what dependencies are already installed on your system and install the remaining ones with the following script.

```
cd scripts/
. provision_environment.sh
```

3. Build the benchmark.

`. build_project.sh`

4. Get system details.

`. get_system_details.sh`

5. Run the benchmark.

`. run_benchmark.sh`




## Options
There are three options for running the benchmark which are set in the `src/botmarkcommon.h` file. Note: the benchmark will have to be recompiled for the changes to take place.

1. The number of runs.

2. Verbose mode - prints update and error status.

3. Visualise mode - displays or plays a visualisation of the workload. Should also set verbose mode.




## Directory Structure
* bin/ - includes all binary programs.
* build/ - includes files for building the package.
* cmake/ - includes files for finding CMake packages.
* CMakeLists.txt - cmake file for building the project.
* data/ - data used for benchmarking, separated into subdirectories per workload.
* res/ - various resources including the 'results.txt' file.
* scripts/ - various scripts and tools.
* src/ - source files for the workloads.
* README.md - you are here.



## Dependencies
1. Cmake (3.5.1)
2. Boost (1.58.0)
3. FCL (0.5.0)
4. OpenRave (latest)
6. MRPT (1.5)
7. Caffe (latest)
8. SphinxBase (latest)
9. Pocketshinx (latest)
10. Opencv (2.4.9)
11. Festival (latest)
12. PCL (1.9.1)
13. GMapping (latest)



## Notes
* Sometimes you must zoom in/out to view objects in the PCL visualiser.
* SpeechSynthesis doesn't exit smoothly with CTRL+C.
* No Visualisation mode for Speech Synthesis.
* Person tracking required hard coding ground coefficients and manual tuning.
* Timing starts once the data is in RAM.
* Nothing else was running when the benchmark was running.
* Motion planning ends on segfault.
* Opencv was compiled without opencl.
* Sometimes underrun occures in alsa.
* HURIC are 48kHz .wav and pocketsphinx requies raw, so they were converted using sox: http://sox.sourceforge.net/ and the bash file at http://benjgorman.com/working-with-pocketsphinx/.
* Even with verbose mode off and a null stream to capture the messages, the GMapping package in the SLAM workload still forces output messages to the terminal.



## Troubleshooting
* Might have to update gcc
* Might have to run `ldconfig` if there are troubles compiling.
* If you get the following error when running speech synthesis workload: `SIOD ERROR: ran out of storage`, try increasing the heap size.
* Might have to run `build_project.sh` as sudo.



## Authors
* Andrew Murtagh (murtagan@tcd.ie)
* Patrick Lynch (plynch13@tcd.ie)
