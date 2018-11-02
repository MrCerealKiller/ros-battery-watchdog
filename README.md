# Battery Watchdog

## Description
A ROS package to monitor batteries and shut the system down in the event of
critically low voltages. It also includes battery monitor firmware based on
the voltage divider principle.

## Dependencies
- rosserial : If using the watchdog with its bundled firmware, rosserial is
required as it is used for communication

## Configuration
**calib.config**
This is used in an attempt to circumvent the imperfections of using a physical
voltage divider in order to get a _slightly_ better reading.

The principle of calibration used here is using a linear transformation
evaluated using static calibration [(see here for some theory and examples)](https://ocw.mit.edu/courses/mechanical-engineering/2-693-principles-of-oceanographic-instrument-systems-sensors-and-measurements-13-998-spring-2004/readings/part4_calibratio.pdf).

There are variables in the config file and the linear transformation can be
described as:

`<voltage_msg> = SCALE * <raw_signal> + BIAS`

When you build the project, a shell script will convert these values into a
header that is used by the firmware.

**watchdog.yaml**
This config file is a convenient place to store the requisite watchdog
rosparams:
- `topic`: the topic that the watchdog will listen to (alternatively, one could
another topic to this)
- `sample_size`: the watchdog uses a rolling mean to prevent outliers from
affecting the script. Sample size is how large of a sample the rolling mean
should keep
- `warning_thresh`: the watchdog issues warnings below this voltage
- `warning_offset`: the number of messages lower than the threshold that must
be received before the watchdog issues a warning
- `crit_thresh`: the watchdog may power off the system below this voltage
- `crit_offset`: the number of messages lower than the threshold that must be
received before the watchdog will power off the system
- `warn_rate`: warnings are issued through the terminal. This rate specifies
how long the watchdog will wait before issuing another warning (in **seconds**)

## Usage
Building the firmware may be done using Arduino CMake as it comes bundled with
rosserial. After building the package with dependencies, you may upload the
firmware to the device using:

`catkin build --no-deps watchdog --make-args watchdog_firmware_voltage_monitor-upload`

[See here for more information](http://wiki.ros.org/rosserial_arduino/Tutorials/CMake)

After this simply use the provided launch script.

## Note on shutdown permissions
By default the user running the script will likely not have permission to shut
down the computer itself. One option is running the script as su... but a more
elegant solution would be to simply set the user (or all users) to not need a
password to use shutdown in the sudoers file.

Ex:

`<user> ALL=NOPASSWD: /sbin/shutdown`
