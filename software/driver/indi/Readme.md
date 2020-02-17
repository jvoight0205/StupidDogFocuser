# Installation
indi_stupiddogfocuser driver must be compiled in order to use. It does not ship with
the rest of the indi library.

# Features

1. Closed loop focuser
2. Absolute and relative focusing
3. Acceleration built into firmware?
4. Temperature measurement
6. Sync support

# Operation

## Connecting to Stupid Dog Focuser

1. The Stupid Dog Focuser is a built on an Arduino UNO R3, so one connects with a USB A cable.
2. Arduino UNO uses a standard comm/tty port for communication
3. It runs at 9600 baud simply due to the distances involved in a remote observatory.


After connecting, how can you operate the focuser?

## Options

The Options tab contains settings for all drivers that include polling (frequency of updates), logging, and debugging.
 Additional driver-specific options can be added here. If you have any additional settings, list them in detail.


Presets
You may set pre-defined presets for common focuser positions in the Presets tab.

Preset Positions: You may set up to 3 preset positions. When you make a change, the new values will be saved
 in the driver's configuration file and are loaded automatically in subsequent uses.

Preset GOTO: Click any preset to go to that position

Issues
There are no known bugs for this driver. If you found a bug, please report it at https://github.com/jvoight0205/StupidDogFocuser/issues.


http://www.indilib.org/download.html


