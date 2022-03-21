sherkbat
========

An extremely jank battery driver for Linux.

Usage
-----

Compile and load the module, then fully charge the battery and record
a discharge profile:

    ./tools/record.py > discharge-profile.txt

Let the battery get as low as it would ever get during normal
usage. Then, use the profile to calibrate the driver:

    ./tools/calibrate.py discharge-profile.txt > src/table.h
    
Finally, edit `src/config.h` to your satisfaction and recompile the
driver.
