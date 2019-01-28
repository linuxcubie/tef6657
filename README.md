# tef6657
it is a kernel module to use tef6657 with linux

compile with arm-poky-linux-gnueabi-gcc

put them inside [kernel source]/drivers/media/radio/tef665x/

modify [kernel source]/drivers/media/radio/Kconfig and Makefile so the kbuild system can recognize the module.

the interface with user space is sysfs.

TODO:
- add other read and write command to sysfs interface
- create a test SW for user space.


