# QNAP-EC - HWMon Driver for QNAP IT8528 E.C. Chips

A Linux hwmon driver kernel module for the QNAP IT8528 Embedded Controller chip (and possibly others). This driver supports reading the fan speeds and temperatures as well as reading and writing the fan P.W.M. values from the ITE Tech Inc. IT8528 embedded controller chip that is used in many QNAP NAS models. Because the IT8528 chip can run custom firmware this driver is most likely specific to the firmware that QNAP uses on these chips. It is based on the reverse engineering knowledge originally gathered by [guedou](https://github.com/guedou) with lots of operational and testing help provided by [r-pufky](https://github.com/r-pufky).

In order to provide the greatest compatibility, this driver uses a library that is supplied by QNAP in its NAS operating system. The `libuLinux_hal.so` library that is part of this repository was taken from a QNAP-TS873A model running QTS 4.5.4.1800. In order to ensure proper functionality, you should replace the `libuLinux_hal.so` library file with one from the operating system image for the exact QNAP NAS model you will be running this driver on. Because this driver uses the QNAP library it is conceivable that it will work with other chips used by QNAP that are supported by the `libuLinux_hal.so` library.

Using a vanilla Ubuntu 20.04.2.0 Live DVD system as an example, you can build this driver by running the following commands:

```bash
sudo apt install build-essential git
git clone https://github.com/piwi3910/QNAP-EC
cd QNAP-EC
sudo make install
```

This will compile, link, and install the needed files along with inserting the module into the kernel (it uses `modprobe` to insert the module into the kernel which will NOT persist after a reboot).

If you would like the kernel module to skip checking for the presence of the IT8528 chip (for example, to run it on a QNAP NAS unit with a different chip to see if this driver will work) run the following command when inserting the module into the kernel:

```bash
sudo modprobe qnap-ec check-for-chip=no
```

For development purposes, there is a simulated `libuLinux_hal.so` library included that can be used when developing on a machine that doesn’t have a compatible embedded controller chip. To build the simulated library, run the following command:

```bash
make sim-lib
```

This will replace the `libuLinux_hal.so` library with the simulated library so that running `sudo make install` will install the simulated library (don't forget to include the `check-for-chip=no` module parameter when inserting the module into the kernel to skip the check for the presence of the IT8528 chip).

To uninstall the driver completely, run the following command:

```bash
sudo make uninstall
```

This driver has three components: the kernel module file called `qnap-ec.ko`, which would be installed in the `/lib/modules/5.8.0-43-generic/extra` directory on a vanilla Ubuntu 20.04.2.0 Live DVD system; the helper program file called `qnap-ec`, which would be installed in the `/usr/local/sbin` directory (on a vanilla Ubuntu 20.04.2.0 Live DVD system); and the QNAP library file called `libuLinux_hal.so`, which would be installed in the `/usr/local/lib` directory (on a vanilla Ubuntu 20.04.2.0 Live DVD).

If this driver is being installed on a Linux distribution with a different folder structure (for example, Unraid), the files will need to be manually installed. The `qnap-ec.ko` kernel module file location will depend on where the system usually expects kernel modules to be located. The `qnap-ec` helper program file will need to reside in one of the following locations in order for the kernel module to be able to call it correctly (the first two locations are only valid if this driver is not being packaged):

```
/usr/local/sbin
/usr/local/bin
/usr/sbin
/usr/bin
/sbin
/bin
```

And the `libuLinux_hal.so` QNAP library file will need to be in a location where the dynamic linker will be able to find it.

If you would like to create a package containing this driver, run the following command which uses the `package` make target in combination with `DESTDIR` to create the necessary files and folders in the package staging location:

```bash
sudo make package DESTDIR=full_path_to_package_staging_location
```

You can also add `DEBIAN=yes` or `SLACKWARE=yes` to the package command to copy the package describing `control` or `slack-desc` files to the staging location.

## User Experience and Additional Notes

I got the fans working. I am not sure the drive fans are working from a relevant temperature, but they are in a range I am comfortable with.

In order to get `apt` to install packages, you have to run the following command. Let me say, everywhere said this is a bad idea and you can really screw stuff up by installing the wrong packages or even updating the wrong package.

```
install-dev-tools
```

Then you need to be able to see the fans. I used the [GitHub - Stonyx/QNAP-EC](https://github.com/Stonyx/QNAP-EC). This isn’t specifically written for my model but it worked to add a driver so I can read the fan values and write them.

```bash
apt install build-essential git
git clone https://github.com/piwi3910/QNAP-EC
cd QNAP-EC
make install
sudo modprobe qnap-ec check-for-chip=no
```

*(Put the last command in a startup script; it has to happen every time to read fans.)*

If you run `sensors` and it shows your fan's speed then you can control them.

To do it, install `fancontrol`:

```bash
apt install fancontrol
```

Run:

```bash
pwmconfig
```

This will walk you through the config file creation. It kind of worked for me but I had to edit mine a bit to make it work. I wasn’t sure which temp to link to my hard drive fans. I took a guess and they are running in a way I like based on the temp I am seeing. The CPU one is good to go. After this, if you run `fancontrol` it should control the fans by the temperature you have linked them with.

Then you need `fancontrol` to start up and run in the background. I did a startup command for that. I haven’t rebooted, but it worked when I ran it from the command line.

```bash
nohup fancontrol &
```

All the stuff that you install with `apt` will probably get wiped out when you update; at least that was suggested in some of the things I read. So, you will have to do it all over again when you update.
