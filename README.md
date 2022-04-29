# vscp-demo-pico-wiznet-tcpiplinksrv

vscp tcp/ip link protocol demo for wiznet  W5100S HAT on a Raspberry Pi Pico. The VSCP link protocol is described [here](https://grodansparadis.github.io/vscp-doc-spec/#/./vscp_tcpiplink). This demo implements a VSCP Level II node that can handle a few I/O's and communicate using the link protocol. This means tools like VSCP Works and user code using the client tools can communicate with the module with ease.

Sa similar demo using MQTT is in the works.

## Setting up

You need the Raspberry Pi Pico SDK for this demo to compile. Installation instructions can be found [here](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf). After the install make shure you have an environment variable PICO_SDK_PATH that points to the pico-sdk directory. Something like

```bash
# pico 
export PICO_SDK_PATH=/usr/local/src/pico/pico-sdk
export PICO_EXAMPLES_PATH=/usr/local/src/pico-examples
export PICO_EXTRAS_PATH=/usr/local/src/pico-extras
export PICO_PLAYGROUND_PATH=/usr/local/src/pico-playground
```

in .bashrc will do good (Only PICO_SDK_PATH ios needed).

Next, checkout the project from github

```bash
git clone --recurse-submodules https://github.com/grodansparadis/vscp-demo-pico-wiznet-tcpiplinksrv.git
$ cd vscp-demo-pico-wiznet-tcpiplinksrv-sdk
$ git submodule update --init
$ mkdir build
$ cd build
$ cmake ..
$ make
```

Copy the generated demo.uf2 to the pico.

git clone --recurse-submodules https://github.com/Wiznet/RP2040-HAT-C.git

## Notes on working with picoprobe on Ubuntu and Visual Studio Code

[This](https://www.digikey.be/en/maker/projects/raspberry-pi-pico-and-rp2040-cc-part-2-debugging-with-vs-code/470abc7efb07432b82c95f6f67f184c0) is an excellent document on how to setup the system for this.

This s the launch code I use

```bash
{
  "version": "0.2.0",
  "configurations": [
      {
          "name": "Pico Debug",
          "cwd": "${workspaceRoot}",
          "executable": "${command:cmake.launchTargetPath}",
          "request": "launch",
          "type": "cortex-debug",
          "servertype": "openocd",
          // This may need to be arm-none-eabi-gdb or gdb-multiarch depending on your system
          "gdbPath" : "/usr/bin/gdb-multiarch",
          "device": "RP2040",
          "configFiles": [
            "interface/picoprobe.cfg",
            "target/rp2040.cfg"
          ],
          "svdFile": "${env:PICO_SDK_PATH}/src/rp2040/hardware_regs/rp2040.svd",
          "runToMain": true,
          // Work around for stopping at main on restart
          "postRestartCommands": [
            "break main",
            "continue"
          ],
          "searchDir": ["/home/akhe/development/pico/openocd/tcl/"],
      }
  ]
}
```

One thing that may hinder you is accessrights to the picoprobe. To solve this I added a new group *development* at the end of __/etc/groups__ and added my user to it

```
development:x:139:akhe
```

then I added a file *60-picoprobe.rules* to /etc/udev/rules.d/ with the following content

```bash
#Raspberry Pi Pico
ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0004", MODE="660", GROUP="development", TAG+="uaccess"
```

reload rules with

```bash
$ udevadm trigger
```

this give you access to the picoprobe.

My __settings.json__ is 

```bash
{
    // These settings tweaks to the cmake plugin will ensure
    // that you debug using cortex-debug instead of trying to launch
    // a Pico binary on the host
    "cmake.statusbar.advanced": {
        "debug": {
            "visibility": "hidden"
        },
        "launch": {
            "visibility": "hidden"
        },
        "build": {
            "visibility": "default"
        },
        "buildTarget": {
            "visibility": "hidden"
        }
    },
    "cmake.buildBeforeRun": true,
    "C_Cpp.default.configurationProvider": "ms-vscode.cmake-tools",
    "cortex-debug.openocdPath": "<PATH TO OPENOCD.EXE>"
}
```

for reference.
