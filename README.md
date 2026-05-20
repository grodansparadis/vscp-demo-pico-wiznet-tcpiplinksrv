# vscp-demo-pico-wiznet-tcpiplinksrv

vscp tcp/ip link protocol demo for wiznet  W5100S HAT on a Raspberry Pi Pico. The VSCP link protocol is described [here](https://grodansparadis.github.io/vscp-doc-spec/#/./vscp_tcpiplink). This demo implements a VSCP Level II node that can handle a few I/O's and communicate using the link protocol. This means tools like VSCP Works and user code using the client tools can communicate with the module with ease.

## Setting up

You need the Raspberry Pi Pico SDK for this demo to compile. Install instructions can be found [here](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf). After the install make shure you have an environment variable PICO_SDK_PATH that points to the pico-sdk directory. Something like

```bash
# pico 
export PICO_SDK_PATH=/usr/local/src/pico/pico-sdk
# pico examples
export PICO_EXAMPLES_PATH=/usr/local/src/pico-examples
# pico extras
export PICO_EXTRAS_PATH=/usr/local/src/pico-extras
# pico playground
export PICO_PLAYGROUND_PATH=/usr/local/src/pico-playground
```

in .bashrc will do good (Only PICO_SDK_PATH is needed).

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

Copy the generated demo.uf2 to the pico. (Hold down BOOTSEL and toggle RESET to get shared drive (RS_PI2) to copy it to)


git clone --recurse-submodules https://github.com/WIZnet-ioNIC/WIZnet-PICO-C fetch the Wiznet Ionic code. Already used as a submodule in this project.


# Setting up the Pico SDK

If you want to make the installation as simple as possible just follow the steps below (you can simply cut and paste these commands into your terminal):

**Step 1.** Create a new directory in your home directory to keep everything tidily in one place:

```bash
cd ~
mkdir pico
cd pico
```

**Step 2.** Install the Pico SDK and examples:

```bash
git clone -b master https://github.com/raspberrypi/pico-sdk.git

# Set the PICO_SDK_PATH environment variable to where you just cloned the repo.
export PICO_SDK_PATH=/path/to/pico-sdk

a good place is to put it in .bashrc for later

cd pico-sdk
git submodule update --init
cd ..
# and at your will
git clone -b master https://github.com/raspberrypi/pico-examples.git
git clone -b master https://github.com/raspberrypi/pico-playground.git
git clone -b master https://github.com/raspberrypi/pico-extras.git
```


**Step 3.** Install the toolchain needed to build Pico projects.


**Debian Linux**
```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi build-essential
```

**macos** (Using Homebrew)
```bash
# Install cmake
brew install cmake

# Install the arm eabi toolchain
brew install --cask gcc-arm-embedded

# The equivalent to build-essential on linux, you probably already have this.
xcode-select --install
```

**Step 4.** Install picotool (if you need it).

picotool is a tool for working with RP2040/RP2350 binaries, and interacting with RP2040/RP2350 devices when they are in BOOTSEL mode. (As of version 1.1 of picotool it is also possible to interact with devices that are not in BOOTSEL mode, but are using USB stdio support from the Raspberry Pi Pico SDK by using the -f argument of picotool).

The repository [pico-sdk-tools](https://github.com/raspberrypi/pico-sdk-tools/releases) have the binary if you don't want to build it yourself. 

**Step 5.** Install openocd (if you need it).

```bash
sudo apt install openocd
```
scripts will be in _/usr/share/openocd/scripts_

or if you want to [build it from source](https://openocd.org/doc-release/README)

In this case scripts will be in _/usr/local/share/openocd/scripts_ if you don't activly change it

There are precompiled binaries [here](https://github.com/raspberrypi/pico-sdk-tools/releases) for all platforms.

## Build

Got to the firmware folder. Create a new folder "build" with

```bash
mkdir build
cd build
```
and enter it. If you plan to debug the project issue

```bash
cmake -DCMAKE_BUILD_TYPE=DEBUG ..
```

and if not (Release build) do

```bash
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
```

You can now build the project with

```bash
make
```

See the task script below for shortkeys.

## Upload firmware using openocd

```bash
sudo openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program demo.elf verify reset exit"
```

## Standalone debug session

```bash
sudo openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000"
```

you should see something like this

```bash
Open On-Chip Debugger 0.12.0+dev-00004-gacff23ffd (2026-05-19-08:05)
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : [rp2040.core0] Hardware thread awareness created
Info : [rp2040.core1] Hardware thread awareness created
adapter speed: 5000 kHz
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : Using CMSIS-DAPv2 interface with VID:PID=0x2e8a:0x000c, serial=E6633861A3473138
Info : CMSIS-DAP: SWD supported
Info : CMSIS-DAP: Atomic commands supported
Info : CMSIS-DAP: Test domain timer supported
Info : CMSIS-DAP: FW Version = 2.0.0
Info : CMSIS-DAP: Interface Initialised (SWD)
Info : SWCLK/TCK = 0 SWDIO/TMS = 0 TDI = 0 TDO = 0 nTRST = 0 nRESET = 0
Info : CMSIS-DAP: Interface ready
Info : clock speed 5000 kHz
Info : SWD DPIDR 0x0bc12477 DPv2
Info : multidrop DLPIDR 0x00000001
Info : SWD DPIDR 0x0bc12477 DPv2
Info : multidrop DLPIDR 0x10000001
Info : [rp2040.core0] Cortex-M0+ r0p1 processor detected
Info : [rp2040.core0] target has 4 breakpoints, 2 watchpoints
Info : [rp2040.core0] Examination succeed
Info : [rp2040.core1] Cortex-M0+ r0p1 processor detected
Info : [rp2040.core1] target has 4 breakpoints, 2 watchpoints
Info : [rp2040.core1] Examination succeed
Info : [rp2040.core0] starting gdb server on 3333
Info : Listening on port 3333 for gdb connections
```

You can now start a debug session manually

```bash
> gdb-multiarch blink.elf
> target remote localhost:3333
> monitor reset init
> continue
```

you can set breakpoints and singlestep etc.

Better yet (for most people) is to use visual studio code and debug in the graphics environment. Se tasks.json below


## launch.json for Visual Studio Code.

This is the launch code I use

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
            // This may need to be "arm-none-eabi-gdb" for some previous builds
            "gdbPath" : "gdb-multiarch",
            "device": "RP2040",
            "configFiles": [
                "interface/cmsis-dap.cfg",
                "target/rp2040.cfg"
            ],
            "svdFile": "${workspaceFolder}/firmware/third-party/pico-sdk/src/rp2040/hardware_regs/RP2040.svd",
            "runToEntryPoint": "main",
            "openOCDLaunchCommands": ["adapter speed 5000"],
            // Work around for stopping at main on restart
            "postRestartCommands": [
                "break main",
                "continue"
            ],
            "showDevDebugOutput": "parsed",
            "searchDir": ["/usr/local/share/openocd/scripts/"]
        }
    ]
}
```

##  tasks.json for Visual Studio Code.

This is the tasks file I use.

```bash
{
    "version": "2.0.0",
    "tasks": [
      {
        "label": "Build Project",
        "type": "process",
        "isBuildCommand": true,
        "command": "make",
        "args": [
          "-C",
          "${workspaceFolder}/firmware/build"
        ],
        "group": "build",
        "presentation": {
          "reveal": "always",
          "panel": "dedicated"
        },
        "problemMatcher": "$gcc",
        "windows": {
          "command": "make"
        }
      },
      {
        "label": "Rebuild Project",
        "type": "process",
        "isBuildCommand": true,
        "command": "make",
        "args": [
          "-C",
          "${workspaceFolder}/firmware/build",
          "clean",
          "all"
        ],
        "group": "build",
        "presentation": {
          "reveal": "always",
          "panel": "dedicated"
        },
        "problemMatcher": "$gcc",
        "windows": {
          "command": "make"
        }
      },
      {
        "label": "Flash current project",
        "type": "process",
        "isBuildCommand": true,
        "command": "openocd",
        "args": [
          "-s",
          "/usr/local/share/openocd/scripts",
          "-f",
          "interface/cmsis-dap.cfg",
          "-f",
          "target/rp2040.cfg",
          "-c",
          "adapter speed 5000; program \"${workspaceFolder}/firmware/build/demo.elf\" verify reset exit"
        ],
        "group": "build",
        "presentation": {
          "reveal": "always",
          "panel": "dedicated"
        },
        "problemMatcher": [],
        "windows": {
          "command": "C:\\Program Files (x86)\\OpenOCD\\bin\\openocd.exe"
        },
      }
    ]
}
```

To activate the kit in VS Code:

- Open the Command Palette (Ctrl+Shift+P)
- Run CMake: Select a Kit
- Choose Pico


Working with the project

- Build with **ctrl+B** "Build"
- Rebuild project with **ctr+B** "Rebuild Project"
- Flash firmware with **ctrl+B** "Flash"
- Debug with "Pico Debug" (arrow at the top left)

## picotool

You can also use picotool. 

## Notes on working with picoprobe on Ubuntu and Visual Studio Code

### Picoprobe accessrights

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
$ sudo udevadm trigger
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
    "cortex-debug.openocdPath": "<PATH TO OPENOCD>"
}
```

for reference.
