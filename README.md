# vscp-demo-pico-wiznet-tcpiplinksrv

vscp tcp/ip link protocol demo for wiznet  W5100S HAT on a Raspberry Pi Pico

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