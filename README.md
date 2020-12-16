# FreeRTOS-robustness-improvement

This project aims to introduce temporal and spatial redundancy to FreeRTOS tasks to improve the overall robustness.

### Prerequisites

The required toolchain used to develop this project is `gnu arm embedded`. It can be downloaded [here](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads).

Linux users can commonly find it as "arm-none-eabi-" using their package manager. The version used with this project is 9.3.1 20200408.
 
Additionally this project requires an STM32F4 compatible on-chip debugger called [st-link](https://github.com/stlink-org/stlink). st-link can be built from source or obtained via the package manager.
 
### Building and flashing

The steps to building the project and flashing it to a microcontroller are fairly simple if you have a working version of make and git utilities set up. First clone the repository using the command below.

`git clone https://github.com/802-15/FreeRTOS-robustness-improvement.git`

Navigate to the source root directory.

`cd FreeRTOS-robustness-improvement/`

Run the make command to build the standard peripherals library, FreeRTOS and the user's application.

`make`

Assuming you have prepared the flashing tool (st-link by default) the firmware can be flashed by connecting the STM32F4 board using USB, and typing in:

`make flash`

### Debug options

To debug the FreeRTOS application an additional tool called [OpenOCD](http://openocd.org/) is used. After installing this tool one can run the debug rules in the Makefile.

Connect the STM32F4 board to the development computer using USB and run the gdbserver rule in one terminal instance:

`make gdbserver`

Then open up another terminal instance and type in:

`make debug`

A remote debugging session should start and the PC should be at the Reset handler. After this you can place breakpoints and use gdb normally.
