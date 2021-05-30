[![GitHub Pages](https://img.shields.io/website.svg?label=stnolting.github.io%2Fneorv32&longCache=true&style=flat-square&url=http%3A%2F%2Fstnolting.github.io%2Fneorv32%2Findex.html&logo=GitHub)](https://stnolting.github.io/neorv32)
[![Processor](https://img.shields.io/github/workflow/status/stnolting/neorv32/Processor/master?longCache=true&style=flat-square&label=Processor&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3AProcessor)
[![riscv-arch-test](https://img.shields.io/github/workflow/status/stnolting/neorv32/riscv-arch-test/master?longCache=true&style=flat-square&label=riscv-arch-test&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3Ariscv-arch-test)
[![Documentation](https://img.shields.io/github/workflow/status/stnolting/neorv32/Documentation/master?longCache=true&style=flat-square&label=Documentation&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3ADocumentation)
[![Implementation](https://img.shields.io/github/workflow/status/stnolting/neorv32/Implementation/master?longCache=true&style=flat-square&label=Implementation&logo=Github%20Actions&logoColor=fff)](https://github.com/stnolting/neorv32/actions?query=workflow%3AImplementation)

[![NEORV32](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/neorv32_logo_dark.png)](https://github.com/stnolting/neorv32)

# The NEORV32 RISC-V Processor

[![license](https://img.shields.io/github/license/stnolting/neorv32?longCache=true&style=flat-square)](https://github.com/stnolting/neorv32/blob/master/LICENSE)
[![release](https://img.shields.io/github/v/release/stnolting/neorv32?longCache=true&style=flat-square&logo=GitHub)](https://github.com/stnolting/neorv32/releases)
[![datasheet (pdf)](https://img.shields.io/badge/data%20sheet-PDF-ffbd00?longCache=true&style=flat-square&logo=asciidoctor)](https://github.com/stnolting/neorv32/releases/tag/nightly)
[![datasheet (html)](https://img.shields.io/badge/data%20sheet-HTML-ffbd00?longCache=true&style=flat-square&logo=asciidoctor)](https://stnolting.github.io/neorv32)
[![doxygen](https://img.shields.io/badge/doxygen-HTML-ffbd00?longCache=true&style=flat-square&logo=Doxygen)](https://stnolting.github.io/neorv32/sw/files.html)

* [Overview](#Overview)
* [CPU Features](#NEORV32-CPU-Features)
* [Processor/SoC Features](#NEORV32-Processor-Features)
* [Software Framework](#NEORV32-Software-Framework)
* [FPGA Implementation Results](#FPGA-Implementation-Results)
* [Performance](#Performance)
* [**Getting Started**](#Getting-Started) :rocket:
* [Legal](#Legal)


## Overview

![neorv32 Overview](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/neorv32_processor.png)

The NEORV32 Processor is a customizable microcontroller-like system on chip (SoC) that is based on the RISC-V NEORV32 CPU.
The project is intended as auxiliary processor in larger SoC designs or as *ready-to-go* stand-alone
custom / customizable microcontroller.

:books: For detailed information take a look at the [NEORV32 documentation /datasheet (online at GitHub-pages)](https://stnolting.github.io/neorv32/).
The `asciidoc` sources can be found in [`docs/src_adoc`](https://github.com/stnolting/neorv32/blob/master/docs/src_adoc).
The *doxygen*-based documentation of the *software framework* is also available online
at [GitHub-pages](https://stnolting.github.io/neorv32/sw/files.html).

:label: The project's change log is available in [`CHANGELOG.md`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md).
To see the changes between *official* releases visit the project's [release page](https://github.com/stnolting/neorv32/releases).

:package: The [`boards`](https://github.com/stnolting/neorv32/tree/master/boards) folder provides exemplary EDA setups targeting
various FPGA boards to get you started.

:spiral_notepad: Check out the [project boards](https://github.com/stnolting/neorv32/projects) for a list of current **ideas**,
**TODOs**, features being **planned** and **work-in-progress**.

:bulb: Feel free to open a [new issue](https://github.com/stnolting/neorv32/issues) or start a
[new discussion](https://github.com/stnolting/neorv32/discussions) if you have questions, comments, ideas or bug-fixes.
Check out how to contribute in [`CONTRIBUTE.md`](https://github.com/stnolting/neorv32/blob/master/CONTRIBUTING.md).

:rocket: Check out the [quick links below](#Getting-Started) or directly jump to the documentation's
[*Let's Get It Started!*](https://stnolting.github.io/neorv32/#_lets_get_it_started) section to get started
setting up your NEORV32 setup!


### Project Key Features

* [CPU](#NEORV32-CPU-Features) plus [Processor/SoC](#NEORV32-Processor-Features) plus [Software Framework](#NEORV32-Software-Framework)
* completely described in behavioral, platform-independent VHDL - no primitives, macros, etc.
* fully synchronous design, no latches, no gated clocks
* be as small as possible (while being as RISC-V-compliant as possible) – but with a reasonable size-performance trade-off
(the processor has to fit in a Lattice iCE40 UltraPlus 5k low-power FPGA running at 22+ MHz)
* from zero to `printf("hello world!");` - completely open source and documented
* easy to use even for FPGA/RISC-V starters – intended to work *out of the box*

[[back to top](#The-NEORV32-RISC-V-Processor)]



## NEORV32 CPU Features

:books: In-depth detailed information regarding the CPU can be found in the
[online documentation - _"NEORV32 Central Processing Unit"_](https://stnolting.github.io/neorv32/#_neorv32_central_processing_unit_cpu).

The CPU (top entity: [`rtl/core/neorv32_cpu.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/core/neorv32_cpu.vhd))
implements the RISC-V 32-bit `rv32` ISA with optional extensions. It is compatible to a subset of the
*Unprivileged ISA Specification* [(Version 2.2)](https://github.com/stnolting/neorv32/blob/master/docs/references/riscv-spec.pdf)
and a subset of the *Privileged Architecture Specification* [(Version 1.12-draft)](https://github.com/stnolting/neorv32/blob/master/docs/references/riscv-privileged.pdf).
The CPU [passes](https://stnolting.github.io/neorv32/#_risc_v_compatibility) the [official RISC-V architecture tests](https://github.com/riscv/riscv-arch-test)
(see [`riscv-arch-test/README`](https://github.com/stnolting/neorv32/blob/master/riscv-arch-test/README.md)).

In order to provide a reduced-size setup the NEORV32 CPU implements a two-stages pipeline, where each stage
uses a multi-cycle processing scheme. Instruction and data accesses are conducted via independant bus interfaces,
that are multiplexed into a single SoC-bus ("modified Harvard architecture"). As a special execution safety feature,
all reserved or unimplemented instructions do raise an exception.  Furthermore, the CPU was assigned an *official*
RISC-V open-source [architecture ID](https://github.com/riscv/riscv-isa-manual/blob/master/marchid.md)

#### Currently implemented RISC-V-compatible _ISA extensions_

* `A` - atomic memory access instructions (optional)
* `B` - bit manipulation instructions (subset, optional, still experimental)
* `C` - compressed 16-bit instructions (optional)
* `E` - embedded CPU (reduced register file size) (optional)
* `I` - base integer instruction set (always enabled)
* `M` - integer multiplication and division hardware (optional)
* `U` - less-privileged `user` mode in combintation with the standard `machine` mode (optional)
* `X` - NEORV32-specific extensions (always enabled)
* `Zfinx` - IEEE-754 single-precision floating-point extensions (optional)
* `Zicsr` - control and status register access instructions (+ exception/irq system) (optional)
* `Zifencei` - instruction stream synchronization (optional)
* `PMP` - physical memory protection (optional)
* `HPM` - hardware performance monitors (optional)
* `DB` - RISC-V CPU debug mode (optional)

#### Operation modes / privilege levels

* `machine`
* `user` (`U` extension)
* `debug_mode` (`DB extension`)

#### Interrupts (machine level)

* RISC-V standard interrupts
  * _timer_ - via MTIME SoC module or via external signal
  * _external_ - via external signal
  * _software_ - via external signal
* 16 additional "fast interrupt" requests

[[back to top](#The-NEORV32-RISC-V-Processor)]



## NEORV32 Processor Features

:books: In-depth detailed information regarding the processor/SoC and the provided optional module can be found in the
[online documentation - _"NEORV32 Processors (SoC)"_](https://stnolting.github.io/neorv32/#_neorv32_processor_soc).

The NEORV32 Processor (top entity: [`rtl/core/neorv32_top.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/core/neorv32_top.vhd))
provides a full-featured SoC build around the NEORV32 CPU. It is highly configurable to allow
a flexible customization according to your needs.

Included SoC modules:
* processor-internal data and instruction memories ([DMEM](https://stnolting.github.io/neorv32/#_data_memory_dmem) /
[IMEM](https://stnolting.github.io/neorv32/#_instruction_memory_imem)) &
cache ([iCACHE](https://stnolting.github.io/neorv32/#_processor_internal_instruction_cache_icache))
* bootloader ([BOOTLDROM](https://stnolting.github.io/neorv32/#_bootloader_rom_bootrom)) with UART console and automatic
application boot from external SPI flash option
* machine system timer ([MTIME](https://stnolting.github.io/neorv32/#_machine_system_timer_mtime)), RISC-V-compatible
* watchdog timer ([WDT](https://stnolting.github.io/neorv32/#_watchdog_timer_wdt))
* two independent universal asynchronous receivers and transmitters
([UART0](https://stnolting.github.io/neorv32/#_primary_universal_asynchronous_receiver_and_transmitter_uart0) and
[UART1](https://stnolting.github.io/neorv32/#_secondary_universal_asynchronous_receiver_and_transmitter_uart1))
with optional RTS/CTS hardware flow control
* 8/16/24/32-bit serial peripheral interface controller
([SPI](https://stnolting.github.io/neorv32/#_serial_peripheral_interface_controller_spi)) with 8 dedicated chip select lines
* two wire serial interface controller ([TWI](https://stnolting.github.io/neorv32/#_two_wire_serial_interface_controller_twi))
supporting clock-stretching, compatible to the I²C standard
* general purpose parallel IO port ([GPIO](https://stnolting.github.io/neorv32/#_general_purpose_input_and_output_port_gpio)),
32xOut & 32xIn  with pin-change interrupt
* 32-bit external bus interface, Wishbone b4 compatible
([WISHBONE](https://stnolting.github.io/neorv32/#_processor_external_memory_interface_wishbone_axi4_lite))
  * wrapper for AXI4-Lite Master Interface
* PWM controller with 4 channels and 8-bit duty cycle resolution
([PWM](https://stnolting.github.io/neorv32/#_pulse_width_modulation_controller_pwm))
* ring-oscillator-based *true random* number generator ([TRNG](https://stnolting.github.io/neorv32/#_true_random_number_generator_trng))
* custom functions subsystem ([CFS](https://stnolting.github.io/neorv32/#_custom_functions_subsystem_cfs))
for tightly-coupled custom co-processor extensions
* numerically-controlled oscillator ([NCO](https://stnolting.github.io/neorv32/#_numerically_controlled_oscillator_nco))
with three independent channels
* smart LED interface ([NEOLED](https://stnolting.github.io/neorv32/#_smart_led_interface_neoled))
to directly drive WS2812-compatible (*NeoPixel(TM)*) LEDs
* on-chip debugger ([OCD](https://stnolting.github.io/neorv32/#_on_chip_debugger_ocd)) via JTGA - compatible to
the [*Minimal RISC-V Debug Specification Version 0.13.2*](https://github.com/riscv/riscv-debug-spec)
and compatible with the *OpenOCD* and *gdb*
* alternative [top entities/wrappers](https://github.com/stnolting/neorv32/blob/master/rtl/top_templates) available

:information_source: It is recommended to use the processor setup even if you want to **use the CPU in stand-alone mode**. Simply disable all the processor-internal
modules via the generics and you will get a "CPU wrapper" that already provides a minimal CPU environment and an external memory interface (like AXI4).
This setup also allows to further use the default bootloader and software framework. From this base you can start building your own processor system.

[[back to top](#The-NEORV32-RISC-V-Processor)]



## NEORV32 Software Framework

:books: In-depth detailed information regarding the software framework can be found in the
[online documentation - _"Software Framework"_](https://stnolting.github.io/neorv32/#_software_framework).

* [core libraries](https://github.com/stnolting/neorv32/tree/master/sw/lib) for high-level usage of the provided functions and peripherals
* application compilation based on GNU makefiles
* gcc-based toolchain ([pre-compiled toolchains available](https://github.com/stnolting/riscv-gcc-prebuilt))
* bootloader with UART interface console
* runtime environment for handling traps
* several [example programs](https://github.com/stnolting/neorv32/tree/master/sw/example) to get started including CoreMark, FreeRTOS and *Conway's Game of Life*
* `doxygen`-based documentation, available on [GitHub pages](https://stnolting.github.io/neorv32/sw/files.html)

[[back to top](#The-NEORV32-RISC-V-Processor)]



## FPGA Implementation Results

:books: More details regarding exemplary FPGA setups including a listing of resource utilization by each SoC module can be found in the
[online documentation - _"FPGA Implementation Results"_](https://stnolting.github.io/neorv32/#_fpga_implementation_results).


### NEORV32 CPU

Implementation results for exemplary CPU configuration generated for an **Intel Cyclone IV EP4CE22F17C6N FPGA** on
a DE0-nano board using **Intel Quartus Prime Lite 20.1** ("balanced implementation"). The timing information is derived
from the Timing Analyzer / Slow 1200mV 0C Model. No constraints were used at all.

Results generated for hardware version [`1.5.3.2`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md).

| CPU Configuration                                 | LEs  | FFs  | Memory bits | DSPs (9-bit) | f_max   |
|:--------------------------------------------------|:----:|:----:|:-----------:|:------------:|:-------:|
| `rv32i`                                           |  980 |  409 |        1024 |            0 | 123 MHz |
| `rv32i`    + `Zicsr`                              | 1835 |  856 |        1024 |            0 | 124 MHz |
| `rv32imac` + `Zicsr`                              | 2685 | 1156 |        1024 |            0 | 124 MHz |
| `rv32imac` + `Zicsr` + `u` + `Zifencei`           | 2715 | 1162 |        1024 |            0 | 122 MHz |
| `rv32imac` + `Zicsr` + `u` + `Zifencei` + `Zfinx` | 4004 | 1812 |        1024 |            7 | 121 MHz |

Setups with enabled  `E` (embedded CPU extension) provide the same LUT and FF utilization and identical f_max as the according
`I` configuration. However, the size of the register file and thus, the embedded memory utilization, is cut in half.

[[back to top](#The-NEORV32-RISC-V-Processor)]



### NEORV32 Processor

:information_source: Check out the [`boards`](https://github.com/stnolting/neorv32/tree/master/boards)
folder for exemplary setups targeting various FPGA boards.

:information_source: The hardware resources used by the processor-internal IO/peripheral modules andmemories is also available in the
[online documentation - _"NEORV32 Central Processing Unit"_](https://stnolting.github.io/neorv32/#_neorv32_central_processing_unit_cpu).

Results generated for hardware version [`1.4.9.0`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md).
If not otherwise note, the setups use *the default configuration* (like no *TRNG*),
no external memory interface and only internal instruction and data memories
(IMEM uses 16kB and DMEM uses 8kB memory space).

| Vendor  | FPGA                              | Board            | Toolchain                  | CPU Configuration                 | LUT / LE   | FF / REG   | DSP (9-bit) | Memory Bits  | BRAM / EBR | SPRAM    | Frequency     |
|:--------|:----------------------------------|:-----------------|:---------------------------|:----------------------------------|:-----------|:-----------|:------------|:-------------|:-----------|:---------|--------------:|
| Intel   | Cyclone IV `EP4CE22F17C6N`        | Terasic DE0-Nano | Quartus Prime Lite 20.1    | `rv32imcu_Zicsr_Zifencei`         | 3813 (17%) | 1904  (8%) | 0 (0%)      | 231424 (38%) |          - |        - |       119 MHz |
| Lattice | iCE40 UltraPlus `iCE40UP5K-SG48I` | [`boards/UPduino_v3`](https://github.com/stnolting/neorv32/tree/master/boards/UPduino_v3) | Radiant 2.1 (LSE) | `rv32imac_Zicsr` | 5123 (97%) | 1972 (37%) | 0 (0%) | - |   12 (40%) | 4 (100%) | *c* 24 MHz |
| Xilinx  | Artix-7 `XC7A35TICSG324-1L`       | Arty A7-35T      | Vivado 2019.2              | `rv32imcu_Zicsr_Zifencei` + `PMP` | 2465 (12%) | 1912  (5%) | 0 (0%)      |            - |    8 (16%) |        - |   *c* 100 MHz |

[[back to top](#The-NEORV32-RISC-V-Processor)]



## Performance

The NEORV32 CPU is based on a two-stages pipelined architecutre. Each stage uses a multi-cycle processing scheme.
Hence, each instruction requires several clock cycles to execute (2 cycles for ALU operations, and up to 40 cycles for divisions).
*By default* the CPU-internal shifter as well as the multiplier and divider of the `M` extension use a bit-serial approach
and require several cycles for completion. The average CPI (cycles per instruction) depends on the instruction mix of a
specific applications and also on the available CPU extensions.

The following table shows the performance results(relative CoreMark score and average cycles per instruction) for successfully
running 2000 iterations of the [CoreMark CPU benchmark](https://www.eembc.org/coremark), which reflects a pretty good "real-life" work load.
The source files are available in [sw/example/coremark](https://github.com/stnolting/neorv32/blob/master/sw/example/coremark).

~~~
**CoreMark Setup**
Hardware:       32kB IMEM, 8kB DMEM, no caches, 100MHz clock
CoreMark:       2000 iterations, MEM_METHOD is MEM_STACK
Compiler:       RISCV32-GCC 10.1.0 (rv32i toolchain)
Compiler flags: default, see makefile
Optimization:   -O3
Peripherals:    UART for printing the results
~~~

Results generated for hardware version [`1.4.9.8`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md).

| CPU (including `Zicsr` extension)           | Executable Size | CoreMark Score | CoreMarks/MHz | Total Clock Cycles | Executed Instructions | Average CPI |
|:--------------------------------------------|:---------------:|:--------------:|:-------------:|-------------------:|----------------------:|:-----------:|
| `rv32i`                                     |    28 756 bytes |          36.36 |    **0.3636** |         5595750503 |            1466028607 |    **3.82** |
| `rv32imc`                                   |    22 008 bytes |          68.97 |    **0.6897** |         2981786734 |             611814918 |    **4.87** |
| `rv32imc` + `FAST_MUL_EN` + `FAST_SHIFT_EN` |    22 008 bytes |          90.91 |    **0.9091** |         2265135174 |             611814948 |    **3.70** |

:information_source: The `FAST_MUL_EN` configuration uses DSPs for the multiplier of the `M` extension
(enabled via the `FAST_MUL_EN` generic). The `FAST_SHIFT_EN` configuration uses a barrel shifter for
CPU shift operations (enabled via the `FAST_SHIFT_EN` generic).

[[back to top](#The-NEORV32-RISC-V-Processor)]



## Getting Started

This overview provides some *quick links* to the most important sections of the :books:
[NEORV32 online documentation](https://stnolting.github.io/neorv32).

### :electric_plug: Hardware Overview

* [NEORV32 Processor](https://stnolting.github.io/neorv32/#_neorv32_processor_soc) - the SoC
  * [Top Entity - Signals](https://stnolting.github.io/neorv32/#_processor_top_entity_signals) - how to connect to the processor
  * [Top Entity - Generics](https://stnolting.github.io/neorv32/#_processor_top_entity_generics) - configuration options
  * [Address Space](https://stnolting.github.io/neorv32/#_address_space) - memory space and memory-mapped IO
  * [SoC Modules](https://stnolting.github.io/neorv32/#_processor_internal_modules) - available IO/peripheral modules and memories
  * [On-Chip Debugger](https://stnolting.github.io/neorv32/#_on_chip_debugger_ocd) - online debugging of the processor via JTAG

* [NEORV32 CPU](https://stnolting.github.io/neorv32/#_neorv32_central_processing_unit_cpu) - the RISC-V core
  * [RISC-V compatibility](https://stnolting.github.io/neorv32/#_risc_v_compatibility) - what is compatible to the specs. and what is not
  * [ISA and Extensions](https://stnolting.github.io/neorv32/#_instruction_sets_and_extensions) - available RISC-V ISA extensions
  * [CSRs](https://stnolting.github.io/neorv32/#_control_and_status_registers_csrs) - control and status registers
  * [Traps](https://stnolting.github.io/neorv32/#_traps_exceptions_and_interrupts) - interrupts and exceptions

### :floppy_disk: Software Overview

* [Core Libraries](https://stnolting.github.io/neorv32/#_core_libraries) - high-level functions for accessing the processor's peripherals
  * [Software Framework Documentation](https://stnolting.github.io/neorv32/sw/files.html) - `doxygen`-based documentation
* [Application Makefiles](https://stnolting.github.io/neorv32/#_application_makefile) - turning your application into an executable
* [Bootloader](https://stnolting.github.io/neorv32/#_bootloader) - the build-in NEORV32 bootloader

### :rocket: User Guides (see [full overview](https://stnolting.github.io/neorv32/#_lets_get_it_started))

* [Toolchain Setup](https://stnolting.github.io/neorv32/#_toolchain_setup) - install and setup RISC-V gcc
* [General Hardware Setup](https://stnolting.github.io/neorv32/#_general_hardware_setup) - setup a new NEORV32 EDA project
* [General Software Setup](https://stnolting.github.io/neorv32/#_general_software_framework_setup) - configure the software framework
* [Application Compilation](https://stnolting.github.io/neorv32/#_application_program_compilation) - compile an application using `make`
* [Upload via Bootloader](https://stnolting.github.io/neorv32/#_uploading_and_starting_of_a_binary_executable_image_via_uart) - upload and execute executables
* [Debugging via the On-Chip Debugger](https://stnolting.github.io/neorv32/#_debugging_using_the_on_chip_debugger) - step through code *online* and *in-system*

[[back to top](#The-NEORV32-RISC-V-Processor)]



## Acknowledgements

**A big shoutout to all [contributors](https://github.com/stnolting/neorv32/graphs/contributors), who helped improving this project! :heart:**

[![RISC-V](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/riscv_logo.png)](https://riscv.org/)

[RISC-V](https://riscv.org/) - Instruction Sets Want To Be Free!

Continous integration provided by [:octocat: GitHub Actions](https://github.com/features/actions) and powered by [GHDL](https://github.com/ghdl/ghdl).

![Open Source Hardware Logo https://www.oshwa.org](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/figures/oshw_logo.png)

This project is not affiliated with or endorsed by the Open Source Initiative (https://www.oshwa.org / https://opensource.org).

[[back to top](#The-NEORV32-RISC-V-Processor)]



## Legal

This project is released under the [BSD 3-Clause license](https://github.com/stnolting/neorv32/blob/master/LICENSE).
No copyright infringement intended.
For more information see the [online documentation - _"Proprietary and Legal Notice"_](https://stnolting.github.io/neorv32/#_proprietary_and_legal_notice).
Other implied or used projects might have different licensing - see their documentation to get more information.

#### Limitation of Liability for External Links

Our website contains links to the websites of third parties ("external links"). As the
content of these websites is not under our control, we cannot assume any liability for
such external content. In all cases, the provider of information of the linked websites
is liable for the content and accuracy of the information provided. At the point in time
when the links were placed, no infringements of the law were recognisable to us. As soon
as an infringement of the law becomes known to us, we will immediately remove the
link in question.

#### Citing

If you are using the NEORV32 or parts of the project in some kind of publication, please cite it as follows:

> S. Nolting, "The NEORV32 RISC-V Processor", github.com/stnolting/neorv32

[[back to top](#The-NEORV32-RISC-V-Processor)]

--------

Made with :coffee: in Hannover, Germany :eu:
