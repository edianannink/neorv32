-- #################################################################################################
-- # << NEORV32 - Minimal setup without a bootloader >>                                            #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;

entity neorv32_ProcessorTop_Minimal is
  generic (
    -- General --
    CLOCK_FREQUENCY       : natural                       := 50000000;      -- clock frequency of clk_i in Hz
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN       : boolean                       := true;          -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE     : natural                       := 16*1024;       -- size of processor-internal instruction memory in bytes
    MEM_INT_IMEM_PREFETCH : boolean                       := true;          -- fetch from external memory and store on internal memory
    MEM_INT_PREFETCH_BASE : std_logic_vector(31 downto 0) := x"00004000";
    MEM_INT_IMEM_ECC_BP   : boolean                       := true;
    MEM_INT_IV_EN         : boolean                       := true;
    -- Internal Data memory --
    MEM_INT_DMEM_EN       : boolean := true;          -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE     : natural := 8*1024;        -- size of processor-internal data memory in bytes
    -- Processor peripherals --
    IO_PWM_NUM_CH         : natural := 0;             -- number of PWM channels to implement (0..12); 0 = disabled
    -- External memory interface (WISHBONE) --
    MEM_EXT_EN            : boolean := true;          -- implement external memory bus interface?
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS            : natural range 0 to 16 := 8; -- number of regions (0..16)
    PMP_MIN_GRANULARITY        : natural := 4;  
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        : boolean := true;  -- implement atomic memory operations extension?
    CPU_EXTENSION_RISCV_B        : boolean := false;  -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        : boolean := true;   -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        : boolean := false;  -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        : boolean := true;   -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        : boolean := false;   -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    : boolean := false;  -- implement 32-bit floating-point extension (using INT regs!)
    CPU_EXTENSION_RISCV_Zicntr   : boolean := true;   -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm    : boolean := true;   -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zmmul    : boolean := false;  -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu    : boolean := false;  -- implement custom (instr.) functions unit?
    -- Tuning Options --
    REGFILE_HW_RST               : boolean := true;                              -- implement full hardware reset for register file
    -- Processor peripherals --
    IO_GPIO_NUM       : natural range 0 to 64     := 8;     -- number of GPIO input/output pairs (0..64)
    IO_UART0_EN       : boolean                   := true;  -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_MTIME_EN       : boolean                   := true;
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS      : natural range 0 to 13 := 13;        -- number of implemented HPM counters (0..13)
    HPM_CNT_WIDTH     : natural range 0 to 64 := 32         -- total size of HPM counters (0..64)
  );
  port (
    -- Global control --
    clk_i  : in  std_logic;
    rstn_i : in  std_logic;
    -- Wishbone bus interface (available if MEM_EXT_EN = true) --
    wb_tag_o  : out std_ulogic_vector(02 downto 0); -- request tag
    wb_adr_o  : out std_ulogic_vector(31 downto 0); -- address
    wb_dat_i  : in  std_ulogic_vector(31 downto 0) := (others => 'U'); -- read data
    wb_dat_o  : out std_ulogic_vector(31 downto 0); -- write data
    wb_we_o   : out std_ulogic; -- read/write
    wb_sel_o  : out std_ulogic_vector(03 downto 0); -- byte enable
    wb_stb_o  : out std_ulogic; -- strobe
    wb_cyc_o  : out std_ulogic; -- valid cycle
    wb_ack_i  : in  std_ulogic := 'L'; -- transfer acknowledge
    wb_err_i  : in  std_ulogic := 'L'; -- transfer error
    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o    : out std_ulogic_vector(7 downto 0); -- parallel output
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o    : out std_ulogic; -- UART0 send data
    uart0_rxd_i    : in  std_ulogic := 'U' -- UART0 receive data
  );
end entity;

architecture neorv32_ProcessorTop_Minimal_rtl of neorv32_ProcessorTop_Minimal is

  -- internal IO connection --
  signal con_gpio_o  : std_ulogic_vector(63 downto 0);
  signal illegal_instr : std_logic := '0';

begin

  -- The core of the problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_inst: entity neorv32.neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY              => CLOCK_FREQUENCY,   -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN            => false,             -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              => MEM_INT_IMEM_EN,        -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE,      -- size of processor-internal instruction memory in bytes
    MEM_INT_IMEM_PREFETCH        => MEM_INT_IMEM_PREFETCH,  -- fetch from external memory and store on internal memory
    MEM_INT_PREFETCH_BASE        => MEM_INT_PREFETCH_BASE,
    MEM_INT_IMEM_ECC_BP          => MEM_INT_IMEM_ECC_BP,
    MEM_INT_IV_EN                => MEM_INT_IV_EN,
    -- Internal Data memory --
    MEM_INT_DMEM_EN              => MEM_INT_DMEM_EN,   -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => MEM_INT_DMEM_SIZE, -- size of processor-internal data memory in bytes
    -- Processor peripherals --
    IO_MTIME_EN                  => IO_MTIME_EN,              -- implement machine system timer (MTIME)?
    IO_PWM_NUM_CH                => IO_PWM_NUM_CH,     -- number of PWM channels to implement (0..12); 0 = disabled
    -- External memory interface (WISHBONE) --
    MEM_EXT_EN                   => MEM_EXT_EN,        -- implement external memory bus interface?
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS,               -- number of regions (0..16)
    PMP_MIN_GRANULARITY          => PMP_MIN_GRANULARITY,  
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        => CPU_EXTENSION_RISCV_A,         -- implement atomic memory operations extension?
    CPU_EXTENSION_RISCV_B        => CPU_EXTENSION_RISCV_B,         -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,         -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,         -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,         -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,         -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    => CPU_EXTENSION_RISCV_Zfinx,     -- implement 32-bit floating-point extension (using INT regs!)
    CPU_EXTENSION_RISCV_Zicntr   => CPU_EXTENSION_RISCV_Zicntr,    -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm    => CPU_EXTENSION_RISCV_Zihpm,     -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zmmul    => CPU_EXTENSION_RISCV_Zmmul,     -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu    => CPU_EXTENSION_RISCV_Zxcfu,     -- implement custom (instr.) functions unit?
    -- Tuning Options --
    REGFILE_HW_RST               => REGFILE_HW_RST,                             -- implement full hardware reset for register file
    -- Processor peripherals --
    IO_GPIO_NUM                  => IO_GPIO_NUM,         -- number of GPIO input/output pairs (0..64)
    IO_UART0_EN                  => IO_UART0_EN,         -- implement primary universal asynchronous receiver/transmitter (UART0)?
    -- HPM --
    HPM_CNT_WIDTH                => HPM_CNT_WIDTH,
    HPM_NUM_CNTS                 => HPM_NUM_CNTS
  )
  port map (
    -- Global control --
    clk_i  => clk_i,    -- global clock, rising edge
    rstn_i => rstn_i,   -- global reset, low-active, async
    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o  => con_gpio_o, -- parallel output
    -- Wishbone bus interface (available if MEM_EXT_EN = true) --
    wb_tag_o => wb_tag_o, 
    wb_adr_o => wb_adr_o,
    wb_dat_i => wb_dat_i,
    wb_dat_o => wb_dat_o,
    wb_we_o  => wb_we_o,
    wb_sel_o => wb_sel_o,
    wb_stb_o => wb_stb_o,
    wb_cyc_o => wb_cyc_o,
    wb_ack_i => wb_ack_i,
    wb_err_i => wb_err_i,
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o => uart0_txd_o, -- UART0 send data
    uart0_rxd_i => uart0_rxd_i, -- UART0 receive data
    -- instruction validator --
    illegal_instr => illegal_instr
  );

  -- GPIO --
  gpio_o(0) <= illegal_instr;
  gpio_o(1) <= illegal_instr;
  gpio_o(2) <= illegal_instr;
  gpio_o(3) <= illegal_instr;
  gpio_o(4) <= illegal_instr;
  gpio_o(5) <= illegal_instr;
  gpio_o(6) <= illegal_instr;
  gpio_o(7) <= illegal_instr;

end architecture;
