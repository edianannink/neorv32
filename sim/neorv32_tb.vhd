-- #################################################################################################
-- # << NEORV32 - VUnit Processor Testbench >>                                                     #
-- # ********************************************************************************************* #
-- # The processor is configured to use a maximum of functional units (for testing purpose).       #
-- # Use the "User Configuration" section to configure the testbench according to your needs.      #
-- # See NEORV32 data sheet for more information.                                                  #
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

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;
context vunit_lib.vc_context;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library neorv32;
use neorv32.neorv32_package.all;
use neorv32.neorv32_application_image.all; -- this file is generated by the image generator
use std.textio.all;

library osvvm;
use osvvm.RandomPkg.all;

use work.uart_rx_pkg.all;

entity neorv32_tb is
  generic (runner_cfg : string := runner_cfg_default;
           ci_mode : boolean := false);
end neorv32_tb;

architecture neorv32_tb_rtl of neorv32_tb is

  -- User Configuration ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- general --
  constant ext_imem_c              : boolean := true; -- false: use and boot from proc-internal IMEM, true: use and boot from external (initialized) simulated IMEM (ext. mem A)
  constant ext_dmem_c              : boolean := true; -- false: use proc-internal DMEM, true: use external simulated DMEM (ext. mem B)
  constant imem_size_c             : natural := 16*1024; -- size in bytes of processor-internal IMEM / external mem A
  constant dmem_size_c             : natural := 8*1024; -- size in bytes of processor-internal DMEM / external mem B
  constant f_clock_c               : natural := 100000000; -- main clock in Hz
  constant baud0_rate_c            : natural := 19200; -- simulation UART0 (primary UART) baud rate
  constant baud1_rate_c            : natural := 19200; -- simulation UART1 (secondary UART) baud rate
  constant icache_en_c             : boolean := true; -- implement i-cache
  constant icache_block_size_c     : natural := 64; -- i-cache block size in bytes
  -- simulated external Wishbone memory A (can be used as external IMEM) --
  constant ext_mem_a_base_addr_c   : std_ulogic_vector(31 downto 0) := x"00000000"; -- wishbone memory base address (external IMEM base)
  constant ext_mem_a_size_c        : natural := imem_size_c; -- wishbone memory size in bytes
  constant ext_mem_a_latency_c     : natural := 8; -- latency in clock cycles (min 1, max 255), plus 1 cycle initial delay
  -- simulated external Wishbone memory B (can be used as external DMEM) --
  constant ext_mem_b_base_addr_c   : std_ulogic_vector(31 downto 0) := x"80000000"; -- wishbone memory base address (external DMEM base)
  constant ext_mem_b_size_c        : natural := dmem_size_c; -- wishbone memory size in bytes
  constant ext_mem_b_latency_c     : natural := 8; -- latency in clock cycles (min 1, max 255), plus 1 cycle initial delay
  -- simulated external Wishbone memory C (can be used to simulate external IO access) --
  constant ext_mem_c_base_addr_c   : std_ulogic_vector(31 downto 0) := x"F0000000"; -- wishbone memory base address (default begin of EXTERNAL IO area)
  constant ext_mem_c_size_c        : natural := icache_block_size_c/2; -- wishbone memory size in bytes, should be smaller than an iCACHE block
  constant ext_mem_c_latency_c     : natural := 128; -- latency in clock cycles (min 1, max 255), plus 1 cycle initial delay
  -- simulation interrupt trigger --
  constant irq_trigger_base_addr_c : std_ulogic_vector(31 downto 0) := x"FF000000";
  -- -------------------------------------------------------------------------------------------

  -- internals - hands off! --
  constant int_imem_c       : boolean := not ext_imem_c;
  constant int_dmem_c       : boolean := not ext_dmem_c;
  constant uart0_baud_val_c : real := real(f_clock_c) / real(baud0_rate_c);
  constant uart1_baud_val_c : real := real(f_clock_c) / real(baud1_rate_c);
  constant t_clock_c        : time := (1 sec) / f_clock_c;

  -- generators --
  signal clk_gen, rst_gen : std_ulogic := '0';

  -- uart --
  signal uart0_txd, uart1_txd : std_ulogic;
  signal uart0_cts, uart1_cts : std_ulogic;

  -- gpio --
  signal gpio : std_ulogic_vector(63 downto 0);

  -- twi --
  signal twi_scl, twi_sda : std_logic;
  signal twi_scl_i, twi_scl_o, twi_sda_i, twi_sda_o : std_ulogic;

  -- 1-wire --
  signal onewire : std_logic;
  signal onewire_i, onewire_o : std_ulogic;

  -- spi & sdi --
  signal spi_csn: std_ulogic_vector(7 downto 0);
  signal spi_di, spi_do, spi_clk : std_ulogic;
  signal sdi_di, sdi_do, sdi_clk, sdi_csn : std_ulogic;

  -- irq --
  signal msi_ring, mei_ring : std_ulogic;

  -- Wishbone bus --
  type wishbone_t is record
    addr  : std_ulogic_vector(31 downto 0); -- address
    wdata : std_ulogic_vector(31 downto 0); -- master write data
    rdata : std_ulogic_vector(31 downto 0); -- master read data
    we    : std_ulogic; -- write enable
    sel   : std_ulogic_vector(03 downto 0); -- byte enable
    stb   : std_ulogic; -- strobe
    cyc   : std_ulogic; -- valid cycle
    ack   : std_ulogic; -- transfer acknowledge
    err   : std_ulogic; -- transfer error
    tag   : std_ulogic_vector(02 downto 0); -- request tag
  end record;
  signal wb_cpu, wb_mem_a, wb_mem_b, wb_mem_c, wb_irq : wishbone_t;

  -- Wishbone access latency type --
  type ext_mem_read_latency_t is array (0 to 255) of std_ulogic_vector(31 downto 0);

  -- simulated external memory c (IO) --
  signal ext_ram_c : mem32_t(0 to ext_mem_c_size_c/4-1); -- uninitialized, used to simulate external IO

  -- simulated external memory bus feedback type --
  type ext_mem_t is record
    rdata  : ext_mem_read_latency_t;
    acc_en : std_ulogic;
    ack    : std_ulogic_vector(255 downto 0);
  end record;
  signal ext_mem_a, ext_mem_b, ext_mem_c : ext_mem_t;

  constant uart0_rx_logger : logger_t := get_logger("UART0.RX");
  constant uart1_rx_logger : logger_t := get_logger("UART1.RX");
  constant uart0_rx_handle : uart_rx_t := new_uart_rx(uart0_baud_val_c, uart0_rx_logger);
  constant uart1_rx_handle : uart_rx_t := new_uart_rx(uart1_baud_val_c, uart1_rx_logger);

begin
  test_runner : process
    variable msg : msg_t;
    variable rnd : RandomPType;
  begin
    test_runner_setup(runner, runner_cfg);

    rnd.InitSeed(test_runner'path_name);

    -- Show passing checks for UART0 on the display (stdout)
    show(uart0_rx_logger, display_handler, pass);
    show(uart1_rx_logger, display_handler, pass);

    if ci_mode then
      check_uart(net, uart0_rx_handle, nul & nul);
    else
      check_uart(net, uart0_rx_handle, "Blinking LED demo program" & cr & lf);
    end if;

    if ci_mode then
      -- No need to send the full expectation in one big chunk
      check_uart(net, uart1_rx_handle, nul & nul);
      check_uart(net, uart1_rx_handle, "0/46" & cr & lf);
    end if;

    -- Wait until all expected data has been received
    --
    -- wait_until_idle can take the VC actor as argument but
    -- the more abstract view is that wait_until_idle is part
    -- of the sync VCI and to use it a VC must be cast
    -- to a sync VC
    wait_until_idle(net, as_sync(uart0_rx_handle));
    wait_until_idle(net, as_sync(uart1_rx_handle));

    -- Wait a bit more if some extra unexpected data is produced. If so,
    -- uart_rx will fail
    wait for (20 * (1e9 / baud0_rate_c)) * ns;

    test_runner_cleanup(runner);
  end process;

  -- In case we get stuck waiting there is a watchdog timeout to terminate and fail the
  -- testbench
  test_runner_watchdog(runner, 50 ms);

  -- Clock/Reset Generator ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clk_gen <= not clk_gen after (t_clock_c/2);
  rst_gen <= '0', '1' after 60*(t_clock_c/2);


  -- The Core of the Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY              => f_clock_c,     -- clock frequency of clk_i in Hz
    HART_ID                      => x"00000000",   -- hardware thread ID
    VENDOR_ID                    => x"00000000",   -- vendor's JEDEC ID
    CUSTOM_ID                    => x"12345678",   -- custom user-defined ID
    INT_BOOTLOADER_EN            => false,         -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
    -- On-Chip Debugger (OCD) --
    ON_CHIP_DEBUGGER_EN          => true,          -- implement on-chip debugger
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_B        => true,          -- implement bit-manipulation extension?
    CPU_EXTENSION_RISCV_C        => true,          -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => false,         -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => true,          -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        => true,          -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    => true,          -- implement 32-bit floating-point extension (using INT reg!)
    CPU_EXTENSION_RISCV_Zicntr   => true,          -- implement base counters?
    CPU_EXTENSION_RISCV_Zicond   => true,          -- implement conditional operations extension?
    CPU_EXTENSION_RISCV_Zihpm    => true,          -- implement hardware performance monitors?
    CPU_EXTENSION_RISCV_Zifencei => true,          -- implement instruction stream sync.?
    CPU_EXTENSION_RISCV_Zmmul    => false,         -- implement multiply-only M sub-extension?
    CPU_EXTENSION_RISCV_Zxcfu    => true,          -- implement custom (instr.) functions unit?
    -- Extension Options --
    FAST_MUL_EN                  => false,         -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => false,         -- use barrel shifter for shift operations
    CPU_IPB_ENTRIES              => 1,             -- entries is instruction prefetch buffer, has to be a power of 2, min 1
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS              => 5,             -- number of regions (0..16)
    PMP_MIN_GRANULARITY          => 4,             -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => 12,            -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                => 40,            -- total size of HPM counters (0..64)
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              => int_imem_c ,   -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => imem_size_c,   -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    MEM_INT_DMEM_EN              => int_dmem_c,    -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => dmem_size_c,   -- size of processor-internal data memory in bytes
    -- Internal Cache memory --
    ICACHE_EN                    => icache_en_c,   -- implement instruction cache
    ICACHE_NUM_BLOCKS            => 8,             -- i-cache: number of blocks (min 2), has to be a power of 2
    ICACHE_BLOCK_SIZE            => icache_block_size_c, -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         => 2,             -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2
    -- Internal Data Cache (dCACHE) --
    DCACHE_EN                    => true,          -- implement data cache
    DCACHE_NUM_BLOCKS            => 8,             -- d-cache: number of blocks (min 1), has to be a power of 2
    DCACHE_BLOCK_SIZE            => 64,            -- d-cache: block size in bytes (min 4), has to be a power of 2
    -- External memory interface --
    MEM_EXT_EN                   => true,          -- implement external memory bus interface?
    MEM_EXT_TIMEOUT              => 256,           -- cycles after a pending bus access auto-terminates (0 = disabled)
    MEM_EXT_PIPE_MODE            => false,         -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
    MEM_EXT_BIG_ENDIAN           => false,         -- byte order: true=big-endian, false=little-endian
    MEM_EXT_ASYNC_RX             => false,         -- use register buffer for RX data when false
    MEM_EXT_ASYNC_TX             => false,         -- use register buffer for TX data when false
    -- External Interrupts Controller (XIRQ) --
    XIRQ_NUM_CH                  => 32,            -- number of external IRQ channels (0..32)
    XIRQ_TRIGGER_TYPE            => (others => '1'), -- trigger type: 0=level, 1=edge
    XIRQ_TRIGGER_POLARITY        => (others => '1'), -- trigger polarity: 0=low-level/falling-edge, 1=high-level/rising-edge
    -- Processor peripherals --
    IO_GPIO_NUM                  => 64,            -- number of GPIO input/output pairs (0..64)
    IO_MTIME_EN                  => true,          -- implement machine system timer (MTIME)?
    IO_UART0_EN                  => true,          -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART0_RX_FIFO             => 32,            -- RX fifo depth, has to be a power of two, min 1
    IO_UART0_TX_FIFO             => 32,            -- TX fifo depth, has to be a power of two, min 1
    IO_UART1_EN                  => true,          -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_UART1_RX_FIFO             => 1,             -- RX fifo depth, has to be a power of two, min 1
    IO_UART1_TX_FIFO             => 1,             -- TX fifo depth, has to be a power of two, min 1
    IO_SPI_EN                    => true,          -- implement serial peripheral interface (SPI)?
    IO_SPI_FIFO                  => 4,             -- SPI RTX fifo depth, has to be zero or a power of two
    IO_SDI_EN                    => true,          -- implement serial data interface (SDI)?
    IO_SDI_FIFO                  => 4,             -- SDI RTX fifo depth, has to be zero or a power of two
    IO_TWI_EN                    => true,          -- implement two-wire interface (TWI)?
    IO_PWM_NUM_CH                => 12,            -- number of PWM channels to implement (0..12); 0 = disabled
    IO_WDT_EN                    => true,          -- implement watch dog timer (WDT)?
    IO_TRNG_EN                   => true,          -- implement true random number generator (TRNG)?
    IO_TRNG_FIFO                 => 4,             -- TRNG fifo depth, has to be a power of two, min 1
    IO_CFS_EN                    => true,          -- implement custom functions subsystem (CFS)?
    IO_CFS_CONFIG                => (others => '0'), -- custom CFS configuration generic
    IO_CFS_IN_SIZE               => 32,            -- size of CFS input conduit in bits
    IO_CFS_OUT_SIZE              => 32,            -- size of CFS output conduit in bits
    IO_NEOLED_EN                 => true,          -- implement NeoPixel-compatible smart LED interface (NEOLED)?
    IO_NEOLED_TX_FIFO            => 8,             -- NEOLED TX FIFO depth, 1..32k, has to be a power of two
    IO_GPTMR_EN                  => true,          -- implement general purpose timer (GPTMR)?
    IO_XIP_EN                    => true,          -- implement execute in place module (XIP)?
    IO_ONEWIRE_EN                => true           -- implement 1-wire interface (ONEWIRE)?
  )
  port map (
    -- Global control --
    clk_i          => clk_gen,         -- global clock, rising edge
    rstn_i         => rst_gen,         -- global reset, low-active, async
    -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
    jtag_trst_i    => '1',             -- low-active TAP reset (optional)
    jtag_tck_i     => '0',             -- serial clock
    jtag_tdi_i     => '0',             -- serial data input
    jtag_tdo_o     => open,            -- serial data output
    jtag_tms_i     => '0',             -- mode select
    -- Wishbone bus interface (available if MEM_EXT_EN = true) --
    wb_tag_o       => wb_cpu.tag,      -- request tag
    wb_adr_o       => wb_cpu.addr,     -- address
    wb_dat_i       => wb_cpu.rdata,    -- read data
    wb_dat_o       => wb_cpu.wdata,    -- write data
    wb_we_o        => wb_cpu.we,       -- read/write
    wb_sel_o       => wb_cpu.sel,      -- byte enable
    wb_stb_o       => wb_cpu.stb,      -- strobe
    wb_cyc_o       => wb_cpu.cyc,      -- valid cycle
    wb_ack_i       => wb_cpu.ack,      -- transfer acknowledge
    wb_err_i       => wb_cpu.err,      -- transfer error
    -- Advanced memory control signals (available if MEM_EXT_EN = true) --
    fence_o        => open,            -- indicates an executed FENCE operation
    fencei_o       => open,            -- indicates an executed FENCEI operation
    -- XIP (execute in place via SPI) signals (available if IO_XIP_EN = true) --
    xip_csn_o      => open,            -- chip-select, low-active
    xip_clk_o      => open,            -- serial clock
    xip_dat_i      => '1',             -- device data input
    xip_dat_o      => open,            -- controller data output
    -- GPIO (available if IO_GPIO_NUM > 0) --
    gpio_o         => gpio,            -- parallel output
    gpio_i         => gpio,            -- parallel input
    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o    => uart0_txd,       -- UART0 send data
    uart0_rxd_i    => uart0_txd,       -- UART0 receive data
    uart0_rts_o    => uart1_cts,       -- HW flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart0_cts_i    => uart0_cts,       -- HW flow control: UART0.TX allowed to transmit, low-active, optional
    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o    => uart1_txd,       -- UART1 send data
    uart1_rxd_i    => uart1_txd,       -- UART1 receive data
    uart1_rts_o    => uart0_cts,       -- HW flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart1_cts_i    => uart1_cts,       -- HW flow control: UART0.TX allowed to transmit, low-active, optional
    -- SPI (available if IO_SPI_EN = true) --
    spi_clk_o      => spi_clk,         -- SPI serial clock
    spi_dat_o      => spi_do,          -- controller data out, peripheral data in
    spi_dat_i      => spi_di,          -- controller data in, peripheral data out
    spi_csn_o      => spi_csn,         -- SPI CS
    -- SDI (available if IO_SDI_EN = true) --
    sdi_clk_i      => sdi_clk,         -- SDI serial clock
    sdi_dat_o      => sdi_do,          -- controller data out, peripheral data in
    sdi_dat_i      => sdi_di,          -- controller data in, peripheral data out
    sdi_csn_i      => sdi_csn,         -- chip-select
    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_i      => twi_sda_i,       -- serial data line sense input
    twi_sda_o      => twi_sda_o,       -- serial data line output (pull low only)
    twi_scl_i      => twi_scl_i,       -- serial clock line sense input
    twi_scl_o      => twi_scl_o,       -- serial clock line output (pull low only)
    -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
    onewire_i      => onewire_i,       -- 1-wire bus sense input
    onewire_o      => onewire_o,       -- 1-wire bus output (pull low only)
    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o          => open,            -- pwm channels
    -- Custom Functions Subsystem IO --
    cfs_in_i       => (others => '0'), -- custom CFS inputs
    cfs_out_o      => open,            -- custom CFS outputs
    -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
    neoled_o       => open,            -- async serial data line
    -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
    xirq_i         => gpio(31 downto 0), -- IRQ channels
    -- CPU Interrupts --
    mtime_irq_i    => '0',             -- machine software interrupt, available if IO_MTIME_EN = false
    msw_irq_i      => msi_ring,        -- machine software interrupt
    mext_irq_i     => mei_ring         -- machine external interrupt
  );

  -- TWI tri-state driver --
  twi_sda   <= '0' when (twi_sda_o = '0') else 'Z'; -- module can only pull the line low actively
  twi_scl   <= '0' when (twi_scl_o = '0') else 'Z';
  twi_sda_i <= std_ulogic(twi_sda);
  twi_scl_i <= std_ulogic(twi_scl);

  -- 1-Wire tri-state driver --
  onewire   <= '0' when (onewire_o = '0') else 'Z'; -- module can only pull the line low actively
  onewire_i <= std_ulogic(onewire);

  -- TWI termination (pull-ups) --
  twi_scl <= 'H';
  twi_sda <= 'H';

  -- 1-Wire termination (pull-up) --
  onewire <= 'H';

  -- SPI/SDI echo --
  sdi_clk <= spi_clk;
  sdi_csn <= spi_csn(7);
  sdi_di  <= spi_do;
  spi_di  <= sdi_do when (spi_csn(7) = '0') else spi_do;

  uart0_checker: entity work.uart_rx
    generic map (uart0_rx_handle)
    port map (
      clk => clk_gen,
      uart_txd => uart0_txd);

  uart1_checker: entity work.uart_rx
    generic map (uart1_rx_handle)
    port map (
      clk => clk_gen,
      uart_txd => uart1_txd);


  -- Wishbone Fabric ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- CPU broadcast signals --
  wb_mem_a.addr  <= wb_cpu.addr;
  wb_mem_a.wdata <= wb_cpu.wdata;
  wb_mem_a.we    <= wb_cpu.we;
  wb_mem_a.sel   <= wb_cpu.sel;
  wb_mem_a.tag   <= wb_cpu.tag;
  wb_mem_a.cyc   <= wb_cpu.cyc;

  wb_mem_b.addr  <= wb_cpu.addr;
  wb_mem_b.wdata <= wb_cpu.wdata;
  wb_mem_b.we    <= wb_cpu.we;
  wb_mem_b.sel   <= wb_cpu.sel;
  wb_mem_b.tag   <= wb_cpu.tag;
  wb_mem_b.cyc   <= wb_cpu.cyc;

  wb_mem_c.addr  <= wb_cpu.addr;
  wb_mem_c.wdata <= wb_cpu.wdata;
  wb_mem_c.we    <= wb_cpu.we;
  wb_mem_c.sel   <= wb_cpu.sel;
  wb_mem_c.tag   <= wb_cpu.tag;
  wb_mem_c.cyc   <= wb_cpu.cyc;

  wb_irq.addr    <= wb_cpu.addr;
  wb_irq.wdata   <= wb_cpu.wdata;
  wb_irq.we      <= wb_cpu.we;
  wb_irq.sel     <= wb_cpu.sel;
  wb_irq.tag     <= wb_cpu.tag;
  wb_irq.cyc     <= wb_cpu.cyc;

  -- CPU read-back signals (no mux here since peripherals have "output gates") --
  wb_cpu.rdata <= wb_mem_a.rdata or wb_mem_b.rdata or wb_mem_c.rdata or wb_irq.rdata;
  wb_cpu.ack   <= wb_mem_a.ack   or wb_mem_b.ack   or wb_mem_c.ack   or wb_irq.ack;
  wb_cpu.err   <= wb_mem_a.err   or wb_mem_b.err   or wb_mem_c.err   or wb_irq.err;

  -- peripheral select via STROBE signal --
  wb_mem_a.stb <= wb_cpu.stb when (wb_cpu.addr >= ext_mem_a_base_addr_c) and (wb_cpu.addr < std_ulogic_vector(unsigned(ext_mem_a_base_addr_c) + ext_mem_a_size_c)) else '0';
  wb_mem_b.stb <= wb_cpu.stb when (wb_cpu.addr >= ext_mem_b_base_addr_c) and (wb_cpu.addr < std_ulogic_vector(unsigned(ext_mem_b_base_addr_c) + ext_mem_b_size_c)) else '0';
  wb_mem_c.stb <= wb_cpu.stb when (wb_cpu.addr >= ext_mem_c_base_addr_c) and (wb_cpu.addr < std_ulogic_vector(unsigned(ext_mem_c_base_addr_c) + ext_mem_c_size_c)) else '0';
  wb_irq.stb   <= wb_cpu.stb when (wb_cpu.addr =  irq_trigger_base_addr_c) else '0';


  -- Wishbone Memory A (simulated external IMEM) --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  generate_ext_imem:
  if ext_imem_c generate
    ext_mem_a_access: process(clk_gen)
      variable ext_ram_a : mem32_t(0 to ext_mem_a_size_c/4-1) := mem32_init_f(application_init_image, ext_mem_a_size_c/4); -- initialized, used to simulate external IMEM
    begin
      if rising_edge(clk_gen) then
        -- control --
        ext_mem_a.ack(0) <= wb_mem_a.cyc and wb_mem_a.stb; -- wishbone acknowledge

        -- write access --
        if ((wb_mem_a.cyc and wb_mem_a.stb and wb_mem_a.we) = '1') then -- valid write access
          for i in 0 to 3 loop
            if (wb_mem_a.sel(i) = '1') then
              ext_ram_a(to_integer(unsigned(wb_mem_a.addr(index_size_f(ext_mem_a_size_c/4)+1 downto 2))))(7+i*8 downto 0+i*8) := wb_mem_a.wdata(7+i*8 downto 0+i*8);
            end if;
          end loop; -- i
        end if;

        -- read access --
        ext_mem_a.rdata(0) <= ext_ram_a(to_integer(unsigned(wb_mem_a.addr(index_size_f(ext_mem_a_size_c/4)+1 downto 2)))); -- word aligned
        -- virtual read and ack latency --
        if (ext_mem_a_latency_c > 1) then
          for i in 1 to ext_mem_a_latency_c-1 loop
            ext_mem_a.rdata(i) <= ext_mem_a.rdata(i-1);
            ext_mem_a.ack(i)   <= ext_mem_a.ack(i-1) and wb_mem_a.cyc;
          end loop;
        end if;

        -- bus output register --
        wb_mem_a.err <= '0';
        if (ext_mem_a.ack(ext_mem_a_latency_c-1) = '1') and (wb_mem_a.cyc = '1') then
          wb_mem_a.rdata <= ext_mem_a.rdata(ext_mem_a_latency_c-1);
          wb_mem_a.ack   <= '1';
        else
          wb_mem_a.rdata <= (others => '0');
          wb_mem_a.ack   <= '0';
        end if;
      end if;
    end process ext_mem_a_access;
  end generate;

  generate_ext_imem_false:
  if (ext_imem_c = false) generate
    wb_mem_a.rdata <= (others => '0');
    wb_mem_a.ack   <= '0';
    wb_mem_a.err   <= '0';
  end generate;


  -- Wishbone Memory B (simulated external DMEM) --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ext_mem_b_access: process(clk_gen)
    variable ext_ram_b : mem32_t(0 to ext_mem_b_size_c/4-1) := (others => (others => '0')); -- zero, used to simulate external DMEM
  begin
    if rising_edge(clk_gen) then
      -- control --
      ext_mem_b.ack(0) <= wb_mem_b.cyc and wb_mem_b.stb; -- wishbone acknowledge

      -- write access --
      if ((wb_mem_b.cyc and wb_mem_b.stb and wb_mem_b.we) = '1') then -- valid write access
        for i in 0 to 3 loop
          if (wb_mem_b.sel(i) = '1') then
            ext_ram_b(to_integer(unsigned(wb_mem_b.addr(index_size_f(ext_mem_b_size_c/4)+1 downto 2))))(7+i*8 downto 0+i*8) := wb_mem_b.wdata(7+i*8 downto 0+i*8);
          end if;
        end loop; -- i
      end if;

      -- read access --
      ext_mem_b.rdata(0) <= ext_ram_b(to_integer(unsigned(wb_mem_b.addr(index_size_f(ext_mem_b_size_c/4)+1 downto 2)))); -- word aligned
      -- virtual read and ack latency --
      if (ext_mem_b_latency_c > 1) then
        for i in 1 to ext_mem_b_latency_c-1 loop
          ext_mem_b.rdata(i) <= ext_mem_b.rdata(i-1);
          ext_mem_b.ack(i)   <= ext_mem_b.ack(i-1) and wb_mem_b.cyc;
        end loop;
      end if;

      -- bus output register --
      wb_mem_b.err <= '0';
      if (ext_mem_b.ack(ext_mem_b_latency_c-1) = '1') and (wb_mem_b.cyc = '1') then
        wb_mem_b.rdata <= ext_mem_b.rdata(ext_mem_b_latency_c-1);
        wb_mem_b.ack   <= '1';
      else
        wb_mem_b.rdata <= (others => '0');
        wb_mem_b.ack   <= '0';
      end if;
    end if;
  end process ext_mem_b_access;


  -- Wishbone Memory C (simulated external IO) ----------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ext_mem_c_access: process(clk_gen)
  begin
    if rising_edge(clk_gen) then
      -- control --
      ext_mem_c.ack(0) <= wb_mem_c.cyc and wb_mem_c.stb; -- wishbone acknowledge

      -- write access --
      if ((wb_mem_c.cyc and wb_mem_c.stb and wb_mem_c.we) = '1') then -- valid write access
        for i in 0 to 3 loop
          if (wb_mem_c.sel(i) = '1') then
            ext_ram_c(to_integer(unsigned(wb_mem_c.addr(index_size_f(ext_mem_c_size_c/4)+1 downto 2))))(7+i*8 downto 0+i*8) <= wb_mem_c.wdata(7+i*8 downto 0+i*8);
          end if;
        end loop; -- i
      end if;

      -- read access --
      ext_mem_c.rdata(0) <= ext_ram_c(to_integer(unsigned(wb_mem_c.addr(index_size_f(ext_mem_c_size_c/4)+1 downto 2)))); -- word aligned
      -- virtual read and ack latency --
      if (ext_mem_c_latency_c > 1) then
        for i in 1 to ext_mem_c_latency_c-1 loop
          ext_mem_c.rdata(i) <= ext_mem_c.rdata(i-1);
          ext_mem_c.ack(i)   <= ext_mem_c.ack(i-1) and wb_mem_c.cyc;
        end loop;
      end if;

      -- bus output register --
      if (ext_mem_c.ack(ext_mem_c_latency_c-1) = '1') and (wb_mem_c.cyc = '1') then
        wb_mem_c.rdata <= ext_mem_c.rdata(ext_mem_c_latency_c-1);
        wb_mem_c.ack   <= '1';
        wb_mem_c.err   <= '0';
      else
        wb_mem_c.rdata <= (others => '0');
        wb_mem_c.ack   <= '0';
        wb_mem_c.err   <= '0';
      end if;
    end if;
  end process ext_mem_c_access;


  -- Wishbone IRQ Triggers ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  irq_trigger: process(rst_gen, clk_gen)
  begin
    if (rst_gen = '0') then
      msi_ring <= '0';
      mei_ring <= '0';
    elsif rising_edge(clk_gen) then
      -- bus interface --
      wb_irq.rdata <= (others => '0');
      wb_irq.ack   <= wb_irq.cyc and wb_irq.stb and wb_irq.we and and_reduce_f(wb_irq.sel);
      wb_irq.err   <= '0';
      -- trigger RISC-V platform IRQs --
      if ((wb_irq.cyc and wb_irq.stb and wb_irq.we and and_reduce_f(wb_irq.sel)) = '1') then
        msi_ring <= wb_irq.wdata(03); -- machine software interrupt
        mei_ring <= wb_irq.wdata(11); -- machine software interrupt
      end if;
    end if;
  end process irq_trigger;


end neorv32_tb_rtl;
