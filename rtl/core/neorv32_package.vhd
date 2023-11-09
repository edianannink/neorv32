-- #################################################################################################
-- # << NEORV32 - Main VHDL Package File (CPU and SoC) >>                                          #
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
-- # The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32       (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package neorv32_package is

-- ****************************************************************************************************************************
-- Architecture Configuration and Constants
-- ****************************************************************************************************************************

  -- Architecture Configuration -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- if register x0 is implemented as a *physical register* it has to be explicitly set to zero by the CPU hardware --
  constant reset_x0_c : boolean := true; -- has to be 'true' for the default register file rtl description (BRAM-based)

  -- max response time for processor-internal bus transactions --
  -- = cycles after which an *unacknowledged* internal bus access will timeout triggering a bus fault exception
  constant bus_timeout_c : natural := 15; -- default = 15

  -- instruction prefetch buffer depth --
  constant ipb_depth_c : natural := 2; -- hast to be a power of two, min 2, default 2

  -- instruction monitor: raise exception if multi-cycle operation times out --
  constant monitor_mc_tmo_c : natural := 9; -- = log2 of max execution cycles (default = 512 cycles)

  -- Architecture Constants -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant hw_version_c : std_ulogic_vector(31 downto 0) := x"01080908"; -- hardware version
  constant archid_c     : natural := 19; -- official RISC-V architecture ID
  constant XLEN         : natural := 32; -- native data path width, do not change!

  -- Check if we're inside the Matrix -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant is_simulation_c : boolean := false -- seems like we're on real hardware
-- pragma translate_off
-- synthesis translate_off
-- synthesis synthesis_off
-- RTL_SYNTHESIS OFF
  or true -- this MIGHT be a simulation
-- RTL_SYNTHESIS ON
-- synthesis synthesis_on
-- synthesis translate_on
-- pragma translate_on
  ;

-- ****************************************************************************************************************************
-- Processor Address Space Layout
-- ****************************************************************************************************************************

  -- Main Address Regions ---
  constant mem_imem_base_c : std_ulogic_vector(31 downto 0) := x"00000000"; -- IMEM size via generic
  constant mem_dmem_base_c : std_ulogic_vector(31 downto 0) := x"80000000"; -- DMEM size via generic
  constant mem_xip_base_c  : std_ulogic_vector(31 downto 0) := x"e0000000"; -- page (4MSBs) only!
  constant mem_xip_size_c  : natural := 256*1024*1024;
  constant mem_boot_base_c : std_ulogic_vector(31 downto 0) := x"ffffc000";
  constant mem_boot_size_c : natural := 8*1024;
  constant mem_io_base_c   : std_ulogic_vector(31 downto 0) := x"ffffe000";
  constant mem_io_size_c   : natural := 8*1024;

  -- Start of uncached memory access (page / 4MSBs only) --
  constant uncached_begin_c  : std_ulogic_vector(31 downto 0) := x"f0000000";

  -- IO Address Map --
  constant iodev_size_c      : natural := 256; -- size of a single IO device (bytes)
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe000"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe100"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe200"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe300"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe400"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe500"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe600"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe700"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe800"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffe900"; -- reserved
--constant base_???_c        : std_ulogic_vector(31 downto 0) := x"ffffea00"; -- reserved
  constant base_io_cfs_c     : std_ulogic_vector(31 downto 0) := x"ffffeb00";
  constant base_io_slink_c   : std_ulogic_vector(31 downto 0) := x"ffffec00";
  constant base_io_dma_c     : std_ulogic_vector(31 downto 0) := x"ffffed00";
  constant base_io_crc_c     : std_ulogic_vector(31 downto 0) := x"ffffee00";
  constant base_io_xip_c     : std_ulogic_vector(31 downto 0) := x"ffffef00";
  constant base_io_pwm_c     : std_ulogic_vector(31 downto 0) := x"fffff000";
  constant base_io_gptmr_c   : std_ulogic_vector(31 downto 0) := x"fffff100";
  constant base_io_onewire_c : std_ulogic_vector(31 downto 0) := x"fffff200";
  constant base_io_xirq_c    : std_ulogic_vector(31 downto 0) := x"fffff300";
  constant base_io_mtime_c   : std_ulogic_vector(31 downto 0) := x"fffff400";
  constant base_io_uart0_c   : std_ulogic_vector(31 downto 0) := x"fffff500";
  constant base_io_uart1_c   : std_ulogic_vector(31 downto 0) := x"fffff600";
  constant base_io_sdi_c     : std_ulogic_vector(31 downto 0) := x"fffff700";
  constant base_io_spi_c     : std_ulogic_vector(31 downto 0) := x"fffff800";
  constant base_io_twi_c     : std_ulogic_vector(31 downto 0) := x"fffff900";
  constant base_io_trng_c    : std_ulogic_vector(31 downto 0) := x"fffffa00";
  constant base_io_wdt_c     : std_ulogic_vector(31 downto 0) := x"fffffb00";
  constant base_io_gpio_c    : std_ulogic_vector(31 downto 0) := x"fffffc00";
  constant base_io_neoled_c  : std_ulogic_vector(31 downto 0) := x"fffffd00";
  constant base_io_sysinfo_c : std_ulogic_vector(31 downto 0) := x"fffffe00";
  constant base_io_dm_c      : std_ulogic_vector(31 downto 0) := x"ffffff00";

  -- On-Chip Debugger - Debug Module Entry Points (Code ROM) --
  constant dm_exc_entry_c  : std_ulogic_vector(31 downto 0) := x"ffffff00"; -- = base_io_dm_c + 0, exceptions entry point
  constant dm_park_entry_c : std_ulogic_vector(31 downto 0) := x"ffffff08"; -- = base_io_dm_c + 8, normal entry point

-- ****************************************************************************************************************************
-- SoC Definitions
-- ****************************************************************************************************************************

  -- SoC Clock Select -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant clk_div2_c    : natural := 0;
  constant clk_div4_c    : natural := 1;
  constant clk_div8_c    : natural := 2;
  constant clk_div64_c   : natural := 3;
  constant clk_div128_c  : natural := 4;
  constant clk_div1024_c : natural := 5;
  constant clk_div2048_c : natural := 6;
  constant clk_div4096_c : natural := 7;

  -- Internal Memory Types ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  type mem32_t is array (natural range <>) of std_ulogic_vector(31 downto 0); -- memory with 32-bit entries
  type mem8_t  is array (natural range <>) of std_ulogic_vector(07 downto 0); -- memory with 8-bit entries
  type mem13_t  is array (natural range <>) of std_ulogic_vector(12 downto 0); -- memory with 13-bit entries

  -- Internal Bus Interface -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- bus request --
  type bus_req_t is record
    addr : std_ulogic_vector(31 downto 0); -- access address
    data : std_ulogic_vector(31 downto 0); -- write data
    ben  : std_ulogic_vector(03 downto 0); -- byte enable
    stb  : std_ulogic; -- request strobe (single-shot)
    rw   : std_ulogic; -- 0=read, 1=write
    src  : std_ulogic; -- access source (1=instruction fetch, 0=data access)
    priv : std_ulogic; -- set if privileged (machine-mode) access
    rvso : std_ulogic; -- set if reservation set operation (atomic LR/SC)
  end record;

  -- bus response --
  type bus_rsp_t is record
    data : std_ulogic_vector(31 downto 0); -- read data
    ack  : std_ulogic; -- access acknowledge (single-shot)
    err  : std_ulogic; -- access error (single-shot)
  end record;

  -- source (request) termination --
  constant req_terminate_c : bus_req_t := (
    addr => (others => '0'),
    data => (others => '0'),
    ben  => (others => '0'),
    stb  => '0',
    rw   => '0',
    src  => '0',
    priv => '0',
    rvso => '0'
  );

  -- endpoint (response) termination --
  constant rsp_terminate_c : bus_rsp_t := (
    data => (others => '0'),
    ack  => '0',
    err  => '0'
  );

  -- Debug Module Interface -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- request --
  type dmi_req_t is record
    addr : std_ulogic_vector(06 downto 0);
    op   : std_ulogic_vector(01 downto 0);
    data : std_ulogic_vector(31 downto 0);
  end record;

  -- request operation --
  constant dmi_req_nop_c : std_ulogic_vector(1 downto 0) := "00"; -- no operation
  constant dmi_req_rd_c  : std_ulogic_vector(1 downto 0) := "01"; -- read access
  constant dmi_req_wr_c  : std_ulogic_vector(1 downto 0) := "10"; -- write access

  -- response --
  type dmi_rsp_t is record
    data : std_ulogic_vector(31 downto 0);
    ack  : std_ulogic;
  end record;

-- ****************************************************************************************************************************
-- RISC-V ISA Definitions
-- ****************************************************************************************************************************

  -- RISC-V 32-Bit Instruction Word Layout --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant instr_opcode_lsb_c  : natural :=  0; -- opcode bit 0
  constant instr_opcode_msb_c  : natural :=  6; -- opcode bit 6
  constant instr_rd_lsb_c      : natural :=  7; -- destination register address bit 0
  constant instr_rd_msb_c      : natural := 11; -- destination register address bit 4
  constant instr_funct3_lsb_c  : natural := 12; -- funct3 bit 0
  constant instr_funct3_msb_c  : natural := 14; -- funct3 bit 2
  constant instr_rs1_lsb_c     : natural := 15; -- source register 1 address bit 0
  constant instr_rs1_msb_c     : natural := 19; -- source register 1 address bit 4
  constant instr_rs2_lsb_c     : natural := 20; -- source register 2 address bit 0
  constant instr_rs2_msb_c     : natural := 24; -- source register 2 address bit 4
  constant instr_rs3_lsb_c     : natural := 27; -- source register 3 address bit 0
  constant instr_rs3_msb_c     : natural := 31; -- source register 3 address bit 4
  constant instr_funct7_lsb_c  : natural := 25; -- funct7 bit 0
  constant instr_funct7_msb_c  : natural := 31; -- funct7 bit 6
  constant instr_funct12_lsb_c : natural := 20; -- funct12 bit 0
  constant instr_funct12_msb_c : natural := 31; -- funct12 bit 11
  constant instr_imm12_lsb_c   : natural := 20; -- immediate12 bit 0
  constant instr_imm12_msb_c   : natural := 31; -- immediate12 bit 11
  constant instr_imm20_lsb_c   : natural := 12; -- immediate20 bit 0
  constant instr_imm20_msb_c   : natural := 31; -- immediate20 bit 21
  constant instr_funct5_lsb_c  : natural := 27; -- funct5 select bit 0
  constant instr_funct5_msb_c  : natural := 31; -- funct5 select bit 4

  -- RISC-V Opcodes -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- alu --
  constant opcode_alui_c   : std_ulogic_vector(6 downto 0) := "0010011"; -- ALU operation with immediate
  constant opcode_alu_c    : std_ulogic_vector(6 downto 0) := "0110011"; -- ALU operation
  constant opcode_lui_c    : std_ulogic_vector(6 downto 0) := "0110111"; -- load upper immediate
  constant opcode_auipc_c  : std_ulogic_vector(6 downto 0) := "0010111"; -- add upper immediate to PC
  -- control flow --
  constant opcode_jal_c    : std_ulogic_vector(6 downto 0) := "1101111"; -- jump and link
  constant opcode_jalr_c   : std_ulogic_vector(6 downto 0) := "1100111"; -- jump and link with register
  constant opcode_branch_c : std_ulogic_vector(6 downto 0) := "1100011"; -- branch
  -- memory access --
  constant opcode_load_c   : std_ulogic_vector(6 downto 0) := "0000011"; -- load
  constant opcode_store_c  : std_ulogic_vector(6 downto 0) := "0100011"; -- store
  constant opcode_amo_c    : std_ulogic_vector(6 downto 0) := "0101111"; -- atomic memory access
  constant opcode_fence_c  : std_ulogic_vector(6 downto 0) := "0001111"; -- fence / fence.i
  -- system/csr --
  constant opcode_system_c : std_ulogic_vector(6 downto 0) := "1110011"; -- system/csr access
  -- floating point operations --
  constant opcode_fop_c    : std_ulogic_vector(6 downto 0) := "1010011"; -- dual/single operand instruction
  -- official custom RISC-V opcodes - free for custom instructions --
  constant opcode_cust0_c  : std_ulogic_vector(6 downto 0) := "0001011"; -- custom-0
  constant opcode_cust1_c  : std_ulogic_vector(6 downto 0) := "0101011"; -- custom-1
  constant opcode_cust2_c  : std_ulogic_vector(6 downto 0) := "1011011"; -- custom-2
  constant opcode_cust3_c  : std_ulogic_vector(6 downto 0) := "1111011"; -- custom-3

  -- RISC-V Funct3 --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- control flow --
  constant funct3_beq_c    : std_ulogic_vector(2 downto 0) := "000"; -- branch if equal
  constant funct3_bne_c    : std_ulogic_vector(2 downto 0) := "001"; -- branch if not equal
  constant funct3_blt_c    : std_ulogic_vector(2 downto 0) := "100"; -- branch if less than
  constant funct3_bge_c    : std_ulogic_vector(2 downto 0) := "101"; -- branch if greater than or equal
  constant funct3_bltu_c   : std_ulogic_vector(2 downto 0) := "110"; -- branch if less than (unsigned)
  constant funct3_bgeu_c   : std_ulogic_vector(2 downto 0) := "111"; -- branch if greater than or equal (unsigned)
  -- memory access --
  constant funct3_lb_c     : std_ulogic_vector(2 downto 0) := "000"; -- load byte (signed)
  constant funct3_lh_c     : std_ulogic_vector(2 downto 0) := "001"; -- load half word (signed)
  constant funct3_lw_c     : std_ulogic_vector(2 downto 0) := "010"; -- load word (signed)
  constant funct3_lbu_c    : std_ulogic_vector(2 downto 0) := "100"; -- load byte (unsigned)
  constant funct3_lhu_c    : std_ulogic_vector(2 downto 0) := "101"; -- load half word (unsigned)
  constant funct3_lwu_c    : std_ulogic_vector(2 downto 0) := "110"; -- load word (unsigned)
  constant funct3_sb_c     : std_ulogic_vector(2 downto 0) := "000"; -- store byte
  constant funct3_sh_c     : std_ulogic_vector(2 downto 0) := "001"; -- store half word
  constant funct3_sw_c     : std_ulogic_vector(2 downto 0) := "010"; -- store word
  -- alu --
  constant funct3_subadd_c : std_ulogic_vector(2 downto 0) := "000"; -- sub/add via funct7
  constant funct3_sll_c    : std_ulogic_vector(2 downto 0) := "001"; -- shift logical left
  constant funct3_slt_c    : std_ulogic_vector(2 downto 0) := "010"; -- set on less
  constant funct3_sltu_c   : std_ulogic_vector(2 downto 0) := "011"; -- set on less unsigned
  constant funct3_xor_c    : std_ulogic_vector(2 downto 0) := "100"; -- xor
  constant funct3_sr_c     : std_ulogic_vector(2 downto 0) := "101"; -- shift right via funct7
  constant funct3_or_c     : std_ulogic_vector(2 downto 0) := "110"; -- or
  constant funct3_and_c    : std_ulogic_vector(2 downto 0) := "111"; -- and
  -- system/csr --
  constant funct3_env_c    : std_ulogic_vector(2 downto 0) := "000"; -- ecall, ebreak, mret, wfi, ...
  constant funct3_csrrw_c  : std_ulogic_vector(2 downto 0) := "001"; -- csr r/w
  constant funct3_csrrs_c  : std_ulogic_vector(2 downto 0) := "010"; -- csr read & set
  constant funct3_csrrc_c  : std_ulogic_vector(2 downto 0) := "011"; -- csr read & clear
  constant funct3_csril_c  : std_ulogic_vector(2 downto 0) := "100"; -- undefined/illegal csr command
  constant funct3_csrrwi_c : std_ulogic_vector(2 downto 0) := "101"; -- csr r/w immediate
  constant funct3_csrrsi_c : std_ulogic_vector(2 downto 0) := "110"; -- csr read & set immediate
  constant funct3_csrrci_c : std_ulogic_vector(2 downto 0) := "111"; -- csr read & clear immediate
  -- fence --
  constant funct3_fence_c  : std_ulogic_vector(2 downto 0) := "000"; -- fence - order IO/memory access
  constant funct3_fencei_c : std_ulogic_vector(2 downto 0) := "001"; -- fence.i - instruction stream sync

  -- RISC-V Funct12 - SYSTEM ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant funct12_ecall_c  : std_ulogic_vector(11 downto 0) := x"000"; -- ecall
  constant funct12_ebreak_c : std_ulogic_vector(11 downto 0) := x"001"; -- ebreak
  constant funct12_wfi_c    : std_ulogic_vector(11 downto 0) := x"105"; -- wfi
  constant funct12_mret_c   : std_ulogic_vector(11 downto 0) := x"302"; -- mret
  constant funct12_dret_c   : std_ulogic_vector(11 downto 0) := x"7b2"; -- dret

  -- RISC-V Floating-Point Stuff ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant float_single_c : std_ulogic_vector(1 downto 0) := "00"; -- single-precision (32-bit)
--constant float_double_c : std_ulogic_vector(1 downto 0) := "01"; -- double-precision (64-bit)
--constant float_half_c   : std_ulogic_vector(1 downto 0) := "10"; -- half-precision (16-bit)
--constant float_quad_c   : std_ulogic_vector(1 downto 0) := "11"; -- quad-precision (128-bit)

  -- number class flags --
  constant fp_class_neg_inf_c    : natural := 0; -- negative infinity
  constant fp_class_neg_norm_c   : natural := 1; -- negative normal number
  constant fp_class_neg_denorm_c : natural := 2; -- negative subnormal number
  constant fp_class_neg_zero_c   : natural := 3; -- negative zero
  constant fp_class_pos_zero_c   : natural := 4; -- positive zero
  constant fp_class_pos_denorm_c : natural := 5; -- positive subnormal number
  constant fp_class_pos_norm_c   : natural := 6; -- positive normal number
  constant fp_class_pos_inf_c    : natural := 7; -- positive infinity
  constant fp_class_snan_c       : natural := 8; -- signaling NaN (sNaN)
  constant fp_class_qnan_c       : natural := 9; -- quiet NaN (qNaN)

  -- exception flags --
  constant fp_exc_nv_c : natural := 0; -- invalid operation
  constant fp_exc_dz_c : natural := 1; -- divide by zero
  constant fp_exc_of_c : natural := 2; -- overflow
  constant fp_exc_uf_c : natural := 3; -- underflow
  constant fp_exc_nx_c : natural := 4; -- inexact

  -- special values (single-precision) --
  constant fp_single_qnan_c     : std_ulogic_vector(31 downto 0) := x"7fc00000"; -- quiet NaN
  constant fp_single_snan_c     : std_ulogic_vector(31 downto 0) := x"7fa00000"; -- signaling NaN
  constant fp_single_pos_inf_c  : std_ulogic_vector(31 downto 0) := x"7f800000"; -- positive infinity
  constant fp_single_neg_inf_c  : std_ulogic_vector(31 downto 0) := x"ff800000"; -- negative infinity
  constant fp_single_pos_zero_c : std_ulogic_vector(31 downto 0) := x"00000000"; -- positive zero
  constant fp_single_neg_zero_c : std_ulogic_vector(31 downto 0) := x"80000000"; -- negative zero

  -- RISC-V CSRs ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- user floating-point CSRs --
  constant csr_fflags_c         : std_ulogic_vector(11 downto 0) := x"001";
  constant csr_frm_c            : std_ulogic_vector(11 downto 0) := x"002";
  constant csr_fcsr_c           : std_ulogic_vector(11 downto 0) := x"003";
  -- machine trap setup --
  constant csr_mstatus_c        : std_ulogic_vector(11 downto 0) := x"300";
  constant csr_misa_c           : std_ulogic_vector(11 downto 0) := x"301";
  constant csr_mie_c            : std_ulogic_vector(11 downto 0) := x"304";
  constant csr_mtvec_c          : std_ulogic_vector(11 downto 0) := x"305";
  constant csr_mcounteren_c     : std_ulogic_vector(11 downto 0) := x"306";
  constant csr_mstatush_c       : std_ulogic_vector(11 downto 0) := x"310";
  -- machine counter setup --
  constant csr_mcountinhibit_c  : std_ulogic_vector(11 downto 0) := x"320";
  constant csr_mcyclecfg_c      : std_ulogic_vector(11 downto 0) := x"321";
  constant csr_minstretcfg_c    : std_ulogic_vector(11 downto 0) := x"322";
  constant csr_mhpmevent3_c     : std_ulogic_vector(11 downto 0) := x"323";
  constant csr_mhpmevent4_c     : std_ulogic_vector(11 downto 0) := x"324";
  constant csr_mhpmevent5_c     : std_ulogic_vector(11 downto 0) := x"325";
  constant csr_mhpmevent6_c     : std_ulogic_vector(11 downto 0) := x"326";
  constant csr_mhpmevent7_c     : std_ulogic_vector(11 downto 0) := x"327";
  constant csr_mhpmevent8_c     : std_ulogic_vector(11 downto 0) := x"328";
  constant csr_mhpmevent9_c     : std_ulogic_vector(11 downto 0) := x"329";
  constant csr_mhpmevent10_c    : std_ulogic_vector(11 downto 0) := x"32a";
  constant csr_mhpmevent11_c    : std_ulogic_vector(11 downto 0) := x"32b";
  constant csr_mhpmevent12_c    : std_ulogic_vector(11 downto 0) := x"32c";
  constant csr_mhpmevent13_c    : std_ulogic_vector(11 downto 0) := x"32d";
  constant csr_mhpmevent14_c    : std_ulogic_vector(11 downto 0) := x"32e";
  constant csr_mhpmevent15_c    : std_ulogic_vector(11 downto 0) := x"32f";
  -- machine trap handling --
  constant csr_mscratch_c       : std_ulogic_vector(11 downto 0) := x"340";
  constant csr_mepc_c           : std_ulogic_vector(11 downto 0) := x"341";
  constant csr_mcause_c         : std_ulogic_vector(11 downto 0) := x"342";
  constant csr_mtval_c          : std_ulogic_vector(11 downto 0) := x"343";
  constant csr_mip_c            : std_ulogic_vector(11 downto 0) := x"344";
  constant csr_mtinst_c         : std_ulogic_vector(11 downto 0) := x"34a";
  -- physical memory protection - configuration --
  constant csr_pmpcfg0_c        : std_ulogic_vector(11 downto 0) := x"3a0";
  constant csr_pmpcfg1_c        : std_ulogic_vector(11 downto 0) := x"3a1";
  constant csr_pmpcfg2_c        : std_ulogic_vector(11 downto 0) := x"3a2";
  constant csr_pmpcfg3_c        : std_ulogic_vector(11 downto 0) := x"3a3";
  -- physical memory protection - address --
  constant csr_pmpaddr0_c       : std_ulogic_vector(11 downto 0) := x"3b0";
  constant csr_pmpaddr1_c       : std_ulogic_vector(11 downto 0) := x"3b1";
  constant csr_pmpaddr2_c       : std_ulogic_vector(11 downto 0) := x"3b2";
  constant csr_pmpaddr3_c       : std_ulogic_vector(11 downto 0) := x"3b3";
  constant csr_pmpaddr4_c       : std_ulogic_vector(11 downto 0) := x"3b4";
  constant csr_pmpaddr5_c       : std_ulogic_vector(11 downto 0) := x"3b5";
  constant csr_pmpaddr6_c       : std_ulogic_vector(11 downto 0) := x"3b6";
  constant csr_pmpaddr7_c       : std_ulogic_vector(11 downto 0) := x"3b7";
  constant csr_pmpaddr8_c       : std_ulogic_vector(11 downto 0) := x"3b8";
  constant csr_pmpaddr9_c       : std_ulogic_vector(11 downto 0) := x"3b9";
  constant csr_pmpaddr10_c      : std_ulogic_vector(11 downto 0) := x"3ba";
  constant csr_pmpaddr11_c      : std_ulogic_vector(11 downto 0) := x"3bb";
  constant csr_pmpaddr12_c      : std_ulogic_vector(11 downto 0) := x"3bc";
  constant csr_pmpaddr13_c      : std_ulogic_vector(11 downto 0) := x"3bd";
  constant csr_pmpaddr14_c      : std_ulogic_vector(11 downto 0) := x"3be";
  constant csr_pmpaddr15_c      : std_ulogic_vector(11 downto 0) := x"3bf";
  -- machine counter setup - continued --
  constant csr_mcyclecfgh_c     : std_ulogic_vector(11 downto 0) := x"721";
  constant csr_minstretcfgh_c   : std_ulogic_vector(11 downto 0) := x"722";
  -- trigger module registers --
  constant csr_tselect_c        : std_ulogic_vector(11 downto 0) := x"7a0";
  constant csr_tdata1_c         : std_ulogic_vector(11 downto 0) := x"7a1";
  constant csr_tdata2_c         : std_ulogic_vector(11 downto 0) := x"7a2";
  constant csr_tdata3_c         : std_ulogic_vector(11 downto 0) := x"7a3";
  constant csr_tinfo_c          : std_ulogic_vector(11 downto 0) := x"7a4";
  constant csr_tcontrol_c       : std_ulogic_vector(11 downto 0) := x"7a5";
  -- debug mode registers --
  constant csr_dcsr_c           : std_ulogic_vector(11 downto 0) := x"7b0";
  constant csr_dpc_c            : std_ulogic_vector(11 downto 0) := x"7b1";
  constant csr_dscratch0_c      : std_ulogic_vector(11 downto 0) := x"7b2";
  -- NEORV32-specific (user-mode) registers --
  constant csr_cfureg0_c        : std_ulogic_vector(11 downto 0) := x"800";
  constant csr_cfureg1_c        : std_ulogic_vector(11 downto 0) := x"801";
  constant csr_cfureg2_c        : std_ulogic_vector(11 downto 0) := x"802";
  constant csr_cfureg3_c        : std_ulogic_vector(11 downto 0) := x"803";
  -- machine counters/timers --
  constant csr_mcycle_c         : std_ulogic_vector(11 downto 0) := x"b00";
--constant csr_mtime_c          : std_ulogic_vector(11 downto 0) := x"b01";
  constant csr_minstret_c       : std_ulogic_vector(11 downto 0) := x"b02";
  constant csr_mhpmcounter3_c   : std_ulogic_vector(11 downto 0) := x"b03";
  constant csr_mhpmcounter4_c   : std_ulogic_vector(11 downto 0) := x"b04";
  constant csr_mhpmcounter5_c   : std_ulogic_vector(11 downto 0) := x"b05";
  constant csr_mhpmcounter6_c   : std_ulogic_vector(11 downto 0) := x"b06";
  constant csr_mhpmcounter7_c   : std_ulogic_vector(11 downto 0) := x"b07";
  constant csr_mhpmcounter8_c   : std_ulogic_vector(11 downto 0) := x"b08";
  constant csr_mhpmcounter9_c   : std_ulogic_vector(11 downto 0) := x"b09";
  constant csr_mhpmcounter10_c  : std_ulogic_vector(11 downto 0) := x"b0a";
  constant csr_mhpmcounter11_c  : std_ulogic_vector(11 downto 0) := x"b0b";
  constant csr_mhpmcounter12_c  : std_ulogic_vector(11 downto 0) := x"b0c";
  constant csr_mhpmcounter13_c  : std_ulogic_vector(11 downto 0) := x"b0d";
  constant csr_mhpmcounter14_c  : std_ulogic_vector(11 downto 0) := x"b0e";
  constant csr_mhpmcounter15_c  : std_ulogic_vector(11 downto 0) := x"b0f";
  --
  constant csr_mcycleh_c        : std_ulogic_vector(11 downto 0) := x"b80";
--constant csr_mtimeh_c         : std_ulogic_vector(11 downto 0) := x"b81";
  constant csr_minstreth_c      : std_ulogic_vector(11 downto 0) := x"b82";
  constant csr_mhpmcounter3h_c  : std_ulogic_vector(11 downto 0) := x"b83";
  constant csr_mhpmcounter4h_c  : std_ulogic_vector(11 downto 0) := x"b84";
  constant csr_mhpmcounter5h_c  : std_ulogic_vector(11 downto 0) := x"b85";
  constant csr_mhpmcounter6h_c  : std_ulogic_vector(11 downto 0) := x"b86";
  constant csr_mhpmcounter7h_c  : std_ulogic_vector(11 downto 0) := x"b87";
  constant csr_mhpmcounter8h_c  : std_ulogic_vector(11 downto 0) := x"b88";
  constant csr_mhpmcounter9h_c  : std_ulogic_vector(11 downto 0) := x"b89";
  constant csr_mhpmcounter10h_c : std_ulogic_vector(11 downto 0) := x"b8a";
  constant csr_mhpmcounter11h_c : std_ulogic_vector(11 downto 0) := x"b8b";
  constant csr_mhpmcounter12h_c : std_ulogic_vector(11 downto 0) := x"b8c";
  constant csr_mhpmcounter13h_c : std_ulogic_vector(11 downto 0) := x"b8d";
  constant csr_mhpmcounter14h_c : std_ulogic_vector(11 downto 0) := x"b8e";
  constant csr_mhpmcounter15h_c : std_ulogic_vector(11 downto 0) := x"b8f";
  -- user counters/timers --
  constant csr_cycle_c          : std_ulogic_vector(11 downto 0) := x"c00";
  constant csr_time_c           : std_ulogic_vector(11 downto 0) := x"c01";
  constant csr_instret_c        : std_ulogic_vector(11 downto 0) := x"c02";
  constant csr_hpmcounter3_c    : std_ulogic_vector(11 downto 0) := x"c03";
  constant csr_hpmcounter4_c    : std_ulogic_vector(11 downto 0) := x"c04";
  constant csr_hpmcounter5_c    : std_ulogic_vector(11 downto 0) := x"c05";
  constant csr_hpmcounter6_c    : std_ulogic_vector(11 downto 0) := x"c06";
  constant csr_hpmcounter7_c    : std_ulogic_vector(11 downto 0) := x"c07";
  constant csr_hpmcounter8_c    : std_ulogic_vector(11 downto 0) := x"c08";
  constant csr_hpmcounter9_c    : std_ulogic_vector(11 downto 0) := x"c09";
  constant csr_hpmcounter10_c   : std_ulogic_vector(11 downto 0) := x"c0a";
  constant csr_hpmcounter11_c   : std_ulogic_vector(11 downto 0) := x"c0b";
  constant csr_hpmcounter12_c   : std_ulogic_vector(11 downto 0) := x"c0c";
  constant csr_hpmcounter13_c   : std_ulogic_vector(11 downto 0) := x"c0d";
  constant csr_hpmcounter14_c   : std_ulogic_vector(11 downto 0) := x"c0e";
  constant csr_hpmcounter15_c   : std_ulogic_vector(11 downto 0) := x"c0f";
  --
  constant csr_cycleh_c         : std_ulogic_vector(11 downto 0) := x"c80";
  constant csr_timeh_c          : std_ulogic_vector(11 downto 0) := x"c81";
  constant csr_instreth_c       : std_ulogic_vector(11 downto 0) := x"c82";
  constant csr_hpmcounter3h_c   : std_ulogic_vector(11 downto 0) := x"c83";
  constant csr_hpmcounter4h_c   : std_ulogic_vector(11 downto 0) := x"c84";
  constant csr_hpmcounter5h_c   : std_ulogic_vector(11 downto 0) := x"c85";
  constant csr_hpmcounter6h_c   : std_ulogic_vector(11 downto 0) := x"c86";
  constant csr_hpmcounter7h_c   : std_ulogic_vector(11 downto 0) := x"c87";
  constant csr_hpmcounter8h_c   : std_ulogic_vector(11 downto 0) := x"c88";
  constant csr_hpmcounter9h_c   : std_ulogic_vector(11 downto 0) := x"c89";
  constant csr_hpmcounter10h_c  : std_ulogic_vector(11 downto 0) := x"c8a";
  constant csr_hpmcounter11h_c  : std_ulogic_vector(11 downto 0) := x"c8b";
  constant csr_hpmcounter12h_c  : std_ulogic_vector(11 downto 0) := x"c8c";
  constant csr_hpmcounter13h_c  : std_ulogic_vector(11 downto 0) := x"c8d";
  constant csr_hpmcounter14h_c  : std_ulogic_vector(11 downto 0) := x"c8e";
  constant csr_hpmcounter15h_c  : std_ulogic_vector(11 downto 0) := x"c8f";
  -- machine information registers --
  constant csr_mvendorid_c      : std_ulogic_vector(11 downto 0) := x"f11";
  constant csr_marchid_c        : std_ulogic_vector(11 downto 0) := x"f12";
  constant csr_mimpid_c         : std_ulogic_vector(11 downto 0) := x"f13";
  constant csr_mhartid_c        : std_ulogic_vector(11 downto 0) := x"f14";
  constant csr_mconfigptr_c     : std_ulogic_vector(11 downto 0) := x"f15";
  -- NEORV32-specific (machine-mode) registers --
  constant csr_mxisa_c          : std_ulogic_vector(11 downto 0) := x"fc0";

-- ****************************************************************************************************************************
-- CPU Control
-- ****************************************************************************************************************************

  -- Main CPU Control Bus -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  type ctrl_bus_t is record
    -- register file --
    rf_wb_en     : std_ulogic; -- write back enable
    rf_rs1       : std_ulogic_vector(04 downto 0); -- source register 1 address
    rf_rs2       : std_ulogic_vector(04 downto 0); -- source register 2 address
    rf_rs3       : std_ulogic_vector(04 downto 0); -- source register 3 address
    rf_rd        : std_ulogic_vector(04 downto 0); -- destination register address
    rf_mux       : std_ulogic_vector(01 downto 0); -- input source select
    rf_zero_we   : std_ulogic;                     -- allow/force write access to x0
    -- alu --
    alu_op       : std_ulogic_vector(02 downto 0); -- ALU operation select
    alu_opa_mux  : std_ulogic;                     -- operand A select (0=rs1, 1=PC)
    alu_opb_mux  : std_ulogic;                     -- operand B select (0=rs2, 1=IMM)
    alu_unsigned : std_ulogic;                     -- is unsigned ALU operation
    alu_cp_trig  : std_ulogic_vector(04 downto 0); -- co-processor trigger (one-hot)
    -- load/store unit --
    lsu_req      : std_ulogic;                     -- trigger memory access request
    lsu_rw       : std_ulogic;                     -- 0: read access, 1: write access
    lsu_mo_we    : std_ulogic;                     -- memory address and data output register write enable
    lsu_fence    : std_ulogic;                     -- fence operation
    lsu_fencei   : std_ulogic;                     -- fence.i operation
    lsu_priv     : std_ulogic;                     -- effective privilege level for load/store
    -- instruction word --
    ir_funct3    : std_ulogic_vector(02 downto 0); -- funct3 bit field
    ir_funct12   : std_ulogic_vector(11 downto 0); -- funct12 bit field
    ir_opcode    : std_ulogic_vector(06 downto 0); -- opcode bit field
    -- cpu status --
    cpu_priv     : std_ulogic;                     -- effective privilege mode
    cpu_sleep    : std_ulogic;                     -- set when CPU is in sleep mode
    cpu_trap     : std_ulogic;                     -- set when CPU is entering trap exec
    cpu_debug    : std_ulogic;                     -- set when CPU is in debug mode
  end record;

  -- control bus reset initializer --
  constant ctrl_bus_zero_c : ctrl_bus_t := (
    rf_wb_en     => '0',
    rf_rs1       => (others => '0'),
    rf_rs2       => (others => '0'),
    rf_rs3       => (others => '0'),
    rf_rd        => (others => '0'),
    rf_mux       => (others => '0'),
    rf_zero_we   => '0',
    alu_op       => (others => '0'),
    alu_opa_mux  => '0',
    alu_opb_mux  => '0',
    alu_unsigned => '0',
    alu_cp_trig  => (others => '0'),
    lsu_req      => '0',
    lsu_rw       => '0',
    lsu_mo_we    => '0',
    lsu_fence    => '0',
    lsu_fencei   => '0',
    lsu_priv     => '0',
    ir_funct3    => (others => '0'),
    ir_funct12   => (others => '0'),
    ir_opcode    => (others => '0'),
    cpu_priv     => '0',
    cpu_sleep    => '0',
    cpu_trap     => '0',
    cpu_debug    => '0'
  );

  -- Comparator Bus -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant cmp_equal_c : natural := 0;
  constant cmp_less_c  : natural := 1; -- for signed and unsigned comparisons

  -- CPU Co-Processor IDs -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant cp_sel_shifter_c  : natural := 0; -- CP0: shift operations (base ISA)
  constant cp_sel_muldiv_c   : natural := 1; -- CP1: multiplication/division operations ('M' extensions)
  constant cp_sel_bitmanip_c : natural := 2; -- CP2: bit manipulation ('B' extensions)
  constant cp_sel_fpu_c      : natural := 3; -- CP3: floating-point unit ('Zfinx' extension)
  constant cp_sel_cfu_c      : natural := 4; -- CP4: custom instructions CFU ('Zxcfu' extension)

  -- ALU Function Codes [DO NOT CHANGE ENCODING!] -------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant alu_op_add_c  : std_ulogic_vector(2 downto 0) := "000"; -- result <= A + B
  constant alu_op_sub_c  : std_ulogic_vector(2 downto 0) := "001"; -- result <= A - B
  constant alu_op_cp_c   : std_ulogic_vector(2 downto 0) := "010"; -- result <= ALU co-processor
  constant alu_op_slt_c  : std_ulogic_vector(2 downto 0) := "011"; -- result <= A < B
  constant alu_op_movb_c : std_ulogic_vector(2 downto 0) := "100"; -- result <= B
  constant alu_op_xor_c  : std_ulogic_vector(2 downto 0) := "101"; -- result <= A xor B
  constant alu_op_or_c   : std_ulogic_vector(2 downto 0) := "110"; -- result <= A or B
  constant alu_op_and_c  : std_ulogic_vector(2 downto 0) := "111"; -- result <= A and B

  -- Register File Input Select -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant rf_mux_alu_c : std_ulogic_vector(1 downto 0) := "00"; -- register file <= alu result
  constant rf_mux_mem_c : std_ulogic_vector(1 downto 0) := "01"; -- register file <= memory read data
  constant rf_mux_csr_c : std_ulogic_vector(1 downto 0) := "10"; -- register file <= CSR read data
  constant rf_mux_npc_c : std_ulogic_vector(1 downto 0) := "11"; -- register file <= next-PC (for branch-and-link)

  -- Trap ID Codes --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- MSB:   1 = interrupt, 0 = sync. exception
  -- MSB-1: 1 = entry to debug mode, 0 = normal trapping
  -- RISC-V compliant synchronous exceptions --
  constant trap_ima_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00000"; -- 0: instruction misaligned
  constant trap_iaf_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00001"; -- 1: instruction access fault
  constant trap_iil_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00010"; -- 2: illegal instruction
  constant trap_brk_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00011"; -- 3: breakpoint
  constant trap_lma_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00100"; -- 4: load address misaligned
  constant trap_laf_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00101"; -- 5: load access fault
  constant trap_sma_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00110"; -- 6: store address misaligned
  constant trap_saf_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "00111"; -- 7: store access fault
  constant trap_env_c      : std_ulogic_vector(6 downto 0) := "0" & "0" & "010UU"; -- 8..11: environment call from u/s/h/m
  -- RISC-V compliant asynchronous exceptions (interrupts) --
  constant trap_msi_c      : std_ulogic_vector(6 downto 0) := "1" & "0" & "00011"; -- 3:  machine software interrupt
  constant trap_mti_c      : std_ulogic_vector(6 downto 0) := "1" & "0" & "00111"; -- 7:  machine timer interrupt
  constant trap_mei_c      : std_ulogic_vector(6 downto 0) := "1" & "0" & "01011"; -- 11: machine external interrupt
  -- NEORV32-specific (RISC-V custom) asynchronous exceptions (interrupts) --
  constant trap_firq0_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10000"; -- 16: fast interrupt 0
  constant trap_firq1_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10001"; -- 17: fast interrupt 1
  constant trap_firq2_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10010"; -- 18: fast interrupt 2
  constant trap_firq3_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10011"; -- 19: fast interrupt 3
  constant trap_firq4_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10100"; -- 20: fast interrupt 4
  constant trap_firq5_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10101"; -- 21: fast interrupt 5
  constant trap_firq6_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10110"; -- 22: fast interrupt 6
  constant trap_firq7_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "10111"; -- 23: fast interrupt 7
  constant trap_firq8_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "11000"; -- 24: fast interrupt 8
  constant trap_firq9_c    : std_ulogic_vector(6 downto 0) := "1" & "0" & "11001"; -- 25: fast interrupt 9
  constant trap_firq10_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11010"; -- 26: fast interrupt 10
  constant trap_firq11_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11011"; -- 27: fast interrupt 11
  constant trap_firq12_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11100"; -- 28: fast interrupt 12
  constant trap_firq13_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11101"; -- 29: fast interrupt 13
  constant trap_firq14_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11110"; -- 30: fast interrupt 14
  constant trap_firq15_c   : std_ulogic_vector(6 downto 0) := "1" & "0" & "11111"; -- 31: fast interrupt 15
  -- entering debug mode (sync./async. exceptions) --
  constant trap_db_break_c : std_ulogic_vector(6 downto 0) := "0" & "1" & "00001"; -- 1: break instruction (sync)
  constant trap_db_trig_c  : std_ulogic_vector(6 downto 0) := "0" & "1" & "00010"; -- 2: hardware trigger (sync)
  constant trap_db_halt_c  : std_ulogic_vector(6 downto 0) := "1" & "1" & "00011"; -- 3: external halt request (async)
  constant trap_db_step_c  : std_ulogic_vector(6 downto 0) := "1" & "1" & "00100"; -- 4: single-stepping (async)

  -- CPU Trap System ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- exception source bits --
  constant exc_iaccess_c  : natural :=  0; -- instruction access fault
  constant exc_illegal_c  : natural :=  1; -- illegal instruction
  constant exc_ialign_c   : natural :=  2; -- instruction address misaligned
  constant exc_ecall_c    : natural :=  3; -- environment call
  constant exc_ebreak_c   : natural :=  4; -- breakpoint
  constant exc_salign_c   : natural :=  5; -- store address misaligned
  constant exc_lalign_c   : natural :=  6; -- load address misaligned
  constant exc_saccess_c  : natural :=  7; -- store access fault
  constant exc_laccess_c  : natural :=  8; -- load access fault
  -- for debug mode only --
  constant exc_db_break_c : natural :=  9; -- enter debug mode via ebreak instruction ("sync EXCEPTION")
  constant exc_db_hw_c    : natural := 10; -- enter debug mode via hw trigger ("sync EXCEPTION")
  --
  constant exc_width_c    : natural := 11; -- length of this list in bits
  -- interrupt source bits --
  constant irq_msi_irq_c  : natural :=  0; -- machine software interrupt
  constant irq_mti_irq_c  : natural :=  1; -- machine timer interrupt
  constant irq_mei_irq_c  : natural :=  2; -- machine external interrupt
  constant irq_firq_0_c   : natural :=  3; -- fast interrupt channel 0
  constant irq_firq_1_c   : natural :=  4; -- fast interrupt channel 1
  constant irq_firq_2_c   : natural :=  5; -- fast interrupt channel 2
  constant irq_firq_3_c   : natural :=  6; -- fast interrupt channel 3
  constant irq_firq_4_c   : natural :=  7; -- fast interrupt channel 4
  constant irq_firq_5_c   : natural :=  8; -- fast interrupt channel 5
  constant irq_firq_6_c   : natural :=  9; -- fast interrupt channel 6
  constant irq_firq_7_c   : natural := 10; -- fast interrupt channel 7
  constant irq_firq_8_c   : natural := 11; -- fast interrupt channel 8
  constant irq_firq_9_c   : natural := 12; -- fast interrupt channel 9
  constant irq_firq_10_c  : natural := 13; -- fast interrupt channel 10
  constant irq_firq_11_c  : natural := 14; -- fast interrupt channel 11
  constant irq_firq_12_c  : natural := 15; -- fast interrupt channel 12
  constant irq_firq_13_c  : natural := 16; -- fast interrupt channel 13
  constant irq_firq_14_c  : natural := 17; -- fast interrupt channel 14
  constant irq_firq_15_c  : natural := 18; -- fast interrupt channel 15
  -- for debug mode only --
  constant irq_db_halt_c  : natural := 19; -- enter debug mode via external halt request ("async IRQ")
  constant irq_db_step_c  : natural := 20; -- enter debug mode via single-stepping ("async IRQ")
  --
  constant irq_width_c    : natural := 21; -- length of this list in bits

  -- CPU Privilege Modes --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant priv_mode_m_c : std_ulogic := '1'; -- machine mode
  constant priv_mode_u_c : std_ulogic := '0'; -- user mode

  -- HPM Event System -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  constant hpmcnt_event_cy_c          : natural := 0;  -- Active cycle
  constant hpmcnt_event_tm_c          : natural := 1;  -- Time (unused/reserved)
  constant hpmcnt_event_ir_c          : natural := 2;  -- Retired instruction
  --constant hpmcnt_event_cir_c       : natural := 3;  -- Retired compressed instruction
  --constant hpmcnt_event_wait_if_c   : natural := 4;  -- Instruction fetch memory wait cycle
  --constant hpmcnt_event_wait_ii_c   : natural := 5;  -- Instruction issue wait cycle
  --constant hpmcnt_event_wait_mc_c   : natural := 6;  -- Multi-cycle ALU-operation wait cycle
  --constant hpmcnt_event_load_c      : natural := 7;  -- Load operation
  constant hpmcnt_event_ecc_se_dmem   : natural := 3; -- Data memory single error
  constant hpmcnt_event_ecc_de_dmem   : natural := 4; -- Data memory double error
  constant hpmcnt_event_ecc_se_regfile: natural := 5; -- Register file single error
  constant hpmcnt_event_ecc_de_regfile: natural := 6; -- Register file double error
  constant hpmcnt_event_iv            : natural := 7; -- Instruction validator detecting an error
  constant hpmcnt_event_store_c       : natural := 8;  -- Store operation
  constant hpmcnt_event_wait_ls_c     : natural := 9;  -- Load/store memory wait cycle
  constant hpmcnt_event_jump_c        : natural := 10; -- Unconditional jump
  constant hpmcnt_event_branch_c      : natural := 11; -- Conditional branch (taken or not taken)
  constant hpmcnt_event_tbranch_c     : natural := 12; -- Conditional taken branch
  constant hpmcnt_event_trap_c        : natural := 13; -- Entered trap
  constant hpmcnt_event_illegal_c     : natural := 14; -- Illegal instruction exception
  --
  constant hpmcnt_event_size_c        : natural := 15; -- length of this list

-- ****************************************************************************************************************************
-- Helper Functions
-- ****************************************************************************************************************************

  function index_size_f(input : natural) return natural;
  function cond_sel_int_f(cond : boolean; val_t : integer; val_f : integer) return integer;
  function cond_sel_natural_f(cond : boolean; val_t : natural; val_f : natural) return natural;
  function cond_sel_suv_f(cond : boolean; val_t : std_ulogic_vector; val_f : std_ulogic_vector) return std_ulogic_vector;
  function cond_sel_string_f(cond : boolean; val_t : string; val_f : string) return string;
  function bool_to_ulogic_f(cond : boolean) return std_ulogic;
  function bin_to_gray_f(input : std_ulogic_vector) return std_ulogic_vector;
  function gray_to_bin_f(input : std_ulogic_vector) return std_ulogic_vector;
  function or_reduce_f(input : std_ulogic_vector) return std_ulogic;
  function and_reduce_f(input : std_ulogic_vector) return std_ulogic;
  function xor_reduce_f(input : std_ulogic_vector) return std_ulogic;
  function su_undefined_f(input : std_ulogic) return boolean;
  function to_hexchar_f(input : std_ulogic_vector(3 downto 0)) return character;
  function to_hstring32_f(input : std_ulogic_vector(31 downto 0)) return string;
  function bit_rev_f(input : std_ulogic_vector) return std_ulogic_vector;
  function is_power_of_two_f(input : natural) return boolean;
  function bswap32_f(input : std_ulogic_vector) return std_ulogic_vector;
  function popcount_f(input : std_ulogic_vector) return natural;
  function leading_zeros_f(input : std_ulogic_vector) return natural;
  impure function mem32_init_f(init : mem32_t; depth : natural) return mem32_t;

-- ****************************************************************************************************************************
-- NEORV32 Processor Top Entity (component prototype)
-- ****************************************************************************************************************************

  component neorv32_top
    generic (
      -- General --
      CLOCK_FREQUENCY              : natural;
      HART_ID                      : std_ulogic_vector(31 downto 0) := x"00000000";
      VENDOR_ID                    : std_ulogic_vector(31 downto 0) := x"00000000";
      INT_BOOTLOADER_EN            : boolean := false;
      -- On-Chip Debugger (OCD) --
      ON_CHIP_DEBUGGER_EN          : boolean := false;
      DM_LEGACY_MODE               : boolean := false;
      -- RISC-V CPU Extensions --
      CPU_EXTENSION_RISCV_A        : boolean := false;
      CPU_EXTENSION_RISCV_B        : boolean := false;
      CPU_EXTENSION_RISCV_C        : boolean := false;
      CPU_EXTENSION_RISCV_E        : boolean := false;
      CPU_EXTENSION_RISCV_M        : boolean := false;
      CPU_EXTENSION_RISCV_U        : boolean := false;
      CPU_EXTENSION_RISCV_Zfinx    : boolean := false;
      CPU_EXTENSION_RISCV_Zicntr   : boolean := true;
      CPU_EXTENSION_RISCV_Zihpm    : boolean := false;
      CPU_EXTENSION_RISCV_Zifencei : boolean := false;
      CPU_EXTENSION_RISCV_Zmmul    : boolean := false;
      CPU_EXTENSION_RISCV_Zxcfu    : boolean := false;
      -- Tuning Options --
      FAST_MUL_EN                  : boolean := false;
      FAST_SHIFT_EN                : boolean := false;
      -- Physical Memory Protection (PMP) --
      PMP_NUM_REGIONS              : natural range 0 to 16 := 0;
      PMP_MIN_GRANULARITY          : natural := 4;
      -- Hardware Performance Monitors (HPM) --
      HPM_NUM_CNTS                 : natural range 0 to 13 := 0;
      HPM_CNT_WIDTH                : natural range 0 to 64 := 40;
      -- Atomic Memory Access - Reservation Set Granularity --
      AMO_RVS_GRANULARITY          : natural := 4;
      -- Internal Instruction memory (IMEM) --
      MEM_INT_IMEM_EN              : boolean := false;
      MEM_INT_IMEM_SIZE            : natural := 16*1024;
      MEM_INT_IMEM_PREFETCH        : boolean := false;
      -- Internal Data memory (DMEM) --
      MEM_INT_DMEM_EN              : boolean := false;
      MEM_INT_DMEM_SIZE            : natural := 8*1024;
      -- Internal Instruction Cache (iCACHE) --
      ICACHE_EN                    : boolean                  := false;
      ICACHE_NUM_BLOCKS            : natural range 1 to 256   := 4;
      ICACHE_BLOCK_SIZE            : natural range 4 to 2**16 := 64;
      ICACHE_ASSOCIATIVITY         : natural range 1 to 2     := 1;
      -- Internal Data Cache (dCACHE) --
      DCACHE_EN                    : boolean                  := false;
      DCACHE_NUM_BLOCKS            : natural range 1 to 256   := 4;
      DCACHE_BLOCK_SIZE            : natural range 4 to 2**16 := 64;
      -- External memory interface (WISHBONE) --
      MEM_EXT_EN                   : boolean := false;
      MEM_EXT_TIMEOUT              : natural := 255;
      MEM_EXT_PIPE_MODE            : boolean := false;
      MEM_EXT_BIG_ENDIAN           : boolean := false;
      MEM_EXT_ASYNC_RX             : boolean := false;
      MEM_EXT_ASYNC_TX             : boolean := false;
      -- External Interrupts Controller (XIRQ) --
      XIRQ_NUM_CH                  : natural range 0 to 32          := 0;
      XIRQ_TRIGGER_TYPE            : std_ulogic_vector(31 downto 0) := x"ffffffff";
      XIRQ_TRIGGER_POLARITY        : std_ulogic_vector(31 downto 0) := x"ffffffff";
      -- Processor peripherals --
      IO_GPIO_NUM                  : natural range 0 to 64          := 0;
      IO_MTIME_EN                  : boolean                        := false;
      IO_UART0_EN                  : boolean                        := false;
      IO_UART0_RX_FIFO             : natural range 1 to 2**15       := 1;
      IO_UART0_TX_FIFO             : natural range 1 to 2**15       := 1;
      IO_UART1_EN                  : boolean                        := false;
      IO_UART1_RX_FIFO             : natural range 1 to 2**15       := 1;
      IO_UART1_TX_FIFO             : natural range 1 to 2**15       := 1;
      IO_SPI_EN                    : boolean                        := false;
      IO_SPI_FIFO                  : natural range 1 to 2**15       := 1;
      IO_SDI_EN                    : boolean                        := false;
      IO_SDI_FIFO                  : natural range 1 to 2**15       := 1;
      IO_TWI_EN                    : boolean                        := false;
      IO_PWM_NUM_CH                : natural range 0 to 12          := 0;
      IO_WDT_EN                    : boolean                        := false;
      IO_TRNG_EN                   : boolean                        := false;
      IO_TRNG_FIFO                 : natural range 1 to 2**15       := 1;
      IO_CFS_EN                    : boolean                        := false;
      IO_CFS_CONFIG                : std_ulogic_vector(31 downto 0) := x"00000000";
      IO_CFS_IN_SIZE               : natural                        := 32;
      IO_CFS_OUT_SIZE              : natural                        := 32;
      IO_NEOLED_EN                 : boolean                        := false;
      IO_NEOLED_TX_FIFO            : natural range 1 to 2**15       := 1;
      IO_GPTMR_EN                  : boolean                        := false;
      IO_XIP_EN                    : boolean                        := false;
      IO_ONEWIRE_EN                : boolean                        := false;
      IO_DMA_EN                    : boolean                        := false;
      IO_SLINK_EN                  : boolean                        := false;
      IO_SLINK_RX_FIFO             : natural range 1 to 2**15       := 1;
      IO_SLINK_TX_FIFO             : natural range 1 to 2**15       := 1;
      IO_CRC_EN                    : boolean                        := false
    );
    port (
      -- Global control --
      clk_i          : in  std_ulogic;
      rstn_i         : in  std_ulogic;
      -- JTAG on-chip debugger interface --
      jtag_trst_i    : in  std_ulogic := 'U';
      jtag_tck_i     : in  std_ulogic := 'U';
      jtag_tdi_i     : in  std_ulogic := 'U';
      jtag_tdo_o     : out std_ulogic;
      jtag_tms_i     : in  std_ulogic := 'U';
      -- Wishbone bus interface (available if MEM_EXT_EN = true) --
      wb_tag_o       : out std_ulogic_vector(02 downto 0);
      wb_adr_o       : out std_ulogic_vector(31 downto 0);
      wb_dat_i       : in  std_ulogic_vector(31 downto 0) := (others => 'U');
      wb_dat_o       : out std_ulogic_vector(31 downto 0);
      wb_we_o        : out std_ulogic;
      wb_sel_o       : out std_ulogic_vector(03 downto 0);
      wb_stb_o       : out std_ulogic;
      wb_cyc_o       : out std_ulogic;
      wb_ack_i       : in  std_ulogic := 'L';
      wb_err_i       : in  std_ulogic := 'L';
      -- Stream Link Interface (available if IO_SLINK_EN = true) --
      slink_rx_dat_i : in  std_ulogic_vector(31 downto 0) := (others => 'U');
      slink_rx_val_i : in  std_ulogic := 'L';
      slink_rx_rdy_o : out std_ulogic;
      slink_tx_dat_o : out std_ulogic_vector(31 downto 0);
      slink_tx_val_o : out std_ulogic;
      slink_tx_rdy_i : in  std_ulogic := 'L';
      -- Advanced memory control signals --
      fence_o        : out std_ulogic;
      fencei_o       : out std_ulogic;
      -- XIP (execute in place via SPI) signals (available if IO_XIP_EN = true) --
      xip_csn_o      : out std_ulogic;
      xip_clk_o      : out std_ulogic;
      xip_dat_i      : in  std_ulogic := 'L';
      xip_dat_o      : out std_ulogic;
      -- GPIO (available if IO_GPIO_NUM > 0) --
      gpio_o         : out std_ulogic_vector(63 downto 0);
      gpio_i         : in  std_ulogic_vector(63 downto 0) := (others => 'U');
      -- primary UART0 (available if IO_UART0_EN = true) --
      uart0_txd_o    : out std_ulogic;
      uart0_rxd_i    : in  std_ulogic := 'U';
      uart0_rts_o    : out std_ulogic;
      uart0_cts_i    : in  std_ulogic := 'L';
      -- secondary UART1 (available if IO_UART1_EN = true) --
      uart1_txd_o    : out std_ulogic;
      uart1_rxd_i    : in  std_ulogic := 'U'; -- UART1 receive data
      uart1_rts_o    : out std_ulogic;
      uart1_cts_i    : in  std_ulogic := 'L';
      -- SPI (available if IO_SPI_EN = true) --
      spi_clk_o      : out std_ulogic;
      spi_dat_o      : out std_ulogic;
      spi_dat_i      : in  std_ulogic := 'U';
      spi_csn_o      : out std_ulogic_vector(07 downto 0); -- SPI CS
      -- SDI (available if IO_SDI_EN = true) --
      sdi_clk_i      : in  std_ulogic := 'U';
      sdi_dat_o      : out std_ulogic;
      sdi_dat_i      : in  std_ulogic := 'U';
      sdi_csn_i      : in  std_ulogic := 'H';
      -- TWI (available if IO_TWI_EN = true) --
      twi_sda_i      : in  std_ulogic := 'H';
      twi_sda_o      : out std_ulogic;
      twi_scl_i      : in  std_ulogic := 'H';
      twi_scl_o      : out std_ulogic;
      -- 1-Wire Interface (available if IO_ONEWIRE_EN = true) --
      onewire_i      : in  std_ulogic := 'H';
      onewire_o      : out std_ulogic;
      -- PWM (available if IO_PWM_NUM_CH > 0) --
      pwm_o          : out std_ulogic_vector(11 downto 0); -- pwm channels
      -- Custom Functions Subsystem IO --
      cfs_in_i       : in  std_ulogic_vector(IO_CFS_IN_SIZE-1 downto 0) := (others => 'U');
      cfs_out_o      : out std_ulogic_vector(IO_CFS_OUT_SIZE-1 downto 0);
      -- NeoPixel-compatible smart LED interface (available if IO_NEOLED_EN = true) --
      neoled_o       : out std_ulogic;
      -- External platform interrupts (available if XIRQ_NUM_CH > 0) --
      xirq_i         : in  std_ulogic_vector(31 downto 0) := (others => 'L');
      -- CPU Interrupts --
      mtime_irq_i    : in  std_ulogic := 'L';
      msw_irq_i      : in  std_ulogic := 'L';
      mext_irq_i     : in  std_ulogic := 'L'
    );
  end component;

end neorv32_package;

package body neorv32_package is

-- ****************************************************************************************************************************
-- Functions
-- ****************************************************************************************************************************

  -- Minimal required number of bits to represent <input> numbers ---------------------------
  -- -------------------------------------------------------------------------------------------
  function index_size_f(input : natural) return natural is
  begin
    for i in 0 to natural'high loop
      if (2**i >= input) then
        return i;
      end if;
    end loop;
    return 0;
  end function index_size_f;

  -- Conditional select integer -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_int_f(cond : boolean; val_t : integer; val_f : integer) return integer is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_int_f;

  -- Conditional select natural -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_natural_f(cond : boolean; val_t : natural; val_f : natural) return natural is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_natural_f;

  -- Conditional select std_ulogic_vector ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_suv_f(cond : boolean; val_t : std_ulogic_vector; val_f : std_ulogic_vector) return std_ulogic_vector is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_suv_f;

  -- Conditional select string --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function cond_sel_string_f(cond : boolean; val_t : string; val_f : string) return string is
  begin
    if (cond = true) then
      return val_t;
    else
      return val_f;
    end if;
  end function cond_sel_string_f;

  -- Convert boolean to std_ulogic ----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function bool_to_ulogic_f(cond : boolean) return std_ulogic is
  begin
    if (cond = true) then
      return '1';
    else
      return '0';
    end if;
  end function bool_to_ulogic_f;

  -- Convert binary to gray -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function bin_to_gray_f(input : std_ulogic_vector) return std_ulogic_vector is
    variable tmp_v : std_ulogic_vector(input'range);
  begin
    tmp_v(input'length-1) := input(input'length-1); -- keep MSB
    for i in input'length-2 downto 0 loop
      tmp_v(i) := input(i) xor input(i+1);
    end loop;
    return tmp_v;
  end function bin_to_gray_f;

  -- Convert gray to binary -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function gray_to_bin_f(input : std_ulogic_vector) return std_ulogic_vector is
    variable tmp_v : std_ulogic_vector(input'range);
  begin
    tmp_v(input'length-1) := input(input'length-1); -- keep MSB
    for i in input'length-2 downto 0 loop
      tmp_v(i) := tmp_v(i+1) xor input(i);
    end loop;
    return tmp_v;
  end function gray_to_bin_f;

  -- OR all bits ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function or_reduce_f(input : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := '0';
    for i in input'range loop
      tmp_v := tmp_v or input(i);
    end loop;
    return tmp_v;
  end function or_reduce_f;

  -- AND all bits ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function and_reduce_f(input : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := '1';
    for i in input'range loop
      tmp_v := tmp_v and input(i);
    end loop;
    return tmp_v;
  end function and_reduce_f;

  -- XOR all bits ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function xor_reduce_f(input : std_ulogic_vector) return std_ulogic is
    variable tmp_v : std_ulogic;
  begin
    tmp_v := '0';
    for i in input'range loop
      tmp_v := tmp_v xor input(i);
    end loop;
    return tmp_v;
  end function xor_reduce_f;

  -- Check if std_ulogic is not '1' or '0' --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function su_undefined_f(input : std_ulogic) return boolean is
  begin
    case input is
      when '1' | '0' => return false;
      when others    => return true;
    end case;
  end function su_undefined_f;

  -- Convert std_ulogic_vector to lowercase HEX char ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  function to_hexchar_f(input : std_ulogic_vector(3 downto 0)) return character is
    variable hex_v : string(1 to 16);
  begin
    hex_v := "0123456789abcdef";
    if (su_undefined_f(input(3)) = true) or (su_undefined_f(input(2)) = true) or
       (su_undefined_f(input(1)) = true) or (su_undefined_f(input(0)) = true) then
      return '?';
    else
      return hex_v(to_integer(unsigned(input)) + 1);
    end if;
  end function to_hexchar_f;

  -- Convert 32-bit std_ulogic_vector to hex string -----------------------------------------
  -- -------------------------------------------------------------------------------------------
  function to_hstring32_f(input : std_ulogic_vector(31 downto 0)) return string is
    variable res_v : string(1 to 8);
  begin
    for i in 7 downto 0 loop
      res_v(8-i) := to_hexchar_f(input(i*4+3 downto i*4+0));
    end loop;
    return res_v;
  end function to_hstring32_f;

  -- Bit reversal ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function bit_rev_f(input : std_ulogic_vector) return std_ulogic_vector is
    variable output_v : std_ulogic_vector(input'range);
  begin
    for i in 0 to input'length-1 loop
      output_v(input'length-i-1) := input(i);
    end loop;
    return output_v;
  end function bit_rev_f;

  -- Test if input number is a power of two -------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function is_power_of_two_f(input : natural) return boolean is
    variable tmp : unsigned(31 downto 0);
  begin
    if (input = 0) then
      return false;
    elsif (input = 1) then
      return true;
    else
      tmp := to_unsigned(input, 32);
      if ((tmp and (tmp - 1)) = 0) then
        return true;
      else
        return false;
      end if;
    end if;
  end function is_power_of_two_f;

  -- Swap all bytes of a 32-bit word (endianness conversion) --------------------------------
  -- -------------------------------------------------------------------------------------------
  function bswap32_f(input : std_ulogic_vector) return std_ulogic_vector is
    variable output_v : std_ulogic_vector(input'range);
  begin
    output_v(07 downto 00) := input(31 downto 24);
    output_v(15 downto 08) := input(23 downto 16);
    output_v(23 downto 16) := input(15 downto 08);
    output_v(31 downto 24) := input(07 downto 00);
    return output_v;
  end function bswap32_f;

  -- Population count (number of set bits) --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function popcount_f(input : std_ulogic_vector) return natural is
    variable cnt_v : natural range 0 to input'length;
  begin
    cnt_v := 0;
    for i in input'length-1 downto 0 loop
      if (input(i) = '1') then
        cnt_v := cnt_v + 1;
      end if;
    end loop;
    return cnt_v;
  end function popcount_f;

  -- Count leading zeros --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  function leading_zeros_f(input : std_ulogic_vector) return natural is
    variable cnt_v : natural range 0 to input'length;
  begin
    cnt_v := 0;
    for i in input'length-1 downto 0 loop
      if (input(i) = '0') then
        cnt_v := cnt_v + 1;
      else
        exit;
      end if;
    end loop;
    return cnt_v;
  end function leading_zeros_f;

  -- Initialize mem32_t array from another mem32_t array ------------------------------------
  -- -------------------------------------------------------------------------------------------
  impure function mem32_init_f(init : mem32_t; depth : natural) return mem32_t is
    variable mem_v : mem32_t(0 to depth-1);
  begin
    mem_v := (others => (others => '0')); -- [IMPORTANT] make sure remaining memory entries are set to zero
    if (init'length > depth) then
      return mem_v;
    end if;
    for i in 0 to init'length-1 loop -- initialize only in range of source data array
      mem_v(i) := init(i);
    end loop;
    return mem_v;
  end function mem32_init_f;


end neorv32_package;

-- ****************************************************************************************************************************
-- Additional Packages
-- ****************************************************************************************************************************

  -- Prototype Definition: bootloader_init_image --------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- > memory content in 'neorv32_bootloader_image.vhd', auto-generated by 'image_gen'
  -- > used by 'neorv32_boot_rom.vhd'
  -- > enables body-only recompile in case of firmware change (NEORV32 PR #338)

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

package neorv32_bootloader_image is
  constant bootloader_init_image : mem32_t;
end neorv32_bootloader_image;


  -- Prototype Definition: neorv32_application_image ----------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- > memory content in 'neorv32_application_image.vhd', auto-generated by 'image_gen'
  -- > used by 'mem/neorv32_imem.*.vhd'
  -- > enables body-only recompile in case of firmware change (NEORV32 PR #338)

library ieee;
use ieee.std_logic_1164.all;

library neorv32;
use neorv32.neorv32_package.all;

package neorv32_application_image is
  constant application_init_image : mem32_t := (
x"30005073",
x"30401073",
x"00000097",
x"13408093",
x"30509073",
x"80002117",
x"fe810113",
x"80000197",
x"7e418193",
x"00000213",
x"00000293",
x"00000313",
x"00000393",
x"00000413",
x"00000493",
x"00000813",
x"00000893",
x"00000913",
x"00000993",
x"00000a13",
x"00000a93",
x"00000b13",
x"00000b93",
x"00000c13",
x"00000c93",
x"00000d13",
x"00000d93",
x"00000e13",
x"00000e93",
x"00000f13",
x"00000f93",
x"00001597",
x"4ac58593",
x"80000617",
x"f7c60613",
x"80000697",
x"f7468693",
x"00c58e63",
x"00d65c63",
x"0005a703",
x"00e62023",
x"00458593",
x"00460613",
x"fedff06f",
x"80000717",
x"f5070713",
x"87418793",
x"00f75863",
x"00072023",
x"00470713",
x"ff5ff06f",
x"00001417",
x"fb840413",
x"00001497",
x"fb048493",
x"00945a63",
x"00042083",
x"000080e7",
x"00440413",
x"ff1ff06f",
x"00000513",
x"00000593",
x"090000ef",
x"30401073",
x"34051073",
x"00001417",
x"f8040413",
x"00001497",
x"f7848493",
x"00945a63",
x"00042083",
x"000080e7",
x"00440413",
x"ff1ff06f",
x"00000093",
x"00008463",
x"000080e7",
x"10500073",
x"ffdff06f",
x"ff810113",
x"00812023",
x"00912223",
x"34202473",
x"02044663",
x"34102473",
x"00041483",
x"0034f493",
x"00240413",
x"34141073",
x"00300413",
x"00941863",
x"34102473",
x"00240413",
x"34141073",
x"00012403",
x"00412483",
x"00810113",
x"30200073",
x"ff010113",
x"00112623",
x"00912223",
x"00812423",
x"135000ef",
x"fffff4b7",
x"000055b7",
x"50048513",
x"00000613",
x"b0058593",
x"2b9000ef",
x"50048513",
x"279000ef",
x"00051e63",
x"00100513",
x"00c12083",
x"00812403",
x"00412483",
x"01010113",
x"00008067",
x"fc0027f3",
x"2007f793",
x"fe0780e3",
x"26c000ef",
x"fc050ce3",
x"264000ef",
x"00050413",
x"298000ef",
x"fff00713",
x"32071073",
x"00000793",
x"b0379073",
x"b8379073",
x"b0479073",
x"b8479073",
x"b0579073",
x"b8579073",
x"b0679073",
x"b8679073",
x"b0779073",
x"b8779073",
x"b0879073",
x"b8879073",
x"b0979073",
x"b8979073",
x"b0a79073",
x"b8a79073",
x"b0b79073",
x"b8b79073",
x"b0c79073",
x"b8c79073",
x"b0d79073",
x"b8d79073",
x"b0e79073",
x"b8e79073",
x"00800693",
x"32369073",
x"01000693",
x"32469073",
x"02000693",
x"32569073",
x"04000693",
x"32669073",
x"08000693",
x"32769073",
x"10000693",
x"32869073",
x"20000693",
x"32969073",
x"40000693",
x"32a69073",
x"000016b7",
x"80068613",
x"32b61073",
x"32c69073",
x"000026b7",
x"32d69073",
x"000046b7",
x"32e69073",
x"32079073",
x"00000073",
x"f120d073",
x"32071073",
x"000015b7",
x"08458593",
x"50048513",
x"2a9000ef",
x"14040863",
x"b0302673",
x"000015b7",
x"09458593",
x"50048513",
x"291000ef",
x"00100793",
x"12f40a63",
x"b0402673",
x"000015b7",
x"0b858593",
x"50048513",
x"275000ef",
x"00200793",
x"10f40c63",
x"b0502673",
x"000015b7",
x"0dc58593",
x"50048513",
x"259000ef",
x"00300793",
x"0ef40e63",
x"b0602673",
x"000015b7",
x"10058593",
x"50048513",
x"23d000ef",
x"00400793",
x"0ef40063",
x"b0702673",
x"000015b7",
x"12458593",
x"50048513",
x"221000ef",
x"00500793",
x"0cf40263",
x"b0802673",
x"000015b7",
x"14858593",
x"50048513",
x"205000ef",
x"00600793",
x"0af40463",
x"b0902673",
x"000015b7",
x"fffff4b7",
x"16c58593",
x"50048513",
x"1e5000ef",
x"00700793",
x"08f40463",
x"b0a02673",
x"000015b7",
x"19058593",
x"50048513",
x"1c9000ef",
x"00800793",
x"06f40663",
x"b0b02673",
x"000015b7",
x"1b458593",
x"50048513",
x"1ad000ef",
x"00900793",
x"04f40863",
x"b0c02673",
x"000015b7",
x"1d858593",
x"50048513",
x"191000ef",
x"00a00793",
x"02f40a63",
x"b0d02673",
x"000015b7",
x"1fc58593",
x"50048513",
x"175000ef",
x"00b00793",
x"00f40c63",
x"b0e02673",
x"000015b7",
x"22058593",
x"50048513",
x"159000ef",
x"000015b7",
x"fffff537",
x"50050513",
x"24458593",
x"145000ef",
x"00000513",
x"d79ff06f",
x"fc002573",
x"20057513",
x"02050263",
x"32002773",
x"ff800793",
x"3207a073",
x"320027f3",
x"00000513",
x"0037d793",
x"00079663",
x"32071073",
x"00008067",
x"00150513",
x"0017d793",
x"fedff06f",
x"fc002573",
x"20057513",
x"02050663",
x"00800793",
x"3207a073",
x"fff00793",
x"b0379073",
x"b8379073",
x"b03027f3",
x"00000513",
x"00079863",
x"b83027f3",
x"00079a63",
x"00008067",
x"00150513",
x"0017d793",
x"fe9ff06f",
x"00150513",
x"0017d793",
x"fe5ff06f",
x"34011073",
x"f8010113",
x"00012023",
x"00112223",
x"340110f3",
x"00112423",
x"00312623",
x"00412823",
x"00512a23",
x"00612c23",
x"00712e23",
x"02812023",
x"02912223",
x"02a12423",
x"02b12623",
x"02c12823",
x"02d12a23",
x"02e12c23",
x"02f12e23",
x"05012023",
x"05112223",
x"05212423",
x"05312623",
x"05412823",
x"05512a23",
x"05612c23",
x"05712e23",
x"07812023",
x"07912223",
x"07a12423",
x"07b12623",
x"07c12823",
x"07d12a23",
x"07e12c23",
x"07f12e23",
x"342027f3",
x"00b00713",
x"02f76a63",
x"00001737",
x"00279793",
x"26470713",
x"00e787b3",
x"0007a783",
x"00078067",
x"00001737",
x"00279793",
x"29470713",
x"00e787b3",
x"0007a783",
x"00078067",
x"80000737",
x"ffd74713",
x"00e787b3",
x"01c00713",
x"fcf77ce3",
x"000017b7",
x"84478793",
x"00c0006f",
x"800007b7",
x"0007a783",
x"000780e7",
x"342027f3",
x"0207cc63",
x"00100713",
x"02e78863",
x"341026f3",
x"00468713",
x"301027f3",
x"0047f793",
x"00078c63",
x"34a027f3",
x"00300613",
x"0037f793",
x"00c78463",
x"00268713",
x"34171073",
x"00412083",
x"00c12183",
x"01012203",
x"01412283",
x"01812303",
x"01c12383",
x"02012403",
x"02412483",
x"02812503",
x"02c12583",
x"03012603",
x"03412683",
x"03812703",
x"03c12783",
x"04012803",
x"04412883",
x"04812903",
x"04c12983",
x"05012a03",
x"05412a83",
x"05812b03",
x"05c12b83",
x"06012c03",
x"06412c83",
x"06812d03",
x"06c12d83",
x"07012e03",
x"07412e83",
x"07812f03",
x"07c12f83",
x"00812103",
x"30200073",
x"800007b7",
x"0047a783",
x"f39ff06f",
x"800007b7",
x"0087a783",
x"f2dff06f",
x"800007b7",
x"00c7a783",
x"f21ff06f",
x"800007b7",
x"0107a783",
x"f15ff06f",
x"800007b7",
x"0147a783",
x"f09ff06f",
x"800007b7",
x"0187a783",
x"efdff06f",
x"800007b7",
x"01c7a783",
x"ef1ff06f",
x"800007b7",
x"0207a783",
x"ee5ff06f",
x"800007b7",
x"0247a783",
x"ed9ff06f",
x"800007b7",
x"0287a783",
x"ecdff06f",
x"800007b7",
x"02c7a783",
x"ec1ff06f",
x"800007b7",
x"0307a783",
x"eb5ff06f",
x"800007b7",
x"0347a783",
x"ea9ff06f",
x"800007b7",
x"0387a783",
x"e9dff06f",
x"800007b7",
x"03c7a783",
x"e91ff06f",
x"8401a783",
x"e89ff06f",
x"8441a783",
x"e81ff06f",
x"8481a783",
x"e79ff06f",
x"84c1a783",
x"e71ff06f",
x"8501a783",
x"e69ff06f",
x"8541a783",
x"e61ff06f",
x"8581a783",
x"e59ff06f",
x"85c1a783",
x"e51ff06f",
x"8601a783",
x"e49ff06f",
x"8641a783",
x"e41ff06f",
x"8681a783",
x"e39ff06f",
x"86c1a783",
x"e31ff06f",
x"8701a783",
x"e29ff06f",
x"fe010113",
x"00812c23",
x"fffff437",
x"01312623",
x"00050993",
x"50040513",
x"00112e23",
x"00912a23",
x"01212823",
x"01412423",
x"460000ef",
x"04050863",
x"50040513",
x"03000593",
x"524000ef",
x"50040513",
x"07800593",
x"00001937",
x"514000ef",
x"01c00493",
x"4f890913",
x"50040413",
x"ffc00a13",
x"0099d7b3",
x"00f7f793",
x"00f907b3",
x"0007c583",
x"00040513",
x"ffc48493",
x"4e8000ef",
x"ff4492e3",
x"01c12083",
x"01812403",
x"01412483",
x"01012903",
x"00c12983",
x"00812a03",
x"02010113",
x"00008067",
x"ff010113",
x"00812423",
x"fffff437",
x"50040513",
x"00112623",
x"00912223",
x"3d4000ef",
x"22050263",
x"000015b7",
x"30858593",
x"50040513",
x"4ac000ef",
x"300027f3",
x"00002737",
x"80070713",
x"00e7f7b3",
x"02078c63",
x"000015b7",
x"31058593",
x"50040513",
x"488000ef",
x"34202473",
x"00b00793",
x"0287e463",
x"00001737",
x"00241793",
x"4c870713",
x"00e787b3",
x"0007a783",
x"00078067",
x"000015b7",
x"31858593",
x"fcdff06f",
x"800007b7",
x"00b78713",
x"18e40463",
x"02876a63",
x"00378713",
x"16e40263",
x"00778793",
x"16f40463",
x"000015b7",
x"fffff537",
x"46858593",
x"50050513",
x"424000ef",
x"00040513",
x"ea9ff0ef",
x"05c0006f",
x"ff07c793",
x"00f407b3",
x"00f00713",
x"fcf76ae3",
x"000015b7",
x"fffff537",
x"45c58593",
x"50050513",
x"3f4000ef",
x"00f47493",
x"00048513",
x"e75ff0ef",
x"01048493",
x"00100793",
x"009797b3",
x"3447b073",
x"0180006f",
x"000015b7",
x"32058593",
x"fffff537",
x"50050513",
x"3c0000ef",
x"fffff4b7",
x"000015b7",
x"47c58593",
x"50048513",
x"3ac000ef",
x"34102573",
x"e31ff0ef",
x"000015b7",
x"48458593",
x"50048513",
x"394000ef",
x"34a02573",
x"e19ff0ef",
x"000015b7",
x"49058593",
x"50048513",
x"37c000ef",
x"34302573",
x"e01ff0ef",
x"00100793",
x"0a87ea63",
x"000015b7",
x"49c58593",
x"50048513",
x"35c000ef",
x"00000793",
x"30479073",
x"10500073",
x"ffdff06f",
x"000015b7",
x"34058593",
x"f79ff06f",
x"000015b7",
x"35c58593",
x"f6dff06f",
x"000015b7",
x"37058593",
x"f61ff06f",
x"000015b7",
x"37c58593",
x"f55ff06f",
x"000015b7",
x"39458593",
x"f49ff06f",
x"000015b7",
x"3a858593",
x"f3dff06f",
x"000015b7",
x"3c458593",
x"f31ff06f",
x"000015b7",
x"3d858593",
x"f25ff06f",
x"000015b7",
x"3f858593",
x"f19ff06f",
x"000015b7",
x"41858593",
x"f0dff06f",
x"000015b7",
x"43058593",
x"f01ff06f",
x"000015b7",
x"44458593",
x"ef5ff06f",
x"00812403",
x"00c12083",
x"50048513",
x"00412483",
x"000015b7",
x"4bc58593",
x"01010113",
x"29c0006f",
x"00c12083",
x"00812403",
x"00412483",
x"01010113",
x"00008067",
x"01c00793",
x"02a7e463",
x"800007b7",
x"00251513",
x"00078793",
x"00001737",
x"00a787b3",
x"84470713",
x"00e7a023",
x"00000513",
x"00008067",
x"fff00513",
x"00008067",
x"ff010113",
x"000027b7",
x"00112623",
x"00812423",
x"00912223",
x"80078793",
x"30079073",
x"4dc00793",
x"30579073",
x"00000793",
x"30479073",
x"34479073",
x"00000413",
x"01d00493",
x"00040513",
x"00140413",
x"f8dff0ef",
x"fe941ae3",
x"00c12083",
x"00812403",
x"00412483",
x"01010113",
x"00008067",
x"fd010113",
x"02812423",
x"02912223",
x"03212023",
x"01312e23",
x"01412c23",
x"02112623",
x"01512a23",
x"00001a37",
x"00050493",
x"00058413",
x"00058523",
x"00000993",
x"00410913",
x"51ca0a13",
x"00a00593",
x"00048513",
x"4b4000ef",
x"00aa0533",
x"00054783",
x"01390ab3",
x"00048513",
x"00fa8023",
x"00a00593",
x"450000ef",
x"00198993",
x"00a00793",
x"00050493",
x"fcf996e3",
x"00090693",
x"00900713",
x"03000613",
x"0096c583",
x"00070793",
x"fff70713",
x"01071713",
x"01075713",
x"00c59a63",
x"000684a3",
x"fff68693",
x"fe0710e3",
x"00000793",
x"00f907b3",
x"00000713",
x"0007c683",
x"00068c63",
x"00170613",
x"00e40733",
x"00d70023",
x"01061713",
x"01075713",
x"fff78693",
x"02f91863",
x"00e40433",
x"00040023",
x"02c12083",
x"02812403",
x"02412483",
x"02012903",
x"01c12983",
x"01812a03",
x"01412a83",
x"03010113",
x"00008067",
x"00068793",
x"fadff06f",
x"fffff7b7",
x"50078693",
x"00050713",
x"00d51a63",
x"e0802503",
x"01155513",
x"00157513",
x"00008067",
x"60078793",
x"00000513",
x"fef71ae3",
x"e0802503",
x"01955513",
x"fe5ff06f",
x"ff010113",
x"00812423",
x"00912223",
x"00112623",
x"00052023",
x"00050493",
x"e0002503",
x"00159593",
x"00060413",
x"34c000ef",
x"00000713",
x"3fe00693",
x"04a6e663",
x"fff50793",
x"000106b7",
x"fff68693",
x"00679793",
x"00d7f7b3",
x"07c006b7",
x"00d47433",
x"00371713",
x"0087e7b3",
x"01877713",
x"00c12083",
x"00812403",
x"00e7e7b3",
x"0017e793",
x"00f4a023",
x"00412483",
x"01010113",
x"00008067",
x"ffe70793",
x"ffd7f793",
x"00079863",
x"00355513",
x"00170713",
x"fa1ff06f",
x"00155513",
x"ff5ff06f",
x"00200737",
x"00052783",
x"00e7f7b3",
x"fe079ce3",
x"00b52223",
x"00008067",
x"fe010113",
x"00812c23",
x"00912a23",
x"01312623",
x"00112e23",
x"01212823",
x"00050493",
x"00058413",
x"00a00993",
x"00044903",
x"00140413",
x"02091063",
x"01c12083",
x"01812403",
x"01412483",
x"01012903",
x"00c12983",
x"02010113",
x"00008067",
x"01391863",
x"00d00593",
x"00048513",
x"f91ff0ef",
x"00090593",
x"00048513",
x"f85ff0ef",
x"fbdff06f",
x"fa010113",
x"04f12a23",
x"04810793",
x"02912a23",
x"03212823",
x"03312623",
x"03412423",
x"03512223",
x"03612023",
x"01712e23",
x"01812c23",
x"01912a23",
x"01a12823",
x"02112e23",
x"02812c23",
x"00050493",
x"00058913",
x"04c12423",
x"04d12623",
x"04e12823",
x"05012c23",
x"05112e23",
x"00f12023",
x"02500993",
x"00a00b93",
x"06900a13",
x"07500a93",
x"07800c13",
x"07000c93",
x"07300d13",
x"06300b13",
x"00094403",
x"02041e63",
x"03c12083",
x"03812403",
x"03412483",
x"03012903",
x"02c12983",
x"02812a03",
x"02412a83",
x"02012b03",
x"01c12b83",
x"01812c03",
x"01412c83",
x"01012d03",
x"06010113",
x"00008067",
x"17341463",
x"00194403",
x"03440e63",
x"068a6863",
x"13640263",
x"028b6463",
x"02500593",
x"13340463",
x"05800793",
x"08f40663",
x"02500593",
x"00048513",
x"e91ff0ef",
x"00040593",
x"10c0006f",
x"06400793",
x"fef414e3",
x"00012783",
x"0007a403",
x"00478713",
x"00e12023",
x"00045a63",
x"02d00593",
x"00048513",
x"40800433",
x"e5dff0ef",
x"00410593",
x"00040513",
x"c75ff0ef",
x"00410593",
x"0240006f",
x"0d540a63",
x"028ae663",
x"03940663",
x"fba410e3",
x"00012783",
x"0007a583",
x"00478713",
x"00e12023",
x"00048513",
x"e39ff0ef",
x"00290913",
x"f19ff06f",
x"f7841ee3",
x"00012783",
x"00410693",
x"00001637",
x"0007a803",
x"00478713",
x"00e12023",
x"00068593",
x"00000713",
x"50860613",
x"02000513",
x"00e857b3",
x"00f7f793",
x"00f607b3",
x"0007c783",
x"00470713",
x"fff68693",
x"00f68423",
x"fea712e3",
x"00010623",
x"05800793",
x"f8f41ee3",
x"00058793",
x"01900513",
x"00f10613",
x"0007c703",
x"f9f70693",
x"0ff6f693",
x"00d56663",
x"fe070713",
x"00e78023",
x"00178793",
x"fec792e3",
x"f6dff06f",
x"00012783",
x"0007c583",
x"00478713",
x"00e12023",
x"00048513",
x"d79ff0ef",
x"f59ff06f",
x"00012783",
x"00410593",
x"00478713",
x"0007a503",
x"00e12023",
x"f0dff06f",
x"01741863",
x"00d00593",
x"00048513",
x"d4dff0ef",
x"00040593",
x"00048513",
x"00190913",
x"d3dff0ef",
x"e39ff06f",
x"06054063",
x"0605c663",
x"00058613",
x"00050593",
x"fff00513",
x"02060c63",
x"00100693",
x"00b67a63",
x"00c05863",
x"00161613",
x"00169693",
x"feb66ae3",
x"00000513",
x"00c5e663",
x"40c585b3",
x"00d56533",
x"0016d693",
x"00165613",
x"fe0696e3",
x"00008067",
x"00008293",
x"fb5ff0ef",
x"00058513",
x"00028067",
x"40a00533",
x"00b04863",
x"40b005b3",
x"f9dff06f",
x"40b005b3",
x"00008293",
x"f91ff0ef",
x"40a00533",
x"00028067",
x"00008293",
x"0005ca63",
x"00054c63",
x"f79ff0ef",
x"00058513",
x"00028067",
x"40b005b3",
x"fe0558e3",
x"40a00533",
x"f61ff0ef",
x"40b00533",
x"00028067",
x"4d50480a",
x"73657220",
x"73746c75",
x"00000a3a",
x"304d5048",
x"6f6c2e33",
x"44282077",
x"204d454d",
x"20434345",
x"29646573",
x"20202020",
x"7525203d",
x"0000000a",
x"304d5048",
x"6f6c2e34",
x"44282077",
x"204d454d",
x"20434345",
x"29646564",
x"20202020",
x"7525203d",
x"0000000a",
x"304d5048",
x"6f6c2e35",
x"52282077",
x"69666765",
x"4520656c",
x"73204343",
x"20296465",
x"7525203d",
x"0000000a",
x"304d5048",
x"6f6c2e36",
x"52282077",
x"69666765",
x"4520656c",
x"64204343",
x"20296465",
x"7525203d",
x"0000000a",
x"304d5048",
x"6f6c2e37",
x"49282077",
x"6c662056",
x"20296761",
x"20202020",
x"20202020",
x"7525203d",
x"0000000a",
x"304d5048",
x"6f6c2e38",
x"4d282077",
x"73204d45",
x"65726f74",
x"20202973",
x"20202020",
x"7525203d",
x"0000000a",
x"304d5048",
x"6f6c2e39",
x"4d282077",
x"77204d45",
x"29746961",
x"20202020",
x"20202020",
x"7525203d",
x"0000000a",
x"314d5048",
x"6f6c2e30",
x"6a282077",
x"73706d75",
x"20202029",
x"20202020",
x"20202020",
x"7525203d",
x"0000000a",
x"314d5048",
x"6f6c2e31",
x"63282077",
x"2e646e6f",
x"61726220",
x"6568636e",
x"20202973",
x"7525203d",
x"0000000a",
x"314d5048",
x"6f6c2e32",
x"74282077",
x"6e656b61",
x"61726220",
x"6568636e",
x"20202973",
x"7525203d",
x"0000000a",
x"314d5048",
x"6f6c2e33",
x"45282077",
x"20734358",
x"5249202b",
x"20297351",
x"20202020",
x"7525203d",
x"0000000a",
x"314d5048",
x"6f6c2e34",
x"69282077",
x"67656c6c",
x"69206c61",
x"7274736e",
x"2020292e",
x"7525203d",
x"0000000a",
x"4d50480a",
x"6d656420",
x"7270206f",
x"6172676f",
x"6f63206d",
x"656c706d",
x"2e646574",
x"0000000a",
x"000005c4",
x"0000068c",
x"00000698",
x"000006a4",
x"000006b0",
x"000006bc",
x"000006c8",
x"000006d4",
x"000006e0",
x"000005b8",
x"000005b8",
x"000006ec",
x"000006f8",
x"000005b8",
x"000005b8",
x"000005b8",
x"00000704",
x"000005b8",
x"000005b8",
x"000005b8",
x"00000710",
x"000005b8",
x"000005b8",
x"000005b8",
x"000005b8",
x"0000071c",
x"00000728",
x"00000734",
x"00000740",
x"00000748",
x"00000750",
x"00000758",
x"00000760",
x"00000768",
x"00000770",
x"00000778",
x"00000780",
x"00000788",
x"00000790",
x"00000798",
x"000007a0",
x"4554523c",
x"0000203e",
x"205d4d5b",
x"00000000",
x"205d555b",
x"00000000",
x"74736e49",
x"74637572",
x"206e6f69",
x"72646461",
x"20737365",
x"6173696d",
x"6e67696c",
x"00006465",
x"74736e49",
x"74637572",
x"206e6f69",
x"65636361",
x"66207373",
x"746c7561",
x"00000000",
x"656c6c49",
x"206c6167",
x"74736e69",
x"74637572",
x"006e6f69",
x"61657242",
x"696f706b",
x"0000746e",
x"64616f4c",
x"64646120",
x"73736572",
x"73696d20",
x"67696c61",
x"0064656e",
x"64616f4c",
x"63636120",
x"20737365",
x"6c756166",
x"00000074",
x"726f7453",
x"64612065",
x"73657264",
x"696d2073",
x"696c6173",
x"64656e67",
x"00000000",
x"726f7453",
x"63612065",
x"73736563",
x"75616620",
x"0000746c",
x"69766e45",
x"6d6e6f72",
x"20746e65",
x"6c6c6163",
x"6f726620",
x"2d55206d",
x"65646f6d",
x"00000000",
x"69766e45",
x"6d6e6f72",
x"20746e65",
x"6c6c6163",
x"6f726620",
x"2d4d206d",
x"65646f6d",
x"00000000",
x"6863614d",
x"20656e69",
x"74666f73",
x"65726177",
x"51524920",
x"00000000",
x"6863614d",
x"20656e69",
x"656d6974",
x"52492072",
x"00000051",
x"6863614d",
x"20656e69",
x"65747865",
x"6c616e72",
x"51524920",
x"00000000",
x"74736146",
x"51524920",
x"00000020",
x"6e6b6e55",
x"206e776f",
x"70617274",
x"75616320",
x"00206573",
x"50204020",
x"00003d43",
x"544d202c",
x"54534e49",
x"0000003d",
x"544d202c",
x"3d4c4156",
x"00000000",
x"41465b20",
x"204c4154",
x"45435845",
x"4f495450",
x"205d214e",
x"746c6148",
x"20676e69",
x"2e555043",
x"522f3c20",
x"0a3e4554",
x"00000000",
x"0000094c",
x"000009d4",
x"000009e0",
x"000009ec",
x"000009f8",
x"00000a04",
x"00000a10",
x"00000a1c",
x"00000a28",
x"000008e8",
x"000008e8",
x"00000a34",
x"33323130",
x"37363534",
x"42413938",
x"46454443",
x"33323130",
x"37363534",
x"62613938",
x"66656463",
x"00000000",
x"33323130",
x"37363534",
x"00003938"
);
end neorv32_application_image;
