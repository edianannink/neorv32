-- #################################################################################################
-- # << NEORV32 - Processor-internal data memory (DMEM) >>                                         #
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
use neorv32.neorv32_package.all;

architecture neorv32_dmem_rtl of neorv32_dmem is

  -- local signals --
  signal rdata : std_ulogic_vector(31 downto 0);
  signal rden  : std_ulogic;
  signal addr  : std_ulogic_vector(index_size_f(DMEM_SIZE/4)-1 downto 0);

  -- -------------------------------------------------------------------------------------------------------------- --
  -- The memory (RAM) is built from 4 individual byte-wide memories b0..b3, since some synthesis tools have         --
  -- problems with 32-bit memories that provide dedicated byte-enable signals AND/OR with multi-dimensional arrays. --
  -- [NOTE] Read-during-write behavior is irrelevant as read and write access are mutually exclusive.               --
  -- -------------------------------------------------------------------------------------------------------------- --

  -- RAM - not initialized at all --
  signal mem_ram_b0 : mem13_t(0 to DMEM_SIZE/4-1) := (others => (others => '0'));
  signal mem_ram_b1 : mem13_t(0 to DMEM_SIZE/4-1) := (others => (others => '0'));
  signal mem_ram_b2 : mem13_t(0 to DMEM_SIZE/4-1) := (others => (others => '0'));
  signal mem_ram_b3 : mem13_t(0 to DMEM_SIZE/4-1) := (others => (others => '0'));

  -- read data --
  signal mem_ram_b0_rd, mem_ram_b1_rd, mem_ram_b2_rd, mem_ram_b3_rd : std_ulogic_vector(12 downto 0);

  component prim_secded_13_8_enc
  port (
    data_i : in std_ulogic_vector(7 downto 0);
    data_o : out std_ulogic_vector(12 downto 0)
  );
  end component;

  component prim_secded_13_8_dec
  port (
    data_i : in std_ulogic_vector(12 downto 0);
    data_o : out std_ulogic_vector(7 downto 0);
    syndrome_o : out std_ulogic_vector(4 downto 0);
    err_o : out std_ulogic_vector(1 downto 0)
  );
  end component;

  signal ecc_enc_byte0_out, ecc_enc_byte1_out, ecc_enc_byte2_out, ecc_enc_byte3_out : std_ulogic_vector(12 downto 0);
  signal ecc_error_byte0, ecc_error_byte1, ecc_error_byte2, ecc_error_byte3 : std_ulogic_vector(1 downto 0);
  signal ecc_error: std_ulogic_vector(1 downto 0);

begin

  -- Sanity Checks --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  assert not (is_power_of_two_f(DMEM_SIZE) = false) report
    "NEORV32 PROCESSOR CONFIG ERROR: Internal DMEM size has to be a power of two!" severity error;

  assert false report
    "NEORV32 PROCESSOR CONFIG NOTE: Implementing DEFAULT processor-internal DMEM (RAM, " & natural'image(DMEM_SIZE) &
    " bytes)." severity note;


  -- Access Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  addr <= bus_req_i.addr(index_size_f(DMEM_SIZE/4)+1 downto 2); -- word aligned


  -- Memory Access --------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mem_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (bus_req_i.stb = '1') and (bus_req_i.rw = '1') then
        if (bus_req_i.ben(0) = '1') then -- byte 0
          mem_ram_b0(to_integer(unsigned(addr))) <= ecc_enc_byte0_out;
        end if;
        if (bus_req_i.ben(1) = '1') then -- byte 1
          mem_ram_b1(to_integer(unsigned(addr))) <= ecc_enc_byte1_out;
        end if;
        if (bus_req_i.ben(2) = '1') then -- byte 2
          mem_ram_b2(to_integer(unsigned(addr))) <= ecc_enc_byte2_out;
        end if;
        if (bus_req_i.ben(3) = '1') then -- byte 3
          mem_ram_b3(to_integer(unsigned(addr))) <= ecc_enc_byte3_out;
        end if;
      end if;
      mem_ram_b0_rd <= mem_ram_b0(to_integer(unsigned(addr)));
      mem_ram_b1_rd <= mem_ram_b1(to_integer(unsigned(addr)));
      mem_ram_b2_rd <= mem_ram_b2(to_integer(unsigned(addr)));
      mem_ram_b3_rd <= mem_ram_b3(to_integer(unsigned(addr)));
    end if;
  end process mem_access;


  -- Bus Feedback ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_feedback: process(clk_i)
  begin
    if rising_edge(clk_i) then
      rden          <= bus_req_i.stb and (not bus_req_i.rw);
      bus_rsp_o.ack <= bus_req_i.stb;
    end if;
  end process bus_feedback;

  -- output gate --
  bus_rsp_o.data <= rdata when (rden = '1') else (others => '0');

  -- no access error possible --
  bus_rsp_o.err <= '0';

  -- ECC --------------------------------------------------------------------------------------
  prim_secded_13_8_enc_inst_byte0 : prim_secded_13_8_enc
  port map (
    data_i => bus_req_i.data(7 downto 0),
    data_o => ecc_enc_byte0_out
  );

  prim_secded_13_8_enc_inst_byte1 : prim_secded_13_8_enc
  port map (
    data_i => bus_req_i.data(15 downto 8),
    data_o => ecc_enc_byte1_out
  );

  prim_secded_13_8_enc_inst_byte2 : prim_secded_13_8_enc
  port map (
    data_i => bus_req_i.data(23 downto 16),
    data_o => ecc_enc_byte2_out
  );

  prim_secded_13_8_enc_inst_byte3 : prim_secded_13_8_enc
  port map (
    data_i => bus_req_i.data(31 downto 24),
    data_o => ecc_enc_byte3_out
  );

  prim_secded_13_8_dec_inst_byte0: prim_secded_13_8_dec
    port map (
      data_i     => mem_ram_b0_rd,
      data_o     => rdata(7 downto 0),
      syndrome_o => open,
      err_o      => ecc_error_byte0
    );

  prim_secded_13_8_dec_inst_byte1: prim_secded_13_8_dec
    port map (
      data_i     => mem_ram_b1_rd,
      data_o     => rdata(15 downto 8),
      syndrome_o => open,
      err_o      => ecc_error_byte1
    );

  prim_secded_13_8_dec_inst_byte2: prim_secded_13_8_dec
    port map (
      data_i     => mem_ram_b2_rd,
      data_o     => rdata(23 downto 16),
      syndrome_o => open,
      err_o      => ecc_error_byte2
    );

  prim_secded_13_8_dec_inst_byte3: prim_secded_13_8_dec
    port map (
      data_i     => mem_ram_b3_rd,
      data_o     => rdata(31 downto 24),
      syndrome_o => open,
      err_o      => ecc_error_byte3
    );

  ecc_error(0) <= ecc_error_byte0(0) when (bus_req_i.ben(3) = '1') else '0' or 
                  ecc_error_byte0(0) when (bus_req_i.ben(2) = '1') else '0' or 
                  ecc_error_byte0(0) when (bus_req_i.ben(1) = '1') else '0' or 
                  ecc_error_byte0(0) when (bus_req_i.ben(0) = '1') else '0';
                      
  ecc_error(1) <= ecc_error_byte0(1) when (bus_req_i.ben(3) = '1') else '0' or 
                  ecc_error_byte1(1) when (bus_req_i.ben(2) = '1') else '0' or 
                  ecc_error_byte2(1) when (bus_req_i.ben(1) = '1') else '0' or 
                  ecc_error_byte3(1) when (bus_req_i.ben(0) = '1') else '0';

  ecc_error_o <= ecc_error when rden = '1' else (others => '0');

end neorv32_dmem_rtl;
