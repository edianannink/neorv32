library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_imem_prefetch is
  generic (
    IMEM_SIZE: natural;
    IMEM_AS_IROM: boolean;
    IMEM_ECC_BYPASS: boolean;
    IMEM_PREFETCH_BASE: std_ulogic_vector(31 downto 0)
  );
  port (
    clk_i       : in std_ulogic;
    rstn_i      : in std_ulogic;
    bus_req_i   : in bus_req_t;   -- bus request
    bus_rsp_o   : out bus_rsp_t;  -- bus response
    bus_req_o   : out bus_req_t;  -- prefetch request
    bus_rsp_i   : in bus_rsp_t;   -- prefetch response
    data_uc_o   : out std_ulogic_vector(31 downto 0);
    fetched_o   : out std_ulogic;
    ecc_error_o : out std_ulogic_vector(1 downto 0)  -- ECC error
  );
end entity;

architecture rtl of neorv32_imem_prefetch is
  signal b0_rom : mem15_t(0 to IMEM_SIZE/4-1);
  signal b1_rom : mem16_t(0 to IMEM_SIZE/4-1);
  signal b2_rom : mem16_t(0 to IMEM_SIZE/4-1);

	signal prefetch_addr  : std_ulogic_vector(XLEN-1 downto 2);
  signal fetched        : std_ulogic;
  signal resp           : std_ulogic;
  signal rden           : std_ulogic;

	type prefetch_state is (start, request, wait_until_ready, all_fetched);
	signal fsm_state : prefetch_state;

  component prim_secded_15_10_enc
  port (
    data_i : in std_ulogic_vector(9 downto 0);
    data_o : out std_ulogic_vector(14 downto 0)
  );
  end component;

  component prim_secded_15_10_dec
  generic (
    sec : integer
  );
  port (
    data_i : in std_ulogic_vector(14 downto 0);
    data_o : out std_ulogic_vector(9 downto 0);
    data_orig_o : out std_ulogic_vector(9 downto 0);
    syndrome_o : out std_ulogic_vector(4 downto 0);
    err_o : out std_ulogic_vector(1 downto 0)
  );
  end component;

  component prim_secded_16_11_enc
  port (
    data_i : in std_ulogic_vector(10 downto 0);
    data_o : out std_ulogic_vector(15 downto 0)
  );
  end component;

  component prim_secded_16_11_dec
  generic (
    sec : integer
  );
  port (
    data_i : in std_ulogic_vector(15 downto 0);
    data_o : out std_ulogic_vector(10 downto 0);
    data_orig_o : out std_ulogic_vector(10 downto 0);
    syndrome_o : out std_ulogic_vector(4 downto 0);
    err_o : out std_ulogic_vector(1 downto 0)
  );
  end component;

  signal b0_ecc_enc_o                                                     : std_ulogic_vector(14 downto 0); 
  signal b0_ecc_dec_i                                                     : std_ulogic_vector(14 downto 0);
  signal b0_ecc_dec_o, b0_ecc_dec_orig_o                                  : std_ulogic_vector(9 downto 0);

  signal b1_ecc_enc_o, b2_ecc_enc_o                                       : std_ulogic_vector(15 downto 0);
  signal b1_ecc_dec_i, b2_ecc_dec_i                                       : std_ulogic_vector(15 downto 0);
  signal b1_ecc_dec_o, b1_ecc_dec_orig_o, b2_ecc_dec_o, b2_ecc_dec_orig_o : std_ulogic_vector(10 downto 0);
  
  signal b0_ecc_err_o, b1_ecc_err_o, b2_ecc_err_o : std_ulogic_vector(1 downto 0);

begin
-- ECC --------------------------------------------------------------------------------------
  prim_secded_15_10_enc_inst_byte0 : prim_secded_15_10_enc
  port map (
    data_i => bus_rsp_i.data(9 downto 0),
    data_o => b0_ecc_enc_o
  );

  prim_secded_16_11_enc_inst_byte1 : prim_secded_16_11_enc
  port map (
    data_i => bus_rsp_i.data(20 downto 10),
    data_o => b1_ecc_enc_o
  );

  prim_secded_16_11_enc_inst_byte2 : prim_secded_16_11_enc
  port map (
    data_i => bus_rsp_i.data(31 downto 21),
    data_o => b2_ecc_enc_o
  );

  prim_secded_15_10_dec_inst_byte0: prim_secded_15_10_dec
    generic map (
      sec => 0
    )
    port map (
      data_i     => b0_ecc_dec_i,
      data_o     => b0_ecc_dec_o,
      data_orig_o=> b0_ecc_dec_orig_o,
      syndrome_o => open,
      err_o      => b0_ecc_err_o
    );

  prim_secded_16_11_dec_inst_byte1: prim_secded_16_11_dec
    generic map (
      sec => 0
    )
    port map (
      data_i     => b1_ecc_dec_i,
      data_o     => b1_ecc_dec_o,
      data_orig_o=> b1_ecc_dec_orig_o,
      syndrome_o => open,
      err_o      => b1_ecc_err_o
    );

  prim_secded_16_11_dec_inst_byte2: prim_secded_16_11_dec
    generic map (
      sec => 0
    )
    port map (
      data_i     => b2_ecc_dec_i,
      data_o     => b2_ecc_dec_o,
      data_orig_o=> b2_ecc_dec_orig_o,
      syndrome_o => open,
      err_o      => b2_ecc_err_o
    );

  fetch: process(clk_i, rstn_i) begin
    if rstn_i = '0' then
      fsm_state <= start;
      prefetch_addr <= IMEM_PREFETCH_BASE(XLEN-1 downto 2);
      fetched <= '0';
		elsif rising_edge(clk_i) then
      case fsm_state is
        when start =>
          fsm_state <= request;
        when request =>
          fsm_state <= wait_until_ready;
        when wait_until_ready =>
          if resp = '1' then
            b0_rom(to_integer(unsigned(prefetch_addr) - unsigned(IMEM_PREFETCH_BASE(XLEN-1 downto 2)))) <= b0_ecc_enc_o;
            b1_rom(to_integer(unsigned(prefetch_addr) - unsigned(IMEM_PREFETCH_BASE(XLEN-1 downto 2)))) <= b1_ecc_enc_o;
            b2_rom(to_integer(unsigned(prefetch_addr) - unsigned(IMEM_PREFETCH_BASE(XLEN-1 downto 2)))) <= b2_ecc_enc_o;
            if to_integer(unsigned(prefetch_addr) - unsigned(IMEM_PREFETCH_BASE(XLEN-1 downto 2)) + 1) >= (IMEM_SIZE/4) then
              fsm_state <= all_fetched;
            else
              prefetch_addr <= std_ulogic_vector(unsigned(prefetch_addr) + 1);
              fsm_state <= request;
            end if;
          end if;
        when all_fetched =>
          fetched <= '1';
          b0_ecc_dec_i <= b0_rom(to_integer(unsigned(bus_req_i.addr(index_size_f(IMEM_SIZE/4)+1 downto 2))));
          b1_ecc_dec_i <= b1_rom(to_integer(unsigned(bus_req_i.addr(index_size_f(IMEM_SIZE/4)+1 downto 2))));
          b2_ecc_dec_i <= b2_rom(to_integer(unsigned(bus_req_i.addr(index_size_f(IMEM_SIZE/4)+1 downto 2))));
      end case;
      
		end if;
  end process;

  -- Bus Feedback ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_feedback: process(clk_i)
  begin
    if (rstn_i = '0') then
      rden          <= '0';
      bus_rsp_o.ack <= '0';
    elsif rising_edge(clk_i) then
      rden <= bus_req_i.stb and (not bus_req_i.rw);
      if (IMEM_AS_IROM = true) then
        bus_rsp_o.ack <= bus_req_i.stb and (not bus_req_i.rw); -- read-only!
      else
        bus_rsp_o.ack <= bus_req_i.stb;
      end if;
    end if;
  end process bus_feedback;

  resp <= '1' when (bus_rsp_i.ack = '1') or (bus_rsp_i.err = '1') else '0';

  -- bus access type --
  bus_req_o.stb   <= '1' when fsm_state = request else '0';
  bus_req_o.addr  <= prefetch_addr & "00";
  bus_req_o.priv  <= '0';
  bus_req_o.data  <= (others => '0'); -- read-only
  bus_req_o.ben   <= (others => '0'); -- read-only
  bus_req_o.rw    <= '0'; -- read-only
  bus_req_o.src   <= '1'; -- source = instruction fetch
  bus_req_o.rvso  <= '0'; -- cannot be a reservation set operation
  bus_rsp_o.err  <= '0'; -- no access error possible

  bus_rsp_o.data <= b2_ecc_dec_o & b1_ecc_dec_o & b0_ecc_dec_o when (rden = '1') else (others => '0');
  data_uc_o <= b2_ecc_dec_orig_o & b1_ecc_dec_orig_o & b0_ecc_dec_orig_o when (rden = '1' and IMEM_ECC_BYPASS = true) else (others => '0');
	fetched_o <= fetched;

  ecc_error_o(0) <= (b0_ecc_err_o(0) or b1_ecc_err_o(0) or b2_ecc_err_o(0)) when (rden = '1') else '0';
  ecc_error_o(1) <= (b0_ecc_err_o(1) or b1_ecc_err_o(1) or b2_ecc_err_o(1)) when (rden = '1') else '0';

end architecture;