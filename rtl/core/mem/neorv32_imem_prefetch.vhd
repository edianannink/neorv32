library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_imem_prefetch is
  generic (
    IMEM_SIZE: natural;
    IMEM_AS_IROM: boolean
  );
  port (
    clk_i     : in std_ulogic;
    rstn_i    : in std_ulogic;
    bus_req_i : in bus_req_t;   -- bus request
    bus_rsp_o : out bus_rsp_t;  -- bus response
    bus_req_o : out bus_req_t;  -- prefetch request
    bus_rsp_i : in bus_rsp_t;   -- prefetch response
    fetched_o : out std_logic
  );
end entity;

architecture rtl of neorv32_imem_prefetch is
  signal mem_rom_b0 : mem8_t(0 to IMEM_SIZE/4-1);
  signal mem_rom_b1 : mem8_t(0 to IMEM_SIZE/4-1);
  signal mem_rom_b2 : mem8_t(0 to IMEM_SIZE/4-1);
  signal mem_rom_b3 : mem8_t(0 to IMEM_SIZE/4-1);
  signal mem_rom_b0_rd, mem_rom_b1_rd, mem_rom_b2_rd, mem_rom_b3_rd : std_ulogic_vector(7 downto 0);
	signal prefetch_addr : std_ulogic_vector(XLEN-1 downto 2);
  signal fetched : std_ulogic;
  signal resp : std_ulogic;
  signal rden  : std_ulogic;
	type prefetch_state is (start, request, wait_until_ready, all_fetched);
	signal fsm_state : prefetch_state;

begin
  fetch: process(clk_i, rstn_i) begin
    if rstn_i = '0' then
      fsm_state <= start;
      prefetch_addr <= (others => '0');
      fetched <= '0';
		elsif rising_edge(clk_i) then
      case fsm_state is
        when start =>
          fsm_state <= request;
        when request =>
          fsm_state <= wait_until_ready;
        when wait_until_ready =>
          if resp = '1' then
            mem_rom_b0(to_integer(unsigned(prefetch_addr))) <= bus_rsp_i.data(7 downto 0);
            mem_rom_b1(to_integer(unsigned(prefetch_addr))) <= bus_rsp_i.data(15 downto 8);
            mem_rom_b2(to_integer(unsigned(prefetch_addr))) <= bus_rsp_i.data(23 downto 16);
            mem_rom_b3(to_integer(unsigned(prefetch_addr))) <= bus_rsp_i.data(31 downto 24);
            if to_integer(unsigned(prefetch_addr) + 1) >= (IMEM_SIZE/4) then
              fsm_state <= all_fetched;
            else
              prefetch_addr <= std_ulogic_vector(unsigned(prefetch_addr) + 1);
              fsm_state <= request;
            end if;
          end if;
        when all_fetched =>
          fetched <= '1';
      end case;
      mem_rom_b0_rd <= mem_rom_b0(to_integer(unsigned(bus_req_i.addr(index_size_f(IMEM_SIZE/4)+1 downto 2))));
      mem_rom_b1_rd <= mem_rom_b1(to_integer(unsigned(bus_req_i.addr(index_size_f(IMEM_SIZE/4)+1 downto 2))));
      mem_rom_b2_rd <= mem_rom_b2(to_integer(unsigned(bus_req_i.addr(index_size_f(IMEM_SIZE/4)+1 downto 2))));
      mem_rom_b3_rd <= mem_rom_b3(to_integer(unsigned(bus_req_i.addr(index_size_f(IMEM_SIZE/4)+1 downto 2))));
		end if;
  end process;

  -- Bus Feedback ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_feedback: process(clk_i)
  begin
    if rising_edge(clk_i) then
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

  bus_rsp_o.data <= mem_rom_b3_rd & mem_rom_b2_rd & mem_rom_b1_rd & mem_rom_b0_rd when (rden = '1') else (others => '0');
	fetched_o  <= fetched;
end architecture;