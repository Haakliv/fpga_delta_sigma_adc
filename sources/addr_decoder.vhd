
-- ************************************************************************
-- Address Decoder for Delta-Sigma ADC Project
-- Routes memory requests between NIOS-V and multiple modules (ADC, About)
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity addr_decoder is
    generic(
        GC_BRIDGE_ADDR_W : natural;
        GC_MEM_DATA_W    : natural;
        GC_MEM_ADDR_W    : natural;
        GC_NUM_MODULES   : natural);
    port(
        clk               : in  std_logic;
        bridge_rd         : in  std_logic;
        bridge_wr         : in  std_logic;
        bridge_addr       : in  std_logic_vector(GC_BRIDGE_ADDR_W - 1 downto 0);
        bridge_wdata      : in  std_logic_vector(GC_MEM_DATA_W - 1 downto 0);
        bridge_rdata      : out std_logic_vector(GC_MEM_DATA_W - 1 downto 0);
        bridge_rdvalid    : out std_logic                                     := '0';
        mem_cs            : out std_logic_vector(GC_NUM_MODULES - 1 downto 0);
        mem_rd            : out std_logic;
        mem_wr            : out std_logic;
        mem_addr          : out std_logic_vector(GC_MEM_ADDR_W - 1 downto 0);
        mem_wdata         : out std_logic_vector(GC_MEM_DATA_W - 1 downto 0);
        mem_rdata_array_0 : in  std_logic_vector(GC_MEM_DATA_W - 1 downto 0); -- Module 0 (ADC)
        mem_rdata_array_1 : in  std_logic_vector(GC_MEM_DATA_W - 1 downto 0); -- Module 1 (About)
        mem_rdvalid       : in  std_logic_vector(GC_NUM_MODULES - 1 downto 0) := (others => '0'));

end entity;

architecture rtl of addr_decoder is

    signal bridge_rd_prev : std_logic                             := '0';
    signal r_index        : natural range 0 to GC_NUM_MODULES - 1 := 0;

begin

    --------------------------------------------------------------------------
    -- Main
    --------------------------------------------------------------------------
    interface_proc : process(clk)
        variable v_index_request : natural range 0 to 2 ** GC_BRIDGE_ADDR_W - 1  := 0;
        variable v_index_valid   : boolean                                       := false;
        variable v_index         : natural range 0 to GC_NUM_MODULES - 1         := 0;
        variable v_cs            : std_logic_vector(GC_NUM_MODULES - 1 downto 0) := (others => '0');
    begin
        if rising_edge(clk) then

            --------------------------------------------------------------------------
            -- Reset / Set variables
            --------------------------------------------------------------------------
            v_index_request := to_integer(unsigned(bridge_addr(GC_BRIDGE_ADDR_W - 1 downto GC_MEM_ADDR_W)));
            v_index         := 0;
            v_index_valid   := false;
            v_cs            := (others => '0');

            --------------------------------------------------------------------------
            -- Decode address to select module (ADC or About)
            --------------------------------------------------------------------------	
            -- Simple address decoding: bit 12 selects module
            -- Address 0x0000-0x0FFF: ADC (module 0)
            -- Address 0x1000-0x1FFF: About (module 1)
            if v_index_request < 16#1000# then
                v_index       := 0;     -- ADC module
                v_index_valid := true;
            elsif v_index_request >= 16#1000# and v_index_request < 16#2000# then
                v_index       := 1;     -- About module
                v_index_valid := true;
            end if;

            --------------------------------------------------------------------------
            -- If index is valid, set chip select bit.
            --------------------------------------------------------------------------
            for i in 0 to GC_NUM_MODULES - 1 loop
                if i = v_index and v_index_valid then
                    v_cs(i) := '1';
                else
                    v_cs(i) := '0';
                end if;
            end loop;

            --------------------------------------------------------------------------
            -- Read has rising edge gating.
            --------------------------------------------------------------------------
            bridge_rd_prev <= bridge_rd;

            if bridge_rd = '1' and bridge_rd_prev = '0' then
                mem_rd <= '1';
            else
                mem_rd <= '0';
            end if;

            -- mem_rd    <= mem_rd;
            mem_cs    <= v_cs;
            mem_wr    <= bridge_wr;
            mem_addr  <= bridge_addr(GC_MEM_ADDR_W - 1 downto 0);
            mem_wdata <= bridge_wdata;

            --------------------------------------------------------------------------
            -- Register index
            --------------------------------------------------------------------------
            if (bridge_rd or bridge_wr) = '1' then
                r_index <= v_index;
            end if;

            --------------------------------------------------------------------------
            -- CYCLE 
            -- Register rdata (mux based on module selection)
            -- Set read ack (fixed delay)
            --------------------------------------------------------------------------
            case r_index is
                when 0      => bridge_rdata <= mem_rdata_array_0; -- ADC module
                when 1      => bridge_rdata <= mem_rdata_array_1; -- About module
                when others => bridge_rdata <= (others => '0');
            end case;
            bridge_rdvalid <= mem_rdvalid(r_index);

        end if;
    end process;
end architecture;

