-- Simple Dual-Clock FIFO for CDC
-- Behavioral implementation using gray-code pointers
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dc_fifo is
    generic(
        GC_DATA_WIDTH : positive := 16;
        GC_FIFO_DEPTH : positive := 32  -- Must be power of 2
    );
    port(
        -- Write side
        wr_clk   : in  std_logic;
        wr_rst   : in  std_logic;
        wr_en    : in  std_logic;
        wr_data  : in  std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
        wr_full  : out std_logic;
        -- Read side
        rd_clk   : in  std_logic;
        rd_rst   : in  std_logic;
        rd_en    : in  std_logic;
        rd_data  : out std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
        rd_empty : out std_logic
    );
end entity;

architecture rtl of dc_fifo is

    constant C_ADDR_WIDTH : positive := 5; -- log2(32) = 5

    type   T_MEM is array (0 to GC_FIFO_DEPTH - 1) of std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
    signal mem   : T_MEM := (others => (others => '0'));

    -- Write domain signals
    signal wr_ptr_bin        : unsigned(C_ADDR_WIDTH downto 0) := (others => '0');
    signal wr_ptr_gray       : unsigned(C_ADDR_WIDTH downto 0) := (others => '0');
    signal wr_ptr_gray_sync1 : unsigned(C_ADDR_WIDTH downto 0) := (others => '0');
    signal wr_ptr_gray_sync2 : unsigned(C_ADDR_WIDTH downto 0) := (others => '0');

    -- Read domain signals
    signal rd_ptr_bin        : unsigned(C_ADDR_WIDTH downto 0) := (others => '0');
    signal rd_ptr_gray       : unsigned(C_ADDR_WIDTH downto 0) := (others => '0');
    signal rd_ptr_gray_sync1 : unsigned(C_ADDR_WIDTH downto 0) := (others => '0');
    signal rd_ptr_gray_sync2 : unsigned(C_ADDR_WIDTH downto 0) := (others => '0');

    -- Status
    signal wr_full_i  : std_logic := '0';
    signal rd_empty_i : std_logic := '1';

    -- Binary to Gray code conversion
    function bin_to_gray(bin : unsigned) return unsigned is
    begin
        return bin xor shift_right(bin, 1);
    end function;

    -- Gray to Binary conversion (not used but included for reference)
    function gray_to_bin(gray : unsigned) return unsigned is
        variable v_bin : unsigned(gray'range);
    begin
        v_bin := gray;
        for i in gray'high - 1 downto 0 loop
            v_bin(i) := v_bin(i) xor v_bin(i + 1);
        end loop;
        return v_bin;
    end function;

begin

    wr_full  <= wr_full_i;
    rd_empty <= rd_empty_i;

    -- ========================================================================
    -- Write side (wr_clk domain)
    -- ========================================================================
    p_write : process(wr_clk)
    begin
        if rising_edge(wr_clk) then
            if wr_rst = '1' then
                wr_ptr_bin  <= (others => '0');
                wr_ptr_gray <= (others => '0');
            else
                if wr_en = '1' and wr_full_i = '0' then
                    -- Write data to memory
                    mem(to_integer(wr_ptr_bin(C_ADDR_WIDTH - 1 downto 0))) <= wr_data;
                    -- Increment pointer
                    wr_ptr_bin                                             <= wr_ptr_bin + 1;
                    wr_ptr_gray                                            <= bin_to_gray(wr_ptr_bin + 1);
                end if;
            end if;
        end if;
    end process;

    -- Sync read pointer to write domain for full detection
    p_sync_rd_to_wr : process(wr_clk)
    begin
        if rising_edge(wr_clk) then
            rd_ptr_gray_sync1 <= rd_ptr_gray;
            rd_ptr_gray_sync2 <= rd_ptr_gray_sync1;
        end if;
    end process;

    -- Full flag: write pointer + 1 equals read pointer (in gray code)
    wr_full_i <= '1' when (bin_to_gray(wr_ptr_bin + 1) = rd_ptr_gray_sync2) else '0';

    -- ========================================================================
    -- Read side (rd_clk domain)
    -- ========================================================================
    p_read : process(rd_clk)
    begin
        if rising_edge(rd_clk) then
            if rd_rst = '1' then
                rd_ptr_bin  <= (others => '0');
                rd_ptr_gray <= (others => '0');
                rd_data     <= (others => '0');
            else
                if rd_en = '1' and rd_empty_i = '0' then
                    -- Read data from memory
                    rd_data     <= mem(to_integer(rd_ptr_bin(C_ADDR_WIDTH - 1 downto 0)));
                    -- Increment pointer
                    rd_ptr_bin  <= rd_ptr_bin + 1;
                    rd_ptr_gray <= bin_to_gray(rd_ptr_bin + 1);
                end if;
            end if;
        end if;
    end process;

    -- Sync write pointer to read domain for empty detection
    p_sync_wr_to_rd : process(rd_clk)
    begin
        if rising_edge(rd_clk) then
            wr_ptr_gray_sync1 <= wr_ptr_gray;
            wr_ptr_gray_sync2 <= wr_ptr_gray_sync1;
        end if;
    end process;

    -- Empty flag: read pointer equals write pointer (in gray code)
    rd_empty_i <= '1' when (rd_ptr_gray = wr_ptr_gray_sync2) else '0';

end architecture;
