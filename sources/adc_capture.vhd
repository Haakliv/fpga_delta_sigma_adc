library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity adc_capture is
    generic(
        GC_DATA_WIDTH : positive := 16; -- Sample data width
        GC_DEPTH      : positive := 4096; -- Buffer depth (samples)
        GC_ADDR_WIDTH : positive := 12  -- Address width (log2(GC_DEPTH))
    );
    port(
        -- Clock and reset
        clk           : in  std_logic;
        rst           : in  std_logic;
        -- Sample input from ADC
        sample_data   : in  std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
        sample_valid  : in  std_logic;
        -- Control interface
        start_capture : in  std_logic;  -- Start capture (one-shot)
        start_dump    : in  std_logic;  -- Start UART dump (one-shot)

        -- Status outputs
        capturing     : out std_logic;  -- Capture in progress
        capture_done  : out std_logic;  -- Buffer full
        dumping       : out std_logic;  -- Dump in progress
        dump_done     : out std_logic;  -- Dump complete
        sample_count  : out std_logic_vector(GC_ADDR_WIDTH - 1 downto 0); -- Current sample count

        -- Output interface (directly to UART streamer)
        dump_data     : out std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
        dump_valid    : out std_logic;
        dump_ready    : in  std_logic   -- Flow control from UART
    );
end entity;

architecture rtl of adc_capture is

    -- ========================================================================
    -- Inferred Block RAM
    -- Quartus will automatically infer M10k/M20k from this array
    -- ========================================================================
    type   T_RAM is array (0 to GC_DEPTH - 1) of std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
    signal ram   : T_RAM;

    -- Synthesis attributes for Intel/Altera block RAM inference
    attribute ramstyle        : string;
    attribute ramstyle of ram : signal is "M20K"; -- Prefer M20K, falls back to M10K

    -- ========================================================================
    -- Address and control signals
    -- ========================================================================
    signal wr_addr     : unsigned(GC_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal rd_addr     : unsigned(GC_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal s_capturing : std_logic                            := '0';
    signal s_dumping   : std_logic                            := '0';
    signal s_full      : std_logic                            := '0';
    signal s_dump_done : std_logic                            := '0';

    -- Read pipeline for block RAM (1-cycle read latency)
    signal rd_data    : std_logic_vector(GC_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal rd_valid   : std_logic                                    := '0';
    signal rd_pending : std_logic                                    := '0'; -- Read request pending

begin

    -- Status outputs
    capturing    <= s_capturing;
    capture_done <= s_full;
    dumping      <= s_dumping;
    dump_done    <= s_dump_done;
    sample_count <= std_logic_vector(wr_addr);

    -- ========================================================================
    -- Write Side: Capture samples to RAM
    -- ========================================================================
    p_capture : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                s_capturing <= '0';
                wr_addr     <= (others => '0');
                s_full      <= '0';
            else
                -- Start capture on rising edge of start_capture
                if start_capture = '1' and s_capturing = '0' and s_dumping = '0' then
                    s_capturing <= '1';
                    wr_addr     <= (others => '0');
                    s_full      <= '0';
                end if;

                -- Capture samples while capturing
                if s_capturing = '1' then
                    if sample_valid = '1' then
                        -- Write sample to RAM
                        ram(to_integer(wr_addr)) <= sample_data;

                        -- Check if buffer full
                        if wr_addr = to_unsigned(GC_DEPTH - 1, GC_ADDR_WIDTH) then
                            s_capturing <= '0';
                            s_full      <= '1';
                        else
                            wr_addr <= wr_addr + 1;
                        end if;
                    end if;
                end if;
                
                -- Clear full flag when dump completes (s_dump_done driven by p_dump)
                if s_dump_done = '1' then
                    s_full <= '0';
                end if;
            end if;
        end if;
    end process;

    -- ========================================================================
    -- Read Side: Dump samples over UART
    -- Uses simple handshaking with dump_ready for flow control
    -- ========================================================================
    p_dump : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                s_dumping   <= '0';
                s_dump_done <= '0';
                rd_addr     <= (others => '0');
                rd_valid    <= '0';
                rd_pending  <= '0';
                rd_data     <= (others => '0');
            else
                -- Default: clear valid after accepted
                if rd_valid = '1' and dump_ready = '1' then
                    rd_valid <= '0';
                end if;
                
                -- Clear dump_done after one cycle (pulse)
                if s_dump_done = '1' then
                    s_dump_done <= '0';
                end if;

                -- Start dump on rising edge of start_dump (only when buffer full)
                if start_dump = '1' and s_dumping = '0' and s_full = '1' then
                    s_dumping   <= '1';
                    s_dump_done <= '0';
                    rd_addr     <= (others => '0');
                    rd_pending  <= '1'; -- Initiate first read
                end if;

                -- Dump state machine
                if s_dumping = '1' then
                    -- Pipeline: request read, then present data on next cycle
                    if rd_pending = '1' then
                        -- Read from RAM (1-cycle latency)
                        rd_data    <= ram(to_integer(rd_addr));
                        rd_pending <= '0';
                        rd_valid   <= '1';
                    elsif rd_valid = '0' or dump_ready = '1' then
                        -- Previous data accepted, advance to next
                        if rd_addr = to_unsigned(GC_DEPTH - 1, GC_ADDR_WIDTH) then
                            -- All samples sent
                            s_dumping   <= '0';
                            s_dump_done <= '1';  -- Pulse to signal capture process to clear s_full
                        else
                            rd_addr    <= rd_addr + 1;
                            rd_pending <= '1';
                        end if;
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- Output assignments
    dump_data  <= rd_data;
    dump_valid <= rd_valid and s_dumping;

end architecture rtl;
