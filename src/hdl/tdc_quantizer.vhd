-- ************************************************************************
-- Time-to-Digital Converter (TDC) for Delta-Sigma ADC
-- Uses parasitic R/C for time measurement without external components
-- ************************************************************************

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.clk_rst_pkg.all;

entity tdc_quantizer is
  generic (
    TDC_BITS     : positive := 8;   -- TDC resolution bits
    COUNTER_BITS : positive := 16   -- Internal counter width
  );
  port (
    clk           : in  std_logic;
    reset         : in  rst_t;
    
    -- TDC input (from parasitic delay line)
    tdc_start     : in  std_logic;  -- Start timing measurement
    tdc_stop      : in  std_logic;  -- Stop timing measurement
    
    -- Control
    enable        : in  std_logic;  -- Enable TDC operation
    trigger       : in  std_logic;  -- Trigger new measurement
    
    -- Output
    tdc_value     : out std_logic_vector(TDC_BITS-1 downto 0);
    tdc_valid     : out std_logic;
    
    -- Status
    overflow      : out std_logic   -- Counter overflow indicator
  );
end entity;

architecture rtl of tdc_quantizer is

  -- TDC state machine
  type tdc_state_t is (IDLE, MEASURING, DONE);
  signal tdc_state : tdc_state_t := IDLE;
  
  -- Internal signals
  signal counter        : unsigned(COUNTER_BITS-1 downto 0) := (others => '0');
  signal start_sync     : std_logic_vector(2 downto 0) := (others => '0');
  signal stop_sync      : std_logic_vector(2 downto 0) := (others => '0');
  signal trigger_sync   : std_logic_vector(2 downto 0) := (others => '0');
  signal measurement    : unsigned(TDC_BITS-1 downto 0) := (others => '0');
  signal valid_reg      : std_logic := '0';
  signal overflow_reg   : std_logic := '0';
  
  -- Edge detection
  signal start_edge     : std_logic;
  signal stop_edge      : std_logic;
  signal trigger_edge   : std_logic;

begin

  -- Synchronizers for input signals
  sync_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        start_sync <= (others => '0');
        stop_sync <= (others => '0');
        trigger_sync <= (others => '0');
      else
        start_sync <= start_sync(1 downto 0) & tdc_start;
        stop_sync <= stop_sync(1 downto 0) & tdc_stop;
        trigger_sync <= trigger_sync(1 downto 0) & trigger;
      end if;
    end if;
  end process;

  -- Edge detection
  start_edge <= start_sync(1) and not start_sync(2);
  stop_edge <= stop_sync(1) and not stop_sync(2);
  trigger_edge <= trigger_sync(1) and not trigger_sync(2);

  -- TDC state machine and counter
  tdc_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        tdc_state <= IDLE;
        counter <= (others => '0');
        measurement <= (others => '0');
        valid_reg <= '0';
        overflow_reg <= '0';
      elsif enable = '1' then
        case tdc_state is
          
          when IDLE =>
            valid_reg <= '0';
            overflow_reg <= '0';
            counter <= (others => '0');
            
            -- Start measurement on trigger or start edge
            if trigger_edge = '1' or start_edge = '1' then
              tdc_state <= MEASURING;
              counter <= (others => '0');
            end if;
          
          when MEASURING =>
            -- Count clock cycles until stop edge
            if stop_edge = '1' then
              -- Capture measurement (scale down to TDC_BITS)
              if counter < 2**TDC_BITS then
                measurement <= counter(TDC_BITS-1 downto 0);
                overflow_reg <= '0';
              else
                measurement <= (others => '1'); -- Max value on overflow
                overflow_reg <= '1';
              end if;
              
              tdc_state <= DONE;
              valid_reg <= '1';
              
            elsif counter = 2**COUNTER_BITS - 1 then
              -- Counter overflow - force stop
              measurement <= (others => '1');
              overflow_reg <= '1';
              tdc_state <= DONE;
              valid_reg <= '1';
              
            else
              -- Continue counting
              counter <= counter + 1;
            end if;
          
          when DONE =>
            -- Hold measurement until next trigger
            if trigger_edge = '1' then
              tdc_state <= IDLE;
            end if;
            
        end case;
      else
        -- TDC disabled
        tdc_state <= IDLE;
        valid_reg <= '0';
      end if;
    end if;
  end process;

  -- Output assignments
  tdc_value <= std_logic_vector(measurement);
  tdc_valid <= valid_reg;
  overflow <= overflow_reg;

end architecture rtl;
