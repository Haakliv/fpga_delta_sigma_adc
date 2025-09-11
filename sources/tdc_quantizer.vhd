-- ************************************************************************
-- Time-to-Digital Converter (TDC) for Delta-Sigma ADC
-- Uses parasitic R/C for time measurement without external components
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Note: library work is implicit, no need to declare
use work.clk_rst_pkg.all;

entity tdc_quantizer is
  generic(
    TDC_BITS     : positive := 8;       -- TDC resolution bits
    COUNTER_BITS : positive := 16       -- Internal counter width
  );
  port(
    clk       : in  std_logic;
    reset     : in  rst_t;
    -- TDC input (from parasitic delay line)
    tdc_start : in  std_logic;          -- Start timing measurement
    tdc_stop  : in  std_logic;          -- Stop timing measurement

    -- Control
    enable    : in  std_logic;          -- Enable TDC operation
    trigger   : in  std_logic;          -- Trigger new measurement

    -- Output
    tdc_value : out std_logic_vector(TDC_BITS - 1 downto 0);
    tdc_valid : out std_logic;
    -- Status
    overflow  : out std_logic           -- Counter overflow indicator
  );
end entity;

architecture rtl of tdc_quantizer is
  -- FSM
  type   tdc_state_t is (IDLE, MEASURING, DONE);
  signal tdc_state   : tdc_state_t := IDLE;

  -- Internals
  signal counter      : unsigned(COUNTER_BITS - 1 downto 0) := (others => '0');
  signal start_sync   : std_logic_vector(2 downto 0)        := (others => '0');
  signal stop_sync    : std_logic_vector(2 downto 0)        := (others => '0');
  signal trigger_sync : std_logic_vector(2 downto 0)        := (others => '0');

  signal meas_reg     : unsigned(TDC_BITS - 1 downto 0) := (others => '0');
  signal valid_reg    : std_logic                       := '0';
  signal overflow_reg : std_logic                       := '0';

  -- Edge detection (3-FF sync; detect on middle vs oldest)
  signal start_edge   : std_logic;
  signal stop_edge    : std_logic;
  signal trigger_edge : std_logic;

  -- Helpers
  function any_high_above_tdc_bits(u : unsigned) return boolean is
    constant zeros : unsigned(u'high downto TDC_BITS) := (others => '0');
  begin
    if u'length <= TDC_BITS then
      return false;
    else
      return (u(u'high downto TDC_BITS) /= zeros);
    end if;
  end function;

begin
  -- Synchronizers
  sync_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        start_sync   <= (others => '0');
        stop_sync    <= (others => '0');
        trigger_sync <= (others => '0');
      else
        start_sync   <= start_sync(1 downto 0) & tdc_start;
        stop_sync    <= stop_sync(1 downto 0) & tdc_stop;
        trigger_sync <= trigger_sync(1 downto 0) & trigger;
      end if;
    end if;
  end process;

  start_edge   <= start_sync(1) and not start_sync(2);
  stop_edge    <= stop_sync(1) and not stop_sync(2);
  trigger_edge <= trigger_sync(1) and not trigger_sync(2);

  -- FSM + counter
  tdc_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        tdc_state    <= IDLE;
        counter      <= (others => '0');
        meas_reg     <= (others => '0');
        valid_reg    <= '0';
        overflow_reg <= '0';

      elsif enable = '1' then
        -- default
        valid_reg <= '0';

        case tdc_state is
          when IDLE =>
            counter      <= (others => '0');
            overflow_reg <= '0';
            if (trigger_edge = '1') or (start_edge = '1') then
              tdc_state <= MEASURING;
            end if;

          when MEASURING =>
            if stop_edge = '1' then
              -- Saturate into TDC_BITS; flag overflow if any high bits above TDC_BITS
              if any_high_above_tdc_bits(counter) then
                meas_reg     <= (others => '1'); -- max code on overflow
                overflow_reg <= '1';
              else
                meas_reg     <= resize(counter, TDC_BITS); -- exact fit
                overflow_reg <= '0';
              end if;
              valid_reg <= '1';         -- 1-cycle pulse
              tdc_state <= DONE;

            elsif counter = (COUNTER_BITS - 1 downto 0 => '1') then
              -- Counter overflow -> force stop
              meas_reg     <= (others => '1');
              overflow_reg <= '1';
              valid_reg    <= '1';
              tdc_state    <= DONE;

            else
              counter <= counter + 1;
            end if;

          when DONE =>
            -- Wait for next trigger/start to re-arm
            if (trigger_edge = '1') or (start_edge = '1') then
              counter   <= (others => '0');
              tdc_state <= MEASURING;
            end if;
        end case;

      else
        -- Disabled
        tdc_state <= IDLE;
        valid_reg <= '0';
      end if;
    end if;
  end process;

  -- Outputs
  tdc_value <= std_logic_vector(meas_reg);
  tdc_valid <= valid_reg;
  overflow  <= overflow_reg;

end architecture;
