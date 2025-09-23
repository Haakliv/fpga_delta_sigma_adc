-- ************************************************************************
-- Top-Level Delta-Sigma ADC
-- Complete RC ADC with LVDS input and all processing stages
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library fpga_lib;
-- Note: library work is implicit, no need to declare
use work.clk_rst_pkg.all;

entity rc_adc_top is
  generic(
    GC_DECIMATION      : positive := 64; -- Oversampling ratio (configurable)
    GC_DATA_WIDTH      : positive := 16; -- Output data width
    GC_ENABLE_MAJORITY : boolean  := true -- Enable majority filter in LVDS
  );
  port(
    -- Clock and reset
    clk          : in  std_logic;
    reset        : in  std_logic;
    -- Memory-mapped interface (Avalon-like)
    mem_cs       : in  std_logic;
    mem_rd       : in  std_logic;
    mem_wr       : in  std_logic;       -- @suppress: Unused (read-only interface)
    mem_addr     : in  std_logic_vector(11 downto 0);
    mem_wdata    : in  std_logic_vector(31 downto 0); -- @suppress: Unused (read-only interface)
    mem_rdata    : out std_logic_vector(31 downto 0);
    mem_rdvalid  : out std_logic;
    -- Physical ADC interface  
    analog_in_p  : in  std_logic;       -- LVDS comparator input positive
    analog_in_n  : in  std_logic;       -- LVDS comparator input negative
    dac_out      : out std_logic;       -- DAC output for feedback

    -- Streaming sample output
    sample_data  : out std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
    sample_valid : out std_logic
  );
end entity;

architecture rtl of rc_adc_top is

  -- Internal signals
  signal lvds_bit_stream : std_logic;   -- Output from LVDS comparator
  signal analog_in_sync  : std_logic_vector(1 downto 0) := (others => '0');
  signal dac_input       : std_logic;
  signal cic_data_out    : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal cic_valid_out   : std_logic;
  signal eq_data_out     : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal eq_valid_out    : std_logic;
  signal lp_data_out     : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal lp_valid_out    : std_logic;

  -- Status monitoring
  signal activity_counter : unsigned(15 downto 0)        := (others => '0');
  signal valid_counter    : unsigned(7 downto 0)         := (others => '0');
  signal status_reg       : std_logic_vector(7 downto 0) := (others => '0');

begin

  -- LVDS input stage with comparator
  i_lvds : entity work.lvds_comparator
    generic map(
      GC_ENABLE_MAJORITY => GC_ENABLE_MAJORITY,
      GC_USE_INTEL_LVDS  => true
    )
    port map(
      clk        => clk,
      reset      => reset,
      lvds_p     => analog_in_p,
      lvds_n     => analog_in_n,
      bit_stream => lvds_bit_stream     -- Connect to synchronizer
    );

  -- 2-FF synchronizer for comparator bit (critical for sigma-delta feedback)
  -- This synchronizes the LVDS comparator output into the clk domain
  p_sync : process(clk)
  begin
    if rising_edge(clk) then
      if reset = C_RST_ACTIVE then
        analog_in_sync <= (others => '0');
      else
        -- Synchronize LVDS comparator output into clk domain
        analog_in_sync <= analog_in_sync(0) & lvds_bit_stream;
      end if;
    end if;
  end process;

  -- Drive DAC at full rate with synchronized bit (CRITICAL!)
  dac_input <= analog_in_sync(1);

  -- DAC feedback at full sampling rate
  i_dac : entity work.dac_1_bit
    port map(
      clk     => clk,
      reset   => reset,
      data_in => dac_input,
      dac_out => dac_out
    );

  -- CIC SINC3 decimator (uses synchronized bit)
  i_cic : entity work.cic_sinc3_decimator
    generic map(
      GC_DECIMATION   => GC_DECIMATION,
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk      => clk,
      reset    => reset,
      data_in  => analog_in_sync(1),    -- 1-bit synchronized input
      data_out => cic_data_out,
      valid    => cic_valid_out
    );

  -- Decimation by 2 equalizer
  i_eq : entity work.fir_equalizer
    generic map(
      GC_INPUT_WIDTH  => GC_DATA_WIDTH,
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk       => clk,
      reset     => reset,
      data_in   => cic_data_out,
      valid_in  => cic_valid_out,
      data_out  => eq_data_out,
      valid_out => eq_valid_out
    );

  -- Final low-pass filter
  i_lp : entity work.fir_lowpass
    generic map(
      GC_INPUT_WIDTH  => GC_DATA_WIDTH,
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk       => clk,
      reset     => reset,
      data_in   => eq_data_out,
      valid_in  => eq_valid_out,
      data_out  => lp_data_out,
      valid_out => lp_valid_out
    );

  -- Expose decimated samples for external streaming logic
  sample_data  <= lp_data_out;
  sample_valid <= lp_valid_out;

  -- Memory-mapped register interface
  p_memory_interface : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        mem_rdata   <= (others => '0');
        mem_rdvalid <= '0';
      else
        mem_rdata   <= (others => '0');
        mem_rdvalid <= '0';

        if mem_cs = '1' and mem_rd = '1' then
          case to_integer(unsigned(mem_addr(7 downto 0))) is
            when 0 =>                   -- ADC Data Register (lower 16 bits)
              mem_rdata(GC_DATA_WIDTH - 1 downto 0) <= lp_data_out;
            when 1 =>                   -- Status Register
              mem_rdata(7 downto 0) <= status_reg;
            when 2 =>                   -- Valid Counter
              mem_rdata(7 downto 0) <= std_logic_vector(valid_counter);
            when 3 =>                   -- Activity Counter
              mem_rdata(15 downto 0) <= std_logic_vector(activity_counter);
            when others =>
              mem_rdata <= (others => '0');
          end case;
          mem_rdvalid <= '1';
        end if;
      end if;
    end if;
  end process;

  -- Status monitoring
  p_status : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        activity_counter <= (others => '0');
        valid_counter    <= (others => '0');
        status_reg       <= (others => '0');
      else
        -- Count activity
        activity_counter <= activity_counter + 1;

        -- Count valid outputs
        if lp_valid_out = '1' then
          valid_counter <= valid_counter + 1;
        end if;

        -- Build status register
        status_reg(0)          <= analog_in_sync(1); -- Live comparator bit
        status_reg(1)          <= cic_valid_out; -- CIC active
        status_reg(2)          <= eq_valid_out; -- Equalizer active
        status_reg(3)          <= lp_valid_out; -- Final output valid
        status_reg(4)          <= activity_counter(15); -- Activity heartbeat
        status_reg(7 downto 5) <= std_logic_vector(valid_counter(7 downto 5)); -- Output rate indicator
      end if;
    end if;
  end process;

end architecture rtl;
