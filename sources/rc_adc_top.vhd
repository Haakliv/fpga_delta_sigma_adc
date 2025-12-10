-- ************************************************************************
-- Top-Level Delta-Sigma ADC
-- Complete RC ADC with LVDS input and all processing stages
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library fpga_lib;
use fpga_lib.dsp_utils_pkg.all;

entity rc_adc_top is
  generic(
    GC_DECIMATION : positive := 64;     -- Oversampling ratio (configurable)
    GC_DATA_WIDTH : positive := 16      -- Output data width
  );
  port(
    -- Clock and reset
    clk            : in  std_logic;
    reset          : in  std_logic;
    -- Physical ADC interface
    -- True differential LVDS input (Quartus manages differential pair)
    analog_in      : in  std_logic;     -- LVDS differential comparator output
    dac_out        : out std_logic;     -- DAC output for feedback
    -- Optional trigger input (when '1', sampling is enabled; when '0', sampling is disabled)
    trigger_enable : in  std_logic := '1'; -- Default '1' for continuous sampling

    -- Streaming sample output
    sample_data    : out std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
    sample_valid   : out std_logic
  );
end entity;

architecture rtl of rc_adc_top is

  -- Internal signals
  signal lvds_bit_stream : std_logic;   -- Direct output from LVDS comparator (pass-through)

  -- Path B: Synchronized path for decimator (noise immunity)
  signal decimator_sync : std_logic_vector(1 downto 0) := (others => '0');

  -- DAC feedback register (1-bit DAC implementation)
  signal dac_feedback : std_logic := '0';

  signal cic_data_out  : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal cic_valid_out : std_logic;
  signal eq_data_out   : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal eq_valid_out  : std_logic;
  signal lp_data_out   : std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
  signal lp_valid_out  : std_logic;

begin

  -- ========================================================================
  -- LVDS Comparator Input
  -- ========================================================================
  -- The LVDS I/O buffer (configured in Quartus with LVDS_E_3V input standard)
  -- performs true differential comparison: analog_in = (P > N) ? '1' : '0'
  -- 
  -- This is a DIRECT PASS-THROUGH (combinational) to minimize deltasigma loop delay
  lvds_bit_stream <= analog_in;

  -- Path A: 1-bit DAC feedback implementation (single register)
  -- This is the ONLY register in the feedback loop
  -- Total loop: LVDS -> combinational -> DAC FF -> output (1 cycle)
  -- RC integrator topology (per davemuscle/sigma_delta_converters):
  --   LVDS+ = Analog input (signal)
  --   LVDS- = RC integrator (DAC feedback through R->C->GND)
  --   When comparator='1' (input > integrator): DAC='1' (charge RC, raise LVDS-)
  --   When comparator='0' (input < integrator): DAC='0' (discharge RC, lower LVDS-)
  --   Negative feedback emerges from RC integration, NOT from inversion
  p_dac_feedback : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        dac_feedback <= '0';
      else
        dac_feedback <= lvds_bit_stream; -- DIRECT connection (no inversion needed)
      end if;
    end if;
  end process;

  dac_out <= dac_feedback;

  -- Path B: 2-FF synchronizer for decimator (metastability protection)
  -- NO synchronous reset to minimize delay and maximize metastability protection
  p_decimator_sync : process(clk)
  begin
    if rising_edge(clk) then
      decimator_sync <= decimator_sync(0) & lvds_bit_stream;
    end if;
  end process;

  -- CIC SINC3 decimator (Path B - synchronized input)
  i_cic : entity work.cic_sinc3_decimator
    generic map(
      GC_DECIMATION   => GC_DECIMATION,
      GC_OUTPUT_WIDTH => GC_DATA_WIDTH
    )
    port map(
      clk      => clk,
      reset    => reset,
      data_in  => decimator_sync(1),    -- 2-FF synchronized input
      ce       => trigger_enable,       -- Gated by trigger
      data_out => cic_data_out,
      valid    => cic_valid_out
    );

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

  -- Streaming output interface (matches TDC ADC)
  sample_data  <= lp_data_out;
  sample_valid <= lp_valid_out;

end architecture rtl;
