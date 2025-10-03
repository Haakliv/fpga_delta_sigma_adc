-- ************************************************************************
-- Top-Level Delta-Sigma ADC
-- Complete RC ADC with LVDS input and all processing stages
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library fpga_lib;
-- Note: library work is implicit, no need to declare
use work.common_pkg.all;                -- For register map constants

entity rc_adc_top is
  generic(
    GC_DECIMATION : positive := 64;     -- Oversampling ratio (configurable)
    GC_DATA_WIDTH : positive := 16      -- Output data width
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
    -- True differential LVDS input (Quartus manages differential pair)
    analog_in    : in  std_logic;       -- LVDS differential comparator output
    dac_out      : out std_logic;       -- DAC output for feedback

    -- Streaming sample output
    sample_data  : out std_logic_vector(GC_DATA_WIDTH - 1 downto 0);
    sample_valid : out std_logic
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

  -- Status monitoring
  signal activity_counter : unsigned(15 downto 0) := (others => '0');
  signal valid_counter    : unsigned(7 downto 0)  := (others => '0');

  -- Status register is combinatorial to save flip-flops
  signal status_reg : std_logic_vector(7 downto 0);

  -- Altera synthesis attributes for proper synchronizer recognition
  attribute ALTERA_ATTRIBUTE                   : string;
  attribute ALTERA_ATTRIBUTE of decimator_sync : signal is "-name SYNCHRONIZER_IDENTIFICATION ""FORCED IF ASYNCHRONOUS""";

begin

  -- ========================================================================
  -- LVDS Comparator Input
  -- ========================================================================
  -- The LVDS I/O buffer (configured in Quartus with LVDS_E_3V input standard)
  -- performs true differential comparison: analog_in = (P > N) ? '1' : '0'
  -- 
  -- This is a DIRECT PASS-THROUGH (combinational) to minimize ΣΔ loop delay
  -- No register here - the only FF in the feedback loop is in p_dac_feedback process
  lvds_bit_stream <= analog_in;

  -- ========================================================================
  -- CRITICAL: Dual-path architecture for optimal sigma-delta performance
  -- ========================================================================
  -- Path A: Direct DAC feedback (0 FF comb, 1 FF in DAC) - Minimize loop delay
  -- Path B: Synchronized decimator input (2 FF) - Metastability protection
  -- Total ΣΔ loop delay = 1 cycle (DAC output register only)
  -- ========================================================================

  -- Path A: 1-bit DAC feedback implementation (single register)
  -- This is the ONLY register in the feedback loop
  -- Total loop: LVDS → combinational → DAC FF → output (1 cycle)
  -- RC integrator topology (per davemuscle/sigma_delta_converters):
  --   LVDS+ = Analog input (signal)
  --   LVDS- = RC integrator (DAC feedback through R→C→GND)
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
            when C_ADC_REG_DATA =>      -- ADC Data Register
              mem_rdata(lp_data_out'range) <= lp_data_out;
            when C_ADC_REG_STATUS =>    -- Status Register
              mem_rdata(status_reg'range) <= status_reg;
            when C_ADC_REG_VALID =>     -- Valid Counter
              mem_rdata(valid_counter'range) <= std_logic_vector(valid_counter);
            when C_ADC_REG_ACTIVITY =>  -- Activity Counter
              mem_rdata(activity_counter'range) <= std_logic_vector(activity_counter);
            when others =>
              mem_rdata <= (others => '0');
          end case;
          mem_rdvalid <= '1';
        end if;
      end if;
    end if;
  end process;

  -- Status monitoring - counters only
  p_status : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then
        activity_counter <= (others => '0');
        valid_counter    <= (others => '0');
      else
        -- Count activity
        activity_counter <= activity_counter + 1;

        -- Count valid outputs
        if lp_valid_out = '1' then
          valid_counter <= valid_counter + 1;
        end if;
      end if;
    end if;
  end process;

  -- Status register - combinatorial (saves 8 flip-flops)
  status_reg(0)          <= decimator_sync(1); -- Live comparator bit (synchronized)
  status_reg(1)          <= cic_valid_out; -- CIC active
  status_reg(2)          <= eq_valid_out; -- Equalizer active
  status_reg(3)          <= lp_valid_out; -- Final output valid
  status_reg(4)          <= activity_counter(15); -- Activity heartbeat
  status_reg(7 downto 5) <= std_logic_vector(valid_counter(7 downto 5)); -- Output rate indicator

end architecture rtl;
