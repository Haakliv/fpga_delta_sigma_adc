-- ************************************************************************
-- Top-Level Delta-Sigma ADC
-- Complete RC ADC with LVDS input and all processing stages
-- ************************************************************************

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.clk_rst_pkg.all;

entity rc_adc_top is
  generic (
    OSR              : positive := 64;     -- Oversampling ratio (configurable)
    DATA_WIDTH       : positive := 16;     -- Output data width
    ENABLE_MAJORITY  : boolean := true     -- Enable majority filter in LVDS
  );
  port (
    -- Clock and reset
    clk           : in  std_logic;
    reset         : in  rst_t;
    
    -- LVDS comparator input
    lvds_p        : in  std_logic;
    lvds_n        : in  std_logic;
    
    -- DAC output for feedback
    dac_out       : out std_logic;
    
    -- Processed data output
    stream_out    : out std_logic_vector(DATA_WIDTH-1 downto 0);
    stream_valid  : out std_logic;
    
    -- Status signals
    status        : out std_logic_vector(7 downto 0)
  );
end entity;

architecture rtl of rc_adc_top is

  -- Component declarations
  component lvds_comparator is
    generic (
      ENABLE_MAJORITY : boolean := true
    );
    port (
      clk           : in  std_logic;
      reset         : in  rst_t;
      lvds_p        : in  std_logic;
      lvds_n        : in  std_logic;
      bit_stream    : out std_logic
    );
  end component;

  component dac_1_bit is
    port (
      clk     : in  std_logic;
      reset   : in  rst_t;
      data_in : in  std_logic;
      dac_out : out std_logic
    );
  end component;

  component cic_sinc3_decimator is
    generic (
      DECIMATION   : positive := 64;
      OUTPUT_WIDTH : positive := 16
    );
    port (
      clk      : in  std_logic;
      reset    : in  rst_t;
      data_in  : in  std_logic;  -- 1-bit delta-sigma stream
      data_out : out std_logic_vector(OUTPUT_WIDTH-1 downto 0);
      valid    : out std_logic   -- high one cycle per output
    );
  end component;

  component fir_equalizer is
    generic (
      INPUT_WIDTH  : positive := 16;
      OUTPUT_WIDTH : positive := 16
    );
    port (
      clk       : in  std_logic;
      reset     : in  rst_t;
      data_in   : in  std_logic_vector(INPUT_WIDTH-1 downto 0);
      valid_in  : in  std_logic;
      data_out  : out std_logic_vector(OUTPUT_WIDTH-1 downto 0);
      valid_out : out std_logic
    );
  end component;

  component fir_lowpass is
    generic (
      INPUT_WIDTH  : positive := 16;
      OUTPUT_WIDTH : positive := 16
    );
    port (
      clk       : in  std_logic;
      reset     : in  rst_t;
      data_in   : in  std_logic_vector(INPUT_WIDTH-1 downto 0);
      valid_in  : in  std_logic;
      data_out  : out std_logic_vector(OUTPUT_WIDTH-1 downto 0);
      valid_out : out std_logic
    );
  end component;

  -- Internal signals
  signal lvds_bit_stream  : std_logic;  -- Output from LVDS comparator
  signal analog_in_sync   : std_logic_vector(1 downto 0) := (others => '0');
  signal dac_input        : std_logic;
  signal cic_data_out     : std_logic_vector(DATA_WIDTH-1 downto 0);
  signal cic_valid_out    : std_logic;
  signal eq_data_out      : std_logic_vector(DATA_WIDTH-1 downto 0);
  signal eq_valid_out     : std_logic;
  signal lp_data_out      : std_logic_vector(DATA_WIDTH-1 downto 0);
  signal lp_valid_out     : std_logic;
  
  -- Status monitoring
  signal activity_counter : unsigned(15 downto 0) := (others => '0');
  signal valid_counter    : unsigned(7 downto 0) := (others => '0');
  signal status_reg       : std_logic_vector(7 downto 0) := (others => '0');

begin

  -- LVDS input stage with comparator
  lvds_inst : lvds_comparator
    generic map (
      ENABLE_MAJORITY => ENABLE_MAJORITY
    )
    port map (
      clk        => clk,
      reset      => reset,
      lvds_p     => lvds_p,
      lvds_n     => lvds_n,
      bit_stream => lvds_bit_stream  -- Connect to synchronizer
    );

  -- 2-FF synchronizer for comparator bit (critical for sigma-delta feedback)
  -- This synchronizes the LVDS comparator output into the clk domain
  sync_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
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
  dac_inst : dac_1_bit
    port map (
      clk     => clk,
      reset   => reset,
      data_in => dac_input,
      dac_out => dac_out
    );

  -- CIC SINC3 decimator (uses synchronized bit)
  cic_inst : cic_sinc3_decimator
    generic map (
      DECIMATION   => OSR,
      OUTPUT_WIDTH => DATA_WIDTH
    )
    port map (
      clk      => clk,
      reset    => reset,
      data_in  => analog_in_sync(1),  -- 1-bit synchronized input
      data_out => cic_data_out,
      valid    => cic_valid_out
    );

  -- Decimation by 2 equalizer
  eq_inst : fir_equalizer
    generic map (
      INPUT_WIDTH  => DATA_WIDTH,
      OUTPUT_WIDTH => DATA_WIDTH
    )
    port map (
      clk       => clk,
      reset     => reset,
      data_in   => cic_data_out,
      valid_in  => cic_valid_out,
      data_out  => eq_data_out,
      valid_out => eq_valid_out
    );

  -- Final low-pass filter
  lp_inst : fir_lowpass
    generic map (
      INPUT_WIDTH  => DATA_WIDTH,
      OUTPUT_WIDTH => DATA_WIDTH
    )
    port map (
      clk       => clk,
      reset     => reset,
      data_in   => eq_data_out,
      valid_in  => eq_valid_out,
      data_out  => lp_data_out,
      valid_out => lp_valid_out
    );

  -- Output assignments
  stream_out   <= lp_data_out;
  stream_valid <= lp_valid_out;

  -- Status monitoring
  status_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        activity_counter <= (others => '0');
        valid_counter <= (others => '0');
        status_reg <= (others => '0');
      else
        -- Count activity
        activity_counter <= activity_counter + 1;
        
        -- Count valid outputs
        if lp_valid_out = '1' then
          valid_counter <= valid_counter + 1;
        end if;
        
        -- Build status register
        status_reg(0) <= analog_in_sync(1);          -- Live comparator bit
        status_reg(1) <= cic_valid_out;               -- CIC active
        status_reg(2) <= eq_valid_out;                -- Equalizer active
        status_reg(3) <= lp_valid_out;                -- Final output valid
        status_reg(4) <= activity_counter(15);        -- Activity heartbeat
        status_reg(7 downto 5) <= std_logic_vector(valid_counter(7 downto 5)); -- Output rate indicator
      end if;
    end if;
  end process;

  status <= status_reg;

end architecture rtl;
