-- ************************************************************************
-- Avalon Memory-Mapped Interface for Delta-Sigma ADC
-- Provides register-based access to ADC data for NIOS-V processor
-- ************************************************************************

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.clk_rst_pkg.all;

entity delta_sigma_adc_avalon is
  generic (
    DECIMATION   : positive := 64;     -- OSR/decimation factor (compile-time constant)
    OUTPUT_WIDTH : positive := 16
  );
  port (
    -- Clock and Reset
    clk           : in  std_logic;
    reset         : in  rst_t;
    
    -- Analog Interface
    analog_in     : in  std_logic;     -- 1-bit sigma-delta input
    dac_out       : out std_logic;     -- 1-bit DAC output
    
    -- Avalon Memory-Mapped Interface (read-only)
    avs_address   : in  std_logic_vector(0 downto 0);  -- Only 1 bit needed for 2 registers
    avs_read      : in  std_logic;
    avs_readdata  : out std_logic_vector(31 downto 0);
    avs_waitrequest : out std_logic;
    
    -- Interrupt
    adc_irq       : out std_logic      -- Interrupt when new data available
  );
end entity;

architecture rtl of delta_sigma_adc_avalon is

  -- ADC signals
  signal adc_data_out : std_logic_vector(OUTPUT_WIDTH-1 downto 0);
  signal adc_valid    : std_logic;
  signal dac_input    : std_logic;
  
  -- Register map (simplified - read-only)
  signal adc_data_reg    : std_logic_vector(15 downto 0);
  signal status_reg      : std_logic_vector(31 downto 0);
  
  -- Internal signals
  signal data_ready      : std_logic;

begin

  -- CIC SINC3 Decimator
  cic_inst : entity work.cic_sinc3_decimator
    generic map (
      DECIMATION   => DECIMATION,  -- Use generic parameter
      OUTPUT_WIDTH => OUTPUT_WIDTH
    )
    port map (
      clk      => clk,
      reset    => reset,
      data_in  => analog_in,       -- 1-bit input
      data_out => adc_data_out,
      valid    => adc_valid
    );

  -- 1-bit DAC
  dac_inst : entity work.dac_1_bit
    port map (
      clk     => clk,
      reset   => reset,
      data_in => dac_input,
      dac_out => dac_out
    );

  -- For now, use simple feedback (MSB of ADC output)
  dac_input <= adc_data_out(OUTPUT_WIDTH-1) when adc_valid = '1' else '0';

  -- Always-ready slave (combinational assignment)
  avs_waitrequest <= '0';

  -- Register interface (simplified - read-only)
  register_process : process(clk)
  begin
    if rising_edge(clk) then
      if reset = RST_ACTIVE then
        adc_data_reg <= (others => '0');
        status_reg <= (others => '0');
        data_ready <= '0';
      else
        -- Capture new ADC data
        if adc_valid = '1' then
          adc_data_reg <= adc_data_out;
          data_ready <= '1';
        end if;
        
        -- Update status register
        status_reg(0) <= data_ready;              -- Data ready flag
        status_reg(1) <= adc_valid;               -- ADC valid flag  
        status_reg(31 downto 16) <= adc_data_reg; -- Current ADC data
        
        -- Handle Avalon read transactions
        
        if avs_read = '1' then
          case avs_address is
            when "0" =>  -- Data Register (0x00)
              avs_readdata <= x"0000" & adc_data_reg;
              data_ready <= '0';  -- Clear data ready flag on read
              
            when "1" =>  -- Status Register (0x04)
              avs_readdata <= status_reg;
              
            when others =>
              avs_readdata <= (others => '0');
          end case;
        end if;
      end if;
    end if;
  end process;

  -- Interrupt output (always enabled now)
  adc_irq <= data_ready;

end architecture rtl;
