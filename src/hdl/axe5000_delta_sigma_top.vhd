-- ************************************************************************
-- Simplified Delta-Sigma ADC Top Level for AXE5000
-- Minimal integration with NIOS-V using Avalon-MM interface
-- ************************************************************************

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.clk_rst_pkg.all;

entity axe5000_delta_sigma_simple is
  generic (
    ADC_DECIMATION : positive := 64    -- Configurable OSR/decimation factor
  );
  port (
    -- Clock and Reset
    CLK_25M_C     : in    std_logic;
    CPU_RESETn    : in    std_logic;
    
    -- UART
    UART_TX       : out   std_logic;
    UART_RX       : in    std_logic;
    
    -- Delta-Sigma ADC
    ANALOG_IN     : in    std_logic;    -- From comparator
    DAC_OUT       : out   std_logic;    -- To integrator/filter
    
    -- Debug
    TEST_PIN      : out   std_logic
  );
end entity;

architecture rtl of axe5000_delta_sigma_simple is

  signal clk_100m        : std_logic;
  signal reset_n         : std_logic;
  signal system_reset    : rst_t;
  
  -- Avalon interface signals
  signal adc_address     : std_logic_vector(0 downto 0);  -- Reduced to 1 bit
  signal adc_read        : std_logic;
  signal adc_readdata    : std_logic_vector(31 downto 0);
  signal adc_waitrequest : std_logic;
  signal adc_irq         : std_logic;

  -- NIOS-V system component (you'll need to create this in Qsys)
  component niosv_system is
    port (
      clk_25m_clk                : in  std_logic;
      reset_reset_n              : in  std_logic;
      pll_100m_outclk0           : out std_logic;
      
      -- ADC Avalon-MM interface (simplified)
      adc_avalon_address         : out std_logic_vector(0 downto 0);
      adc_avalon_read            : out std_logic;
      adc_avalon_readdata        : in  std_logic_vector(31 downto 0);
      adc_avalon_waitrequest     : in  std_logic;
      adc_avalon_irq             : in  std_logic;
      
      -- UART
      uart_rxd                   : in  std_logic;
      uart_txd                   : out std_logic
    );
  end component;

begin

  -- Reset management
  reset_n <= CPU_RESETn;
  system_reset <= not reset_n;
  
  -- Debug output
  TEST_PIN <= adc_irq;

  -- NIOS-V System
  niosv_inst : niosv_system
    port map (
      clk_25m_clk            => CLK_25M_C,
      reset_reset_n          => reset_n,
      pll_100m_outclk0       => clk_100m,
      
      adc_avalon_address     => adc_address,
      adc_avalon_read        => adc_read,
      adc_avalon_readdata    => adc_readdata,
      adc_avalon_waitrequest => adc_waitrequest,
      adc_avalon_irq         => adc_irq,
      
      uart_rxd               => UART_RX,
      uart_txd               => UART_TX
    );

  -- Delta-Sigma ADC with Avalon interface
  adc_avalon_inst : entity work.delta_sigma_adc_avalon
    generic map (
      OUTPUT_WIDTH => 16
    )
    port map (
      clk             => clk_100m,
      reset           => system_reset,
      
      decimation      => ADC_DECIMATION,    -- Configurable OSR
      
      analog_in       => ANALOG_IN,
      dac_out         => DAC_OUT,
      
      avs_address     => adc_address,
      avs_read        => adc_read,
      avs_readdata    => adc_readdata,
      avs_waitrequest => adc_waitrequest,
      
      adc_irq         => adc_irq
    );

end architecture rtl;
