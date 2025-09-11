
-- ************************************************************************
-- About Module for Delta-Sigma ADC Project
-- Provides version and build information via Avalon Memory-Mapped interface
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.about_pkg.all;

entity about is
    generic(
        GC_MEM_ADDR_W      : natural;
        GC_MEM_DATA_W      : natural;
        GC_IMAGE_TYPE      : natural := ADC_PROJECT_TYPE; -- Project type (ADC FPGA)
        GC_IMAGE_ID        : natural := ADC_PROJECT_ID; -- AXE5000 project ID
        GC_REV_MAJOR       : natural := 1; -- Major version
        GC_REV_MINOR       : natural := 0; -- Minor version
        GC_REV_PATCH       : natural := 0; -- Patch version
        GC_REV_BUILDNUMBER : natural := 1; -- Build number
        GC_GIT_HASH_MSB    : natural := 0; -- Git hash upper 32 bits
        GC_GIT_HASH_LSB    : natural := 0; -- Git hash lower 32 bits
        GC_GIT_DIRTY       : natural := 0; -- Git dirty flag
        GC_YYMMDD          : natural := 16#240101#; -- Build date
        GC_HHMMSS          : natural := 16#120000#); -- Build time
    port(
        clk         : in  std_logic;
        mem_cs      : in  std_logic;
        mem_rd      : in  std_logic;
        mem_addr    : in  std_logic_vector(GC_MEM_ADDR_W - 1 downto 0);
        mem_rdata   : out std_logic_vector(GC_MEM_DATA_W - 1 downto 0);
        mem_rdvalid : out std_logic := '0');
end entity;

architecture rtl of about is

begin
    p_mem : process(clk)
        variable v_rdata   : std_logic_vector(GC_MEM_DATA_W - 1 downto 0) := (others => '0');
        variable v_rdvalid : std_logic                                    := '0';
    begin
        if rising_edge(clk) then

            ---------------------------------------
            -- READ
            ---------------------------------------
            v_rdata   := (others => '0');
            v_rdvalid := '0';

            if (mem_cs and mem_rd) = '1' then
                case to_integer(unsigned(mem_addr)) is
                    when ABOUT_REGMAP_IMAGE_TYPE      => v_rdata := std_logic_vector(to_unsigned(GC_IMAGE_TYPE, GC_MEM_DATA_W));
                    when ABOUT_REGMAP_IMAGE_ID        => v_rdata := std_logic_vector(to_unsigned(GC_IMAGE_ID, GC_MEM_DATA_W));
                    when ABOUT_REGMAP_REV_MAJOR       => v_rdata := std_logic_vector(to_unsigned(GC_REV_MAJOR, GC_MEM_DATA_W));
                    when ABOUT_REGMAP_REV_MINOR       => v_rdata := std_logic_vector(to_unsigned(GC_REV_MINOR, GC_MEM_DATA_W));
                    when ABOUT_REGMAP_REV_PATCH       => v_rdata := std_logic_vector(to_unsigned(GC_REV_PATCH, GC_MEM_DATA_W));
                    when ABOUT_REGMAP_REV_BUILDNUMBER => v_rdata := std_logic_vector(to_unsigned(GC_REV_BUILDNUMBER, GC_MEM_DATA_W));
                    when ABOUT_REGMAP_GIT_HASH_MSB    => v_rdata := std_logic_vector(to_unsigned(GC_GIT_HASH_MSB, GC_MEM_DATA_W));
                    when ABOUT_REGMAP_GIT_HASH_LSB    => v_rdata := std_logic_vector(to_unsigned(GC_GIT_HASH_LSB, GC_MEM_DATA_W));
                    when ABOUT_REGMAP_GIT_DIRTY       => v_rdata := std_logic_vector(to_unsigned(GC_GIT_DIRTY, GC_MEM_DATA_W));
                    when ABOUT_REGMAP_YYMMDD          => v_rdata := std_logic_vector(to_unsigned(GC_YYMMDD, GC_MEM_DATA_W));
                    when ABOUT_REGMAP_HHMMSS          => v_rdata := std_logic_vector(to_unsigned(GC_HHMMSS, GC_MEM_DATA_W));
                    when ABOUT_REGMAP_ADC_FEATURES    => v_rdata := std_logic_vector(to_unsigned(16#0001#, GC_MEM_DATA_W)); -- ADC features (Delta-Sigma enabled)
                    when others                       => v_rdata := (others => '0');
                end case;

                -- Always set valid on read request
                -- Set and hold rdata bus
                v_rdvalid := '1';
                mem_rdata <= v_rdata;

            end if;

            -- Set valid pulse
            mem_rdvalid <= v_rdvalid;
        end if;
    end process;

end architecture;
