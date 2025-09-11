
-- ************************************************************************
-- About Package for Delta-Sigma ADC Project
-- Contains register map definitions for project information
-- ************************************************************************

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package about_pkg is

    ---------------------------------------------------------------------------
    -- Address Map for About Module
    ---------------------------------------------------------------------------
    constant ABOUT_REGMAP_IMAGE_TYPE      : natural := 0; -- Project type identifier
    constant ABOUT_REGMAP_IMAGE_ID        : natural := 1; -- Project ID
    constant ABOUT_REGMAP_REV_MAJOR       : natural := 2; -- Major version
    constant ABOUT_REGMAP_REV_MINOR       : natural := 3; -- Minor version
    constant ABOUT_REGMAP_REV_PATCH       : natural := 4; -- Patch version
    constant ABOUT_REGMAP_REV_BUILDNUMBER : natural := 5; -- Build number
    constant ABOUT_REGMAP_GIT_HASH_MSB    : natural := 6; -- Git hash upper 32 bits
    constant ABOUT_REGMAP_GIT_HASH_LSB    : natural := 7; -- Git hash lower 32 bits
    constant ABOUT_REGMAP_GIT_DIRTY       : natural := 8; -- Git dirty flag
    constant ABOUT_REGMAP_YYMMDD          : natural := 9; -- Build date
    constant ABOUT_REGMAP_HHMMSS          : natural := 10; -- Build time
    constant ABOUT_REGMAP_ADC_FEATURES    : natural := 11; -- ADC feature flags

    ---------------------------------------------------------------------------
    -- Project-specific constants
    ---------------------------------------------------------------------------
    constant ADC_PROJECT_TYPE : natural := 16#ADCF#; -- "ADC FPGA" project type
    constant ADC_PROJECT_ID   : natural := 16#5000#; -- AXE5000 project ID

end package;

