--
-- Copyright 2012 Jared Boone
-- Copyright 2013 Benjamin Vernoux
--
-- This file is part of HackRF.
--
-- This program is free software; you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation; either version 2, or (at your option)
-- any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program; see the file COPYING.  If not, write to
-- the Free Software Foundation, Inc., 51 Franklin Street,
-- Boston, MA 02110-1301, USA.

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library UNISIM;
use UNISIM.vcomponents.all;

entity top is
    Port(
        HOST_DATA       : inout std_logic_vector(7 downto 0);
        HOST_CAPTURE    : out   std_logic;
		  HOST_SYNC_EN    : in    std_logic;
        HOST_SYNC_CMD   : out   std_logic;
        HOST_SYNC       : in    std_logic;
        HOST_DISABLE    : in    std_logic;
        HOST_DIRECTION  : in    std_logic;
        HOST_Q_INVERT   : in    std_logic;

        DA              : in    std_logic_vector(7 downto 0);
        DD              : out   std_logic_vector(9 downto 0);

        CODEC_CLK       : in    std_logic;
        CODEC_X2_CLK    : in    std_logic
    );

end top;

architecture Behavioral of top is
    signal codec_clk_rx_i : std_logic;
    signal codec_clk_tx_i : std_logic;
    signal adc_data_i : std_logic_vector(7 downto 0);
    signal dac_data_o : std_logic_vector(9 downto 0);

    signal host_clk_i : std_logic;

    type transfer_direction is (from_adc, to_dac);
    signal transfer_direction_i : transfer_direction;

    signal host_data_enable_i : std_logic;
    signal host_data_capture_o : std_logic;

    signal rx_byte_index : unsigned(5 downto 0) := (others => '0');

    signal pps_bits : std_logic_vector(15 downto 0) := (others => '0');

    signal data_from_host_i : std_logic_vector(7 downto 0);
    signal data_to_host_o : std_logic_vector(7 downto 0);

    signal q_invert : std_logic;
    signal rx_q_invert_mask : std_logic_vector(7 downto 0);
    signal tx_q_invert_mask : std_logic_vector(7 downto 0);

begin
    
    ------------------------------------------------
    -- Codec interface
    
    DD(9 downto 0) <= dac_data_o;
    
    ------------------------------------------------
    -- Clocks
    
    BUFG_host : BUFG
    port map (
        O => host_clk_i,
        I => CODEC_X2_CLK
    );

    ------------------------------------------------
    -- SGPIO interface
    
    HOST_DATA <= data_to_host_o when transfer_direction_i = from_adc
                                else (others => 'Z');

    HOST_CAPTURE <= host_data_capture_o;
	 HOST_SYNC_CMD <= '0';
	 
    host_data_enable_i <= not HOST_DISABLE;
    transfer_direction_i <= to_dac when HOST_DIRECTION = '1'
                                   else from_adc;
     
    ------------------------------------------------
        
    q_invert <= HOST_Q_INVERT;
    rx_q_invert_mask <= X"80" when q_invert = '1' else X"7f";
    tx_q_invert_mask <= X"7f" when q_invert = '1' else X"80";
    
    process(host_clk_i)
    begin
        if rising_edge(host_clk_i) then
            codec_clk_rx_i <= CODEC_CLK;
            adc_data_i <= DA(7 downto 0);

            if transfer_direction_i = from_adc then

                if host_data_enable_i = '1' then
                    -- Output IQ bytes for the first 32 byte slots, then PPS metadata bytes.
                    if rx_byte_index < to_unsigned(32, rx_byte_index'length) then
                        if codec_clk_rx_i = '1' then
                            -- I sample
                            data_to_host_o <= adc_data_i xor X"80";
                        else
                            -- Q sample
                            data_to_host_o <= adc_data_i xor rx_q_invert_mask;
                        end if;
                    else
                        case rx_byte_index is
                            when "100000" =>  -- 32
                                data_to_host_o <= pps_bits(7 downto 0);
                            when "100001" =>  -- 33
                                data_to_host_o <= pps_bits(15 downto 8);
                            when others =>    -- 34,35
                                data_to_host_o <= (others => '0');
                        end case;
                    end if;

                    -- Capture PPS level on I-sample boundaries.
                    if (rx_byte_index < to_unsigned(32, rx_byte_index'length)) and
                       (codec_clk_rx_i = '1') then
                        pps_bits <= pps_bits(14 downto 0) & HOST_SYNC;
                    end if;

                    -- Advance or reset byte index for next cycle.
                    if rx_byte_index = "100011" then -- 35
                        rx_byte_index <= (others => '0');
                        pps_bits <= (others => '0');
                    else
                        rx_byte_index <= rx_byte_index + 1;
                    end if;
                else
                    data_to_host_o <= (others => '0');
                    rx_byte_index <= (others => '0');
                    pps_bits <= (others => '0');
                end if;
            else
                -- Reset PPS tracking in TX mode.
                rx_byte_index <= (others => '0');
                pps_bits <= (others => '0');
            end if;
        end if;
    end process;
    
    process(host_clk_i)
    begin
        if falling_edge(host_clk_i) then
            codec_clk_tx_i <= CODEC_CLK;
            data_from_host_i <= HOST_DATA;
            if transfer_direction_i = to_dac then
                if codec_clk_tx_i = '1' then
                    dac_data_o <= (data_from_host_i xor tx_q_invert_mask) & tx_q_invert_mask(0) & tx_q_invert_mask(0);
                else
                    dac_data_o <= (data_from_host_i xor X"80") & "00";
                end if;
            else
                dac_data_o <= (dac_data_o'high => '0', others => '1');
            end if;
        end if;
    end process;
    
    process(host_clk_i)
    begin
        if rising_edge(host_clk_i) then
            if transfer_direction_i = to_dac then
                if codec_clk_tx_i = '1' then
                    host_data_capture_o <= host_data_enable_i;
                end if;
            else
                host_data_capture_o <= host_data_enable_i;
            end if;
        end if;
    end process;
    
end Behavioral;
