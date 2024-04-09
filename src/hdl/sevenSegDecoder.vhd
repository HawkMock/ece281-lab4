----------------------------------------------------------------------------------
-- Company: USAFA
-- Engineer: C3C Dustin Mock
-- 
-- Create Date: 02/20/2024 09:48:45 AM
-- Design Name: Seven Segment Decoder
-- Module Name: sevenSegDecoder - Behavioral
-- Project Name: Seven Segment Decoder
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments: Used ChatGPT to write some logic.
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity sevenSegDecoder is
    Port ( i_D : in STD_LOGIC_VECTOR (3 downto 0);
           o_S : out STD_LOGIC_VECTOR (6 downto 0));
end sevenSegDecoder;

architecture sevenSegDecoder_arch of sevenSegDecoder is
    
begin
    o_S(0) <= (not(i_D(3)) and not(i_D(2)) and not(i_D(1)) and (i_D(0)))
        or ((i_D(3)) and not(i_D(2)) and (i_D(1)) and (i_D(0)))
        or ((i_D(2)) and not(i_D(1)) and not(i_D(0)))
        or ((i_D(3)) and (i_D(2)) and not(i_D(1)));
    o_S(1) <= (not(i_D(3)) and i_D(2) and not(i_D(1)) and i_D(0))
        or (i_D(3) and i_D(2) and not(i_D(0)))
        or (i_D(2) and i_D(1) and not(i_D(0)))
        or (i_D(3) and (i_D(1)) and i_D(0));
    o_S(2) <= (not(i_D(3)) and not(i_D(2)) and i_D(1) and not(i_D(0)))
        or (i_D(3) and i_D(2) and not(i_D(0)))
        or (i_D(3) and i_D(2) and i_D(1));
    o_S(3) <= '1' when ( (i_D = "0001") or
                         (i_D = "0100") or
                         (i_D = "0111") or
                         (i_D = "1111") or
                         (i_D = "1001") or
                         (i_D = "1010") ) else '0';
    o_S(4) <= '1' when ( (i_D = "0001") or
                         (i_D = "0011") or
                         (i_D = "0101") or
                         (i_D = "0100") or
                         (i_D = "0111") or
                         (i_D = "1001") ) else '0';
    o_S(5) <= '1' when ( (i_D = "0001") or
                         (i_D = "0011") or
                         (i_D = "0010") or
                         (i_D = "0111") or
                         (i_D = "1100") or
                         (i_D = "1101") ) else '0';
    o_S(6) <= '1' when ( (i_D = "0000") or
                         (i_D = "0001") or
                         (i_D = "0111") ) else '0';


end sevenSegDecoder_arch;
