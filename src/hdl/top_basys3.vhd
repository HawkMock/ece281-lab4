--+----------------------------------------------------------------------------
--| 
--| COPYRIGHT 2018 United States Air Force Academy All rights reserved.
--| 
--| United States Air Force Academy     __  _______ ___    _________ 
--| Dept of Electrical &               / / / / ___//   |  / ____/   |
--| Computer Engineering              / / / /\__ \/ /| | / /_  / /| |
--| 2354 Fairchild Drive Ste 2F6     / /_/ /___/ / ___ |/ __/ / ___ |
--| USAF Academy, CO 80840           \____//____/_/  |_/_/   /_/  |_|
--| 
--| ---------------------------------------------------------------------------
--|
--| FILENAME      : top_basys3.vhd
--| AUTHOR(S)     : Capt Phillip Warner, Capt Dan Johnson, and C3C Dustin Mock
--| CREATED       : 2018-03-09  Modified on 2024-04-15
--| DESCRIPTION   : This file implements the top level module for a BASYS 3 to 
--|					drive the Lab 4 Design Project (Advanced Elevator Controller).
--|
--|					Inputs: clk       --> 100 MHz clock from FPGA
--|							btnL      --> Rst Clk
--|							btnR      --> Rst FSM
--|							btnU      --> Rst Master
--|							btnC      --> GO (request floor)
--|							sw(1) --> up_down
--|                         sw(0) --> stop
--|							 
--|					Outputs: led --> indicates the clock speed at which the FSM is updating
--|							   - led(15) = slow clock (FSM) high
--|							 an(3:0)    --> seven-segment display anode active-low enable (AN3 ... AN0)
--|							 seg(6:0)	--> seven-segment display cathodes (CG ... CA.  DP unused)
--|
--| DOCUMENTATION : I asked Maj Seery for help assigning "subvectors" such as
--|                 an(3:2) written as an(3 downto 2). I used ChatGPT to formulaicly
--|                 extend the functionality of my FSM to 16 floors. See prompts here:
--|                 https://chat.openai.com/share/7c202563-8f2c-47f2-9d29-a4d5e05a4eec
--|
--+----------------------------------------------------------------------------
--|
--| REQUIRED FILES :
--|
--|    Libraries : ieee
--|    Packages  : std_logic_1164, numeric_std
--|    Files     : MooreElevatorController.vhd, clock_divider.vhd, sevenSegDecoder.vhd
--|				   thunderbird_fsm.vhd, sevenSegDecoder, TDM4.vhd, OTHERS???
--|
--+----------------------------------------------------------------------------
--|
--| NAMING CONVENSIONS :
--|
--|    xb_<port name>           = off-chip bidirectional port ( _pads file )
--|    xi_<port name>           = off-chip input port         ( _pads file )
--|    xo_<port name>           = off-chip output port        ( _pads file )
--|    b_<port name>            = on-chip bidirectional port
--|    i_<port name>            = on-chip input port
--|    o_<port name>            = on-chip output port
--|    c_<signal name>          = combinatorial signal
--|    f_<signal name>          = synchronous signal
--|    ff_<signal name>         = pipeline stage (ff_, fff_, etc.)
--|    <signal name>_n          = active low signal
--|    w_<signal name>          = top level wiring signal
--|    g_<generic name>         = generic
--|    k_<constant name>        = constant
--|    v_<variable name>        = variable
--|    sm_<state machine type>  = state machine type definition
--|    s_<signal name>          = state name
--|
--+----------------------------------------------------------------------------
library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;


-- Lab 4
entity top_basys3 is
    port(
        -- inputs
        clk     :   in std_logic; -- native 100MHz FPGA clock
        sw      :   in std_logic_vector(15 downto 0);
        btnU    :   in std_logic; -- master_reset
        btnL    :   in std_logic; -- clk_reset
        btnR    :   in std_logic; -- fsm_reset
        
        -- outputs
        led :   out std_logic_vector(15 downto 0);
        -- 7-segment display segments (active-low cathodes)
        seg :   out std_logic_vector(6 downto 0);
        -- 7-segment display active-low enables (anodes)
        an  :   out std_logic_vector(3 downto 0)
    );
end top_basys3;

architecture top_basys3_arch of top_basys3 is 
	-- declare components and signals
	component elevator_controller_fsm is
        Port ( i_clk     : in  STD_LOGIC;
               i_reset   : in  STD_LOGIC;
               i_stop    : in  STD_LOGIC;
               i_up_down : in  STD_LOGIC;
               o_floor   : out STD_LOGIC_VECTOR (3 downto 0)           
             );
    end component elevator_controller_fsm;
	
    component sevenSegDecoder is
        Port ( i_D : in STD_LOGIC_VECTOR (3 downto 0);
               o_S : out STD_LOGIC_VECTOR (6 downto 0));
    end component sevenSegDecoder;
    
    component clock_divider is
        generic ( constant k_DIV : natural := 2    ); -- How many clk cycles until slow clock toggles
                                                   -- Effectively, you divide the clk double this 
                                                   -- number (e.g., k_DIV := 2 --> clock divider of 4)
        port (     i_clk    : in std_logic;
                i_reset  : in std_logic;           -- asynchronous
                o_clk    : out std_logic           -- divided (slow) clock
        );
    end component clock_divider;
    
    component hex2dec is
        Port ( i_hex : in STD_LOGIC_VECTOR (3 downto 0);
               o_tens : out STD_LOGIC_VECTOR (3 downto 0);
               o_ones : out STD_LOGIC_VECTOR (3 downto 0)
             );
    end component hex2dec;
    
    component TDM4 is 
    generic ( constant k_WIDTH : natural  := 4); -- bits in input and output
        Port ( i_clk        : in  STD_LOGIC;
               i_reset        : in  STD_LOGIC; -- asynchronous
               i_D3         : in  STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
               i_D2         : in  STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
               i_D1         : in  STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
               i_D0         : in  STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
               o_data        : out STD_LOGIC_VECTOR (k_WIDTH - 1 downto 0);
               o_sel        : out STD_LOGIC_VECTOR (3 downto 0)    -- selected data line (one-cold)
        );
    end component TDM4;
     
    signal w_fast_clk, w_slow_clk : std_logic;
    signal w_tens, w_ones, w_data, w_anode, w_floor : std_logic_vector(3 downto 0);
  
begin
	-- PORT MAPS ----------------------------------------
    elevator_controller_fsm_inst : elevator_controller_fsm
    port map (
       i_clk        => w_slow_clk,
       i_reset      => btnR or btnU,
       i_stop       => sw(0),
       i_up_down    => sw(1),
       o_floor      => w_floor  
     );
     
     sevenSegDecoder_inst : sevenSegDecoder
     port map (
        i_D => w_data,
        o_S => seg
     );
     
     clock_divider_slow_inst : clock_divider
     generic map ( k_DIV => 25000000 ) -- 4 Hz clock from 100 MHz
     port map (                          
         i_clk   => clk,
         i_reset => btnL or btnU,
         o_clk   => w_slow_clk
     ); 
     
     clock_divider_fast_inst : clock_divider
     generic map ( k_DIV => 125000 ) -- 400 Hz clock from 100 MHz
     port map (                          
         i_clk   => clk,
         i_reset => btnL or btnU,
         o_clk   => w_fast_clk
     ); 
     
     TDM4_inst : TDM4
     generic map ( k_WIDTH => 4 ) -- bits in input and output
     port map (
            i_clk   => w_fast_clk,
            i_reset => btnU,
            i_D3    => w_tens,
            i_D2    => w_ones,
            i_D1    => x"A",
            i_D0    => x"B",
            o_data  => w_data,
            o_sel   => w_anode
     );
     
     hex2dec_inst : hex2dec
     port map ( i_hex => w_floor,
                o_tens => w_tens,
                o_ones => w_ones
     );
	
	
	-- CONCURRENT STATEMENTS ----------------------------
	
	-- LED 15 gets the FSM slow clock signal. The rest are grounded.
	led(15) <= w_slow_clk;
	led(14 downto 0) <= (others => '0');

	-- leave unused switches UNCONNECTED. Ignore any warnings this causes.
	
	-- wire up active-low 7SD anodes (an) as required
	-- Tie any unused anodes to power ('1') to keep them off
    an(3 downto 2) <= w_anode(3 downto 2);
    an(1 downto 0) <= "11";
	
end top_basys3_arch;
