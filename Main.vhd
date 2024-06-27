-- <pre> Test, Toplevel Unit

library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		RCK -- Reference Clock
		: in std_logic; 
		XEN, -- Transmit Enable, for data transmission
		TP2, -- Test Point Two, available on P3-2
		SA0 -- LSB of sensor address
		: out std_logic;
		SDA, -- Serial Data	
		SCL, -- Serial Clock
		IDY -- Interrupt/Data Ready
		: inout std_logic;
		xdac -- Transmit DAC Output, to set data transmit frequency
		: out std_logic_vector(4 downto 0));
		
end;

architecture behavior of main is
	
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;
	
-- Attributes to guide the compiler.
	attribute syn_keep : boolean;
	attribute nomerge : string;
	
-- Power Controller
signal USERSTDBY, CLRFLAG, SFLAG, STDBY, RESET : std_logic;
attribute syn_keep of RESET : signal is true;
attribute nomerge of RESET : signal is "";
signal SWRST : boolean := false;

begin

-- We turn off the logic chip bandgap references and other power-hungry
-- circuits with the power controller unit. Within a few milliseconds
-- of power-up, the chip is fully operational, but consuming several 
-- milliamps. We move the chip into standby mode by first clearing the 
-- standby flag with CLRFLAG, then asserting the USERSTDBY control signal
-- that begins the transition to standby mode. The PCU has two outputs: 
-- STDBY and SFLAG. The STDBY signal is intended as a command to put 
-- circuits to sleep, while SFLAG is intended as a signal that the system
-- is in standby mode, which must be cleared after returning to full-power
-- mode. We return to full-power mode when we program the chip.
	Power_Controller: entity PCU port map (
		CLRFLAG => CLRFLAG,
		USERSTDBY => USERSTDBY, 
		STDBY => STDBY,
		SFLAG => SFLAG);	

-- The Power-Up process 
	PowerUp: process (RCK) is
		constant end_state : integer := 7;
		variable state : integer range 0 to end_state := 0;
	begin
		if rising_edge(RCK) then
			CLRFLAG <= to_std_logic(state = 1);
			USERSTDBY <= to_std_logic(state >= 3);
			RESET <= to_std_logic((state < end_state) or SWRST);

			if (state = 0) then state := 1;
			elsif (state = 1) then state := 2;
			elsif (state = 2) then state := 3;
			elsif (SFLAG = '0') then state := 3;
			elsif (state < end_state) then state := state + 1; 
			else state := end_state; end if;
		end if;
	end process;

xdac <= (others =>'Z');
SDA <= 'Z';
SCL <= 'Z';
SA0 <= 'Z';
IDY <= 'Z';
	
-- Test Point Two
	TP2 <= (IDY and SCL and SDA);

end behavior;