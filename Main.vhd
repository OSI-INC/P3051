-- <pre> Implantable Inertial Sensor (IIS, A3035) Firmware, Toplevel Unit

-- V2.1, 15-JUL-22: Move to the OSR8V3. Add a flag to disable the multiplier so we can be sure variants of the-- OSR8 fit in the device. The initial stack pointer is now available at mmu_sph and mmu_spl locations, HI and LO 
-- bytes respectively, so the CPU process can read the correct stack location from memory and set the stack pointer
-- after a jump to the initialization routine. This OSR8V3 is behind in modification from the one in P3041. Program
-- a new IIS and test: seems to be working. We create P3035 Git repository and replace old version A13 with new
-- version V2.1 and tag.

-- V2.3, 14-SEP-22: Carry latest OSR8V3 modifications into this firmware. Eliminate mmu_sph and mmu_spl. No longer
-- have Stack Overflow (STOF) interrupt. Stack pointer will be initialized by CPU from program on startup.

library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		RCK, -- Reference Clock
		SDO,  -- Sensor Serial Data Out
		INTA, -- Accelerometer Interrupt
		INTG -- Gyroscope Interrupt
		: in std_logic; 
		XEN, -- Transmit Enable, for data transmission
		TP1, -- Test Point One, available on P3-1
		TP2, -- Test Point Two, available on P3-2
		TP3, -- Test Point Three, available on P1-6
		TP4, -- Test Point Four, available on P1-8
		NCSA, -- Acceleromtere Chip Select, Negative-True
		NCSG -- Gyroscope Chip Select, Negative-True
		: out std_logic;
		SDI, -- Serial Data In, for Acceleromater and Gyroscope		
		SCK -- Serial Clock, for Acceleromater and Gyroscope
		: inout std_logic;
		xdac -- Transmit DAC Output, to set data transmit frequency
		: out std_logic_vector(4 downto 0));
		
-- Configuration of OSR8 CPU.
	constant prog_addr_len : integer := 12;
	constant cpu_addr_len : integer := 11;
	constant start_pc : integer := 0;
	constant interrupt_pc : integer := 3;

-- Configuration of peripherals.
	constant enable_multiplier : boolean := true;
	constant ram_addr_len : integer := 10;
	
-- Memory map sizes and base addresses in units of 512 bytes.
	constant ram_base : integer := 0;
	constant ram_range : integer := 2;
	constant ctrl_base : integer := 3;
	constant ctrl_range : integer := 1;	

-- Control space locations, offset from control space base address.
	constant mmu_shb  : integer := 16#00#; -- Sensor Data HI Byte 
	constant mmu_slb  : integer := 16#01#; -- Sensor Data LO Byte
	constant mmu_sar  : integer := 16#02#; -- Sensor Address Register
	constant mmu_scr  : integer := 16#04#; -- Sensor Control Register
	constant mmu_irqb : integer := 16#10#; -- Interrupt Request Bits
	constant mmu_imsk : integer := 16#12#; -- Interrupt Mask Bits
	constant mmu_irst : integer := 16#14#; -- Interrupt Reset Bits
	constant mmu_iset : integer := 16#16#; -- Interrupt Set Bits
	constant mmu_itp  : integer := 16#18#; -- Interrupt Timer Period 
	constant mmu_rst  : integer := 16#19#; -- System Reset
	constant mmu_xhb  : integer := 16#20#; -- Transmit HI Byte
	constant mmu_xlb  : integer := 16#21#; -- Transmit LO Byte
	constant mmu_xcn  : integer := 16#22#; -- Transmit Channel Number
	constant mmu_xcr  : integer := 16#24#; -- Transmit Control Register
	constant mmu_xfc  : integer := 16#26#; -- Transmit Frequency Calibration
	constant mmu_etc  : integer := 16#30#; -- Enable Transmit Clock
	constant mmu_tcf  : integer := 16#32#; -- Transmit Clock Frequency
	constant mmu_tcd  : integer := 16#34#; -- Transmit Clock Divider
	constant mmu_bcc  : integer := 16#36#; -- Boost CPU Clock
	constant mmu_tpr  : integer := 16#38#; -- Test Point Register
	constant mmu_mia  : integer := 16#3A#; -- Multiplier Input A
	constant mmu_mib  : integer := 16#3B#; -- Multiplier Input B
	constant mmu_moh  : integer := 16#3C#; -- Multiplier Output HI	
	constant mmu_mol  : integer := 16#3D#; -- Multiplier Output LO
end;

architecture behavior of main is

-- Attributes to guide the compiler.
	attribute syn_keep : boolean;
	attribute nomerge : string;

-- Power Controller
	signal USERSTDBY, CLRFLAG, SFLAG, STDBY, RESET : std_logic;
	attribute syn_keep of RESET : signal is true;
	attribute nomerge of RESET : signal is "";
	signal SWRST : boolean := false;
	
-- Ring Oscillator and Transmit Clock
	signal TCK, FCK, CK, FCKEN : std_logic;
	attribute syn_keep of TCK, FCK, CK : signal is true;
	attribute nomerge of TCK, FCK, CK : signal is "";  

-- Sensor Readout
	signal CSA, CSG : boolean;
	
-- Message Transmission.
	signal TXI, -- Transmit Initiate
		TXA, -- Transmit Active
		TXD, -- Transmit Done
		TXB, -- Transmit Bit
		FHI -- Frequency High
		: boolean := false;
	attribute syn_keep of TXI, TXA, TXD: signal is true;
	attribute nomerge of TXI, TXA, TXD : signal is "";  
	signal xmit_bits -- Sixteen bits to be transmitted as a message.
		: std_logic_vector(15 downto 0) := (others => '0');
	signal tx_channel : integer range 0 to 255 := 1; -- Transmit channel number
	signal frequency_low : integer range 0 to 15 := 6; -- Low frequency for transmission
	constant frequency_step : integer := 1; -- High minus low frequency
		
-- Sensor Interface
	signal SBYI, -- Sensor Byte Access Initiate 
		SBYD, -- Sensor Byte Access Done
		SBYW, -- Sensor Byte Access Write
		GYSEL, -- Gyroscope Select
		SBYC, -- Sensor Access Continue
		SCKE -- Serial Clock Early
		: boolean := false;
	attribute syn_keep of SBYI, SBYD, SCKE : signal is true;
	attribute nomerge of SBYI, SBYD, SCKE : signal is "";  
	signal sensor_bits_out, -- Eight bits that we will transmit to a sensor.
		sensor_bits_in -- Eight bits that we will receive from a sensor.
		: std_logic_vector(7 downto 0) := (others => '0');

-- Sensor Controller
	signal SAI, -- Sensor Access Initiate 
		SAA, -- Sensor Access Active
		SAD, -- Sensor Access Done
		SAWR, -- Sensor Write
		SA16 -- Sensor Access Sixteen Bits
		: boolean := false;
	attribute syn_keep of SAI, SAA, SAD : signal is true;
	attribute nomerge of SAI, SAA, SAD : signal is "";  
	signal sensor_addr -- Eight bit address for sensor, will use only first seven.
		: std_logic_vector(7 downto 0) := (others => '0');
	signal sensor_data_in, -- Data bits we read from a sensor.
		sensor_data_out -- Data bits we want to write to a sensor.
		: std_logic_vector(15 downto 0) := (others => '0');
		
-- Clock Calibrator
	signal ENTCK : boolean; -- Enable the Transmit Clock
	signal tck_frequency : integer range 0 to 255; -- Transmit Clock Counter
	constant default_tck_divisor : integer := 11;
	signal tck_divisor : integer range 0 to 15 := default_tck_divisor;
	
-- Boost Controller
	signal BOOST : boolean;
	
-- CPU-Writeable Test Points
	signal tp_reg : std_logic_vector(7 downto 0) := (others => '0');
	
-- Program Memory Signals
	signal prog_data : std_logic_vector(7 downto 0); -- ROM Data
	signal prog_addr : std_logic_vector(prog_addr_len-1 downto 0); -- ROM Address
	
-- Random Access Memory Signals
	constant ra_top : integer := ram_addr_len-1;
	signal ram_addr : std_logic_vector(ra_top downto 0); -- RAM Address
	signal ram_out, ram_in : std_logic_vector(7 downto 0); -- RAM Data In and Out
	signal RAMWR : std_logic; -- Command Memory Write
	
-- Eight-Bit Multiplier Signals
	signal cpu_multiplier_a, cpu_multiplier_b : std_logic_vector(7 downto 0);
	signal cpu_multiplier_out : std_logic_vector(15 downto 0);

-- Central Processing Unit Signals
	signal cpu_data_out, cpu_data_in : std_logic_vector(7 downto 0); 
	constant ca_top : integer := cpu_addr_len-1;
	signal cpu_addr : std_logic_vector(ca_top downto 0);
	attribute syn_keep of cpu_addr : signal is true;
	attribute nomerge of cpu_addr : signal is "";  
	signal CPUWR, -- Write (Not Read)
		CPUDS, -- Data Strobe
		CPUIRQ -- Interrupt Request
		: boolean; 
	signal CPUSIG : std_logic_vector(2 downto 0); -- Signals for debugging.

-- Interrupt Handler signals.
	signal int_mask, int_period, int_bits, 
		int_rst, int_set : std_logic_vector(7 downto 0);
	signal TXDS, SADS, INTGS, INTAS : boolean;
	signal INTCTRZ : boolean; -- Interrupt Counter Zero Flag
	
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;

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
	
	
-- Ring Oscillator. This oscillator runs only during message transmission.
	FCKEN <= to_std_logic(TXI or TXA or TXD or SAI or SAA or SAD or ENTCK);
	Fast_CK : entity ring_oscillator port map (
		ENABLE => FCKEN, 
		calib => tck_divisor,
		CK => FCK);
	
-- The Transmit Clock process divides FCK in two so as to produce a clock with
-- exactly 50% duty cycle and frequency close to 5 MHz, which we call the 
-- Transmit Clock (TCK). We clock TCK on the falling edge of FCK.
	Tx_CK : process (FCK) is 
	begin
		if falling_edge(FCK) then TCK <= to_std_logic(TCK = '0'); end if;
	end process;

-- User memory and configuration code for the CPU. This RAM will be initialized at
-- start-up with a configuration file, and so may be read after power up to configure
-- sensor. The configuration data will begin at address zero.
	Process_Memory : entity RAM port map (
		Clock => not CK,
		ClockEn => '1',
        Reset => RESET,
		WE => RAMWR,
		Address => ram_addr, 
		Data => ram_in,
		Q => ram_out);

-- Instruction Memory for CPU. This read-only memory will be initialized with the
-- CPU program, the first instruction of the program being stored at address zero.
-- The CPU reads the instruction memory with a separate address bus, which we call
-- the program counter.
	Program_Memory : entity ROM port map (
		Address => prog_addr,
        OutClock => not CK,
        OutClockEn => '1',
        Reset => RESET,	
        Q => prog_data);

-- The eight-bit multiplier takes two eight-bit numbers and produces their sixteen-
-- bit product.
	Multiplier : entity MULT port map (
		Clock => not CK,
		ClkEn => '1',
		Aclr => RESET,
		DataA => cpu_multiplier_a,
		DataB => cpu_multiplier_b,
		Result => cpu_multiplier_out);
	
-- The processor itself.
	CPU : entity OSR8_CPU 
		generic map (
			prog_addr_len => prog_addr_len,
			cpu_addr_len => cpu_addr_len,
			start_pc => start_pc,
			interrupt_pc => interrupt_pc
		)
		port map (
			prog_data => prog_data,
			prog_addr => prog_addr,
			cpu_data_out => cpu_data_out,
			cpu_data_in => cpu_data_in,
			cpu_addr => cpu_addr,
			WR => CPUWR,
			DS => CPUDS,
			IRQ => CPUIRQ,
			SIG => CPUSIG,
			RESET => RESET,
			CK => CK
		);
		
-- The Memory Manager maps eight-bit read and write access to the 
-- Sensor Controller, Sample Transmitter, Random Access Memory, and 
-- Interrupt Handler. Byte ordering is big-endian (most significant byte at
-- lower address). 
	MMU : process (CK,RESET) is
		variable top_bits : integer range 0 to 15;
		variable bottom_bits : integer range 0 to 63;
	begin
	
		-- Some variables for brevity.
		top_bits := to_integer(unsigned(cpu_addr(ca_top downto 9)));
		bottom_bits := to_integer(unsigned(cpu_addr(5 downto 0)));
		
		-- We want the following signals to be combinatorial functions
		-- of the address. Here we define their default values.
		RAMWR <= '0';
		ram_in <= cpu_data_out;
		ram_addr <= cpu_addr(ra_top downto 0);
		cpu_data_in <= (others => '0');		
		
		-- These signals develop after the CPU assserts a new address
		-- along with CPU Write and CPU Sixteen-Bit Access. They will
		-- be ready before the falling edge of the CPU clock.
		case top_bits is
		when ram_base to (ram_base+ram_range-1) => 
			if not CPUWR then
				cpu_data_in <= ram_out;
			else
				RAMWR <= to_std_logic(CPUDS);
			end if;
		when ctrl_base to (ctrl_base+ctrl_range-1) =>
			if not CPUWR then 
				case bottom_bits is
				when mmu_shb => cpu_data_in <= sensor_data_in(15 downto 8);
				when mmu_slb => cpu_data_in <= sensor_data_in(7 downto 0);
				when mmu_irqb => cpu_data_in <= int_bits;
				when mmu_imsk => cpu_data_in <= int_mask;
				when mmu_itp => cpu_data_in <= int_period;
				when mmu_tcf =>
					cpu_data_in <= std_logic_vector(to_unsigned(tck_frequency,8));
				when mmu_moh => 
					if enable_multiplier then
						cpu_data_in <= cpu_multiplier_out(15 downto 8);
					end if;
				when mmu_mol => 
					if enable_multiplier then
						cpu_data_in <= cpu_multiplier_out(7 downto 0);
					end if;
				end case;
			end if;
		end case;
		
		-- We use RESET to clear some registers and signals, but not all. We do not clear the
		-- software reset signal, SWRST, on RESET, since we want SWRST to assert RESET for one
		-- CK period. After a reset, the cpu address will not select the SWRST location, so
		-- SWRST will be cleared on the next falling edge of CK.
		if (RESET = '1') then
			SAI <= false;
			TXI <= false;
			ENTCK <= false;
			BOOST <= false;
			tck_divisor <= default_tck_divisor;
			tx_channel <= 0;
			int_period <= (others => '0');
			int_mask <= (others => '0');
		-- We use the falling edge of RCK to write to registers and to initiate sensor 
		-- and transmit activity. Some signals we assert only for one CK period, and 
		-- these we assert as false by default.
		elsif falling_edge(CK) then
			SWRST <= false;
			SAI <= false;
			TXI <= false;
			int_rst <= (others => '0');
			int_set <= (others => '0');
			if CPUDS and CPUWR then 
				if (top_bits >= ctrl_base) 
						and (top_bits <= ctrl_base+ctrl_range-1) then
					case bottom_bits is
					when mmu_shb => sensor_data_out(15 downto 8) <= cpu_data_out;
					when mmu_slb => sensor_data_out(7 downto 0) <= cpu_data_out;
					when mmu_sar => sensor_addr(7 downto 0) <= cpu_data_out;
					when mmu_scr => 
						SAI <= true;
						GYSEL <= (cpu_data_out(0) = '1');
						SAWR <= (cpu_data_out(1) = '1');
						SA16 <= (cpu_data_out(2) = '1');
					when mmu_xlb => xmit_bits(7 downto 0) <= cpu_data_out;
					when mmu_xhb => xmit_bits(15 downto 8) <= cpu_data_out;
					when mmu_xcn => tx_channel <= to_integer(unsigned(cpu_data_out));
					when mmu_xcr => TXI <= true;
					when mmu_xfc => frequency_low <= to_integer(unsigned(cpu_data_out));
					when mmu_imsk => int_mask <= cpu_data_out;
					when mmu_itp => int_period <= cpu_data_out;
					when mmu_irst => int_rst <= cpu_data_out;
					when mmu_iset => int_set <= cpu_data_out;
					when mmu_rst => SWRST <= (cpu_data_out(0) = '1');
					when mmu_etc => ENTCK <= (cpu_data_out(0) = '1');
					when mmu_tcd => tck_divisor <= to_integer(unsigned(cpu_data_out));
					when mmu_bcc => BOOST <= (cpu_data_out(0) = '1');
					when mmu_tpr => tp_reg <= cpu_data_out;
					when mmu_mia => cpu_multiplier_a <= cpu_data_out;
					when mmu_mib => cpu_multiplier_b <= cpu_data_out;
					end case;
				end if;
			end if;
		end if;
	end process;
	
	-- The Clock Calibrator counts cycles of TCK for one half-period of RCK after the
	-- assertion of Enable Transmit Clock (ENTCK) and makes them available to the CPU
	-- in the tck_frequency register. If TCK is 5.00 MHz and RCK is 32.768 kHz, 
	-- tck_frequency will be 76 when the counter stops. The counter will hold its 
	-- value until ENTCK is unasserted.
	Clock_Calibrator : process (TCK,ENTCK) is
	variable state, next_state : integer range 0 to 3;
	begin
		if not ENTCK then
			state := 0;
			tck_frequency <= 0;
		elsif rising_edge(TCK) then
			next_state := state;
			if (state = 0) then
				if ENTCK then 
					next_state := 1;
				end if;
				tck_frequency <= 0;
			elsif (state = 1) then
				if (RCK = '1') then 
					next_state := 2;
				end if;
				tck_frequency <= tck_frequency + 1;
			elsif (state = 2) then
				if not ENTCK then 
					next_state := 0;
				end if;
				tck_frequency <= tck_frequency;
			else 
				next_state := 0;
				tck_frequency <= tck_frequency;
			end if;
			state := next_state;
		end if;
	end process;
	
	-- The Boost Controller switches the CPU bewteen RCK and TCK, but makinge 
	-- sure TCK is enabled for two cycles before connecting the CPU clock to
	-- TCK. The CPU must first enable TCK with ENTCK, then assert BOOST. When
	-- switching back to RCK, it must first unassert BOOST, then unassert ENTCK.
	Boost_Controller : process (TCK,ENTCK) is
	variable state, next_state : integer range 0 to 3;
	begin
		if not ENTCK then
			state := 0;
		elsif rising_edge(TCK) then
			case state is
				when 0 =>
					if BOOST then 
						next_state := 1;
					else 
						next_state := 0;
					end if;
				when 1 => next_state := 3;
				when 3 =>
					if (not BOOST) then
						next_state := 2;
					else
						next_state := 3;
					end if;
				when 2 => 
					if (RCK = '0') then
						next_state := 0;
					else
						next_state := 2;
					end if;
			end case;
			state := next_state;
		end if;
		CK <= to_std_logic(((RCK = '1') and (state = 0))
			or ((TCK = '1') and (state = 3))
			or (state = 2));
	end process;

	-- The Interrupt_Controller provides the interrupt signal to the CPU in response to
	-- sensor and timer events. By default, at power-up, all interrupts are maske.
	Interrupt_Controller : process (RCK,CK,RESET) is
	variable counter : integer range 0 to 255;
	begin
		-- The timer itself, counting down from int_period to zero with
		-- period RCK. It never stops, so we can generate regular, periodic 
		-- interrupts. We use the falling edge of RCK to count down, or 
		-- else the compiler gets confused when generating our delayed signal
		-- INTCTRZ in the next section.
		if falling_edge(RCK) then
			if (counter = 0) then
				counter := to_integer(unsigned(int_period));
			else
				counter := counter - 1;
			end if;
		end if;

		-- The interrupt management runs of CK, which can be RCK or TCK.
		if (RESET = '1') then
			CPUIRQ <= false;
			int_bits <= (others => '0');
			INTCTRZ <= false;
		elsif rising_edge(CK) then
		
			-- Edge detecting signals.
			TXDS <= TXD;
			SADS <= SAD;
			INTGS <= (INTG = '1');
			INTAS <= (INTA = '1');
			
			-- The timer interrupt is set when the counter is zero
			-- and reset when we write of 1 to int_rst(0).
			INTCTRZ <= (counter = 0);
			if (int_rst(0) = '1') then
				int_bits(0) <= '0';				
			elsif ((counter = 0) and (not INTCTRZ))
					or (int_set(0) = '1') then
				int_bits(0) <= '1';
			end if;
			
			-- The TXD interrupt is set on a rising edge of
			-- TXD and reset upon a write of 1 to int_rst(1).
			if (int_rst(1) = '1') then
				int_bits(1) <= '0';
			elsif ((not TXDS) and TXD) 
					or (int_set(1) = '1') then
				int_bits(1) <= '1';
			end if;
			
			-- The SAD interrupt is set on a rising edge of
			-- SAD and reset upon a write of 1 to int_rst(2).
			if (int_rst(2) = '1') then
				int_bits(2) <= '0';
			elsif ((not SADS) and SAD) 
					or (int_set(2) = '1') then
				int_bits(2) <= '1';
			end if;
			
			-- The gyroscope interrupt is set on a rising edge
			-- of INTG and reset by a write of 1 to to int_rst(3)
			if (int_rst(3) = '1') then
				int_bits(3) <= '0';
			elsif ((not INTGS) and (INTG = '1')) 
					or (int_set(3) = '1')  then
				int_bits(3) <= '1';
			end if;
			
			-- The accelerometer interrupt is set on a rising edge
			-- of INTA and reset by a write of 1 to to int_rst(3)
			if (int_rst(4) = '1') then
				int_bits(4) <= '0';
			elsif ((not INTAS) and (INTA = '1')) 
					or (int_set(4) = '1') then
				int_bits(4) <= '1';
			end if;
			
			-- The CPU dedicated inputs INT1..INT3 the CPU sets and resets
			-- itself through the int_rst and int_set control registers.
			for i in 5 to 7 loop
				if (int_rst(i) = '1') then
					int_bits(i) <= '0';
				elsif (int_set(i) = '1') then
					int_bits(i) <= '1';
				end if;
			end loop;
		end if;

		-- We generate an interrupt if any one interrupt bit is 
		-- set and unmasked.
		CPUIRQ <= (int_bits and int_mask) /= "00000000";
	end process;
	

	-- The Sensor Controller receives instructions from the CPU through the signals
	-- Sensor Access Write (SAWR), Sensor Access Initiate (SAI), and Sensor Access 
	-- Sixteen Bits (SA16). When it is done with its task, it asserts Sensor Access 
	-- Done (SAD). Ir runs off the Transmit Clock (TCK), which starts up with the 
	-- assertion of SAI and continues until both SAI and SAD are un-asserted.
	Sensor_Controller : process (TCK,RESET) is
		variable state, next_state : integer range 0 to 15 := 0;
		constant idle : integer := 0;
		constant write_addr : integer := 1;
		constant prep_b1 : integer := 2;
		constant write_b1 : integer := 3;
		constant read_b1 : integer := 4;
		constant prep_d : integer := 5;
		constant read_d : integer := 6;
		constant prep_b2 : integer := 7;
		constant write_b2 : integer := 8;
		constant read_b2 : integer := 9;
		constant all_done : integer := 15;
		
 	begin
		-- Upon startup, we make sure we are in the idle state and we are not
		-- requesting a byte access by the Sensor Interface.
		if (RESET = '1') then 
			state := idle;
			SBYI <= false;
			SBYC <= false;
			
		-- The Sensor Contoller proceeds through states so as to manage single and
		-- double-bytes reads and writes from either sensor. Double-byte reads will
		-- be performed with a single serial interface access, so we can take
		-- advantage of the shadowing provided by sensors that keeps both bytes
		-- of a measurement stable during a two-byte read.
		elsif rising_edge(TCK) then
			next_state := state;
			
			case state is
				when idle => 
					SBYI <= false;
					if SAI then 
						next_state := write_addr;
					end if;
					
				when write_addr =>
					SBYI <= true;
					SBYC <= true;
					SBYW <= true;
					sensor_bits_out(6 downto 0) <= sensor_addr(6 downto 0);
					sensor_bits_out(7) <= to_std_logic(not SAWR);
					if SBYD then 
						if (not GYSEL) and (not SAWR) then
							next_state := prep_d; 
						else
							next_state := prep_b1;
						end if;
					end if;
					
				when prep_d =>
					SBYI <= false;
					next_state := read_d;
					
				when read_d =>
					SBYI <= true;
					SBYW <= false;
					if SBYD then 
						next_state := prep_b1;
					end if;
					
				when prep_b1 =>
					SBYI <= false;
					if SAWR then 
						next_state := write_b1;
					else
						next_state := read_b1;
					end if;
				
				when write_b1 =>
					SBYI <= true;
					SBYC <= SA16;
					SBYW <= true;
					sensor_bits_out <= sensor_data_out(7 downto 0);
					if SBYD then 
						if SA16 then 
							next_state := prep_b2;
						else
							next_state := all_done;
						end if;
					end if;

				when read_b1 =>
					SBYI <= true;
					SBYC <= SA16;
					SBYW <= false;
					if SBYD then 
						sensor_data_in(7 downto 0) <= sensor_bits_in;
						if SA16 then 
							next_state := prep_b2;
						else
							next_state := all_done;
						end if;
					end if;

				when prep_b2 =>
					SBYI <= false;
					if SAWR then 
						next_state := write_b2;
					else
						next_state := read_b2;
					end if;
				
				when write_b2 =>
					SBYI <= true;
					SBYC <= false;
					SBYW <= true;
					sensor_bits_out <= sensor_data_out(15 downto 8);
					if SBYD then 
						next_state := all_done;
					end if;

				when read_b2 =>
					SBYI <= true;
					SBYC <= false;
					SBYW <= false;
					if SBYD then 
						sensor_data_in(15 downto 8) <= sensor_bits_in;
						next_state := all_done;
					end if;

				when all_done =>
					SBYI <= false;
					SAD <= true;
					if not SAI then 
						next_state := idle;
					end if;
					
				when others => 
					next_state := idle;
			end case;
			SAD <= (state = all_done);
			SAA <= (state /= idle) and (state /= all_done);
			state := next_state;
		end if;
	end process;
	
	-- The Sensor Interface reads or writes eight bits at a time to the gyroscope 
	-- or the accelerometer. When Sensor Byte Write (SBYW) is asserted, the contents 
	-- of sensor_bits_out are transmitted on the SDI signal, most significant byte 
	-- first. Otherwise, the SDO signal is loaded into sensor_bits_in, most significant
	-- bit first. When GYSEL is asserted, we access the gyroscope, otherwise the 
	-- accelerometer. When SACNT is asserted, we continue to assert the sensor chip
	-- select after the end of the access. The entire process is begun by Sensor Byte
	-- Access Initiate (SBYI). The process asserts Sensor Byte Access Done (SBYD) when
	-- all eight bits have been exchanged.
	Sensor_Interface : process (TCK,FCK,SBYI) is
		constant num_bits : integer := 8;
		constant start_sck : integer := 3;
		constant end_sck : integer := start_sck + num_bits - 1;
		constant all_done : integer := end_sck + 3;
		variable state : integer range 0 to 63 := 0;
				
	begin
		-- We use SBYI as asynchronous reset for the state variable and the done
		-- flag, which makes sure that SBYD clears as soon as the Sensor Controller
		-- unasserst SBYI.
		if (not SBYI) then
			state := 0;
			SBYD <= false;
		
		-- We use the Transmit Clock (TCK), which runs at 5 MHz, to drive the byte
		-- exchange. With fourteen states, the exchange takes 2.8 us.
		elsif rising_edge(TCK) then
			if (state < all_done) then 
				state := state + 1;
			else 
				state := all_done; 
			end if;
			
			-- We assert the sensor chip select during any sensor access, just before
			-- the first falling edge of SCK and until just after the last rising edge
			-- of SCK. After this last rising edge, the chip select will be remain 
			-- asserted only if Serial Byte Continue is asserted. If chip select happens
			-- to be asserted when we start up our Sensor Interface, we leave it asserted,
			-- because the current byte exchange is a continuation of an on-going 
			-- exchange.
			if (state = start_sck - 1) then
				if GYSEL then 
					CSG <= true;
				else
					CSA <= true;
				end if;
			elsif (state = all_done) then
				if not SBYC then 
					if GYSEL then 
						CSG <= false;
					else
						CSA <= false;
					end if;
				end if;
			end if;
				
			-- On a write cycle, we assert the sensor output bits one after another
			-- on the rising edges of TCK.
			if SBYW then
				SDI <= to_std_logic(
					((state = start_sck + 0) and (sensor_bits_out(7) = '1'))
					or ((state = start_sck + 1) and (sensor_bits_out(6) = '1'))
					or ((state = start_sck + 2) and (sensor_bits_out(5) = '1'))
					or ((state = start_sck + 3) and (sensor_bits_out(4) = '1'))
					or ((state = start_sck + 4) and (sensor_bits_out(3) = '1'))
					or ((state = start_sck + 5) and (sensor_bits_out(2) = '1'))
					or ((state = start_sck + 6) and (sensor_bits_out(1) = '1'))
					or ((state = start_sck + 7) and (sensor_bits_out(0) = '1'))
				); 
			end if;
			
			-- SBYD indicates to other processes that the sensor access is complete.
			SBYD <= (state = all_done);			
		end if;
		
		-- On a read cycle, we detect the value of SDO on the faling edges of TCK.
		if falling_edge(TCK) then
			if (not SBYW) then 
				if (state = start_sck + 0) then sensor_bits_in(7) <= SDO; end if;
				if (state = start_sck + 1) then sensor_bits_in(6) <= SDO; end if;
				if (state = start_sck + 2) then sensor_bits_in(5) <= SDO; end if;
				if (state = start_sck + 3) then sensor_bits_in(4) <= SDO; end if;
				if (state = start_sck + 4) then sensor_bits_in(3) <= SDO; end if;
				if (state = start_sck + 5) then sensor_bits_in(2) <= SDO; end if;
				if (state = start_sck + 6) then sensor_bits_in(1) <= SDO; end if;
				if (state = start_sck + 7) then sensor_bits_in(0) <= SDO; end if;	
			end if;
		end if;
		
		-- SCK is the serial clock that drives communication between the sensors and
		-- the logic chip. We generate SCK by first creating Serial Clock Early (SCKE),
		-- then delaying SCKE by one FCK period. The result is a 5-MHz clock that falls
		-- on the rising edges of TCK and rises on the falling edges of TCK. Outside
		if falling_edge(FCK) then 			
			if GYSEL then 
				if (state >= start_sck-1) and (state <= end_sck-1) then
					SCKE <= not SCKE;
				else
					SCKE <= true;
				end if;
				SCK <= to_std_logic(SCKE);
			else
				if (state >= start_sck) and (state <= end_sck) then
					SCK <= not SCK;
				else
					SCK <= '0';
				end if;
			end if;
		end if;

		-- CSA and CSG we unassert for now. They are negative-true, but we have not yet
		-- figure out how to make outputs negative-true, so we set them HI.
		NCSA <= to_std_logic(not CSA);
		NCSG <= to_std_logic(not CSG);
	end process;
	
-- The Sample Transmitter responds to Transmit Initiate (TXI) by turning on the 
-- radio-frequency oscillator, reading sixteen bits from one of the sensors and
-- transmitting the bits.
	Sample_Transmitter : process (TCK) is
		variable channel_num, set_num, completion_code : integer range 0 to 15; -- set number for data
		constant num_sync_bits : integer := 11; -- Num synchronizing bits at start.
		constant num_id_bits : integer := 4; -- Number of ID bits.
		constant num_start_bits : integer := 1; -- Num zero start bits.
		constant num_stop_bits : integer := 2; -- For state machine termination only.
		constant num_data_bits : integer := 16; -- Number of ADC data bits.
		constant num_xmit_bits : integer := -- Number of transmission bit periods.
			num_sync_bits + num_start_bits + num_id_bits + num_data_bits + num_id_bits; 
		constant st_idle : integer := 0; -- Idle state value.
		constant first_sync_bit : integer := 1; -- First transmit state.
		constant first_start_bit : integer := first_sync_bit + num_sync_bits;
		constant first_id_bit : integer := first_start_bit + num_start_bits;
		constant first_data_bit : integer := first_id_bit + num_id_bits;
		constant first_cc_bit : integer := first_data_bit + num_data_bits;
		constant st_done : integer := -- Final state of sample transmit machine.
			num_xmit_bits + num_stop_bits; 
		variable channel_bits : std_logic_vector(3 downto 0);
		variable cc_bits : std_logic_vector(3 downto 0);
		variable state, next_state : integer range 0 to 63 := 0; -- Stample Transmit State
		
	begin
		-- The channel number, set number, and comletion code are a function of the 
		-- device id and the channel offset, which we calculate here.
		channel_num := tx_channel mod 16;
		set_num := tx_channel / 16;
		completion_code := 15 - channel_num + set_num;
		channel_bits := std_logic_vector(to_unsigned(channel_num,4));
		cc_bits := std_logic_vector(to_unsigned(completion_code,4));
		
		if rising_edge(TCK) then
			-- We reset the state when TXI is false. Otherwise, we increment the
			-- state until it reaches st_done. At st_done, the state remains fixed 
			-- until not TXI. When we first enable sample transmission, the state is 
			-- zero. When TXI is asserted, the ring oscillator turns on, which starts
			-- TCK, the 5-MHz transmit clock. On the first rising edge of TCK, the 
			-- state becomes 1, and thereafter increments to st_done. Now TXD is true,
			-- which keeps the ring oscillator running while TXI becomes false. Once 
			-- false, the state switches back to zero, TXD is unasserted, and the 
			-- ring oscillator turns off, unless it is kept running by some other
			-- process. With no rising edges on TCK, the state remains zero. If rising
			-- edges on TCK continue because the ring oscillator is still running,
			-- the state will remain zero until TXI is asserted again.
			case state is
				when st_idle => 
					if TXI then
						next_state := 1;
					else
						next_state := 0;
					end if;
				
				when st_done =>
					if not TXI then
						next_state := st_idle;
					else
						next_state := st_done;
					end if;
				
				when others =>
					next_state := state + 1;
			end case;
		
			-- The data bit is the outgoing bit value for transmission of the sensor signal.
			TXB <= ((state >= 0) and (state < first_start_bit))
				or ((state = first_id_bit + 0) and (channel_bits(3) = '1'))
				or ((state = first_id_bit + 1) and (channel_bits(2) = '1'))
				or ((state = first_id_bit + 2) and (channel_bits(1) = '1'))
				or ((state = first_id_bit + 3) and (channel_bits(0) = '1'))
				or ((state = first_data_bit) and (xmit_bits(15) = '1'))
				or ((state = first_data_bit+1) and (xmit_bits(14) = '1'))
				or ((state = first_data_bit+2) and (xmit_bits(13) = '1'))
				or ((state = first_data_bit+3) and (xmit_bits(12) = '1'))
				or ((state = first_data_bit+4) and (xmit_bits(11) = '1'))
				or ((state = first_data_bit+5) and (xmit_bits(10) = '1'))
				or ((state = first_data_bit+6) and (xmit_bits(9) = '1'))
				or ((state = first_data_bit+7) and (xmit_bits(8) = '1'))
				or ((state = first_data_bit+8) and (xmit_bits(7) = '1'))
				or ((state = first_data_bit+9) and (xmit_bits(6) = '1'))
				or ((state = first_data_bit+10) and (xmit_bits(5) = '1'))
				or ((state = first_data_bit+11) and (xmit_bits(4) = '1'))
				or ((state = first_data_bit+12) and (xmit_bits(3) = '1'))
				or ((state = first_data_bit+13) and (xmit_bits(2) = '1'))
				or ((state = first_data_bit+14) and (xmit_bits(1) = '1'))
				or ((state = first_data_bit+15) and (xmit_bits(0) = '1'))
				or ((state = first_cc_bit + 0) and (cc_bits(3) = '1'))
				or ((state = first_cc_bit + 1) and (cc_bits(2) = '1'))
				or ((state = first_cc_bit + 2) and (cc_bits(1) = '1'))
				or ((state = first_cc_bit + 3) and (cc_bits(0) = '1'));
				
			-- TXD indicates to other processes that the transmission is complete, while
			TXD <= (state = st_done);

			-- TXA indicates that a transmission is on-going.
			TXA <= (state /= st_idle) and (state /= st_done);
			
			-- Assert the next state value.
			state := next_state;
		end if;
	end process;

-- With XEN we enable the VCO.
	XEN <= to_std_logic(TXA);
			
-- The Frequency Modulation process takes the transmit bit values provided by
-- the Sample Transmitter, turns them into a sequence of rising and falling
-- edges so as to balance the ratio of HI and LO, and modulates the transmit DAC
-- output (xdac) between the HI and LO frequency values. These values are turned
-- into analog voltages on the TUNE input of the radio frequency oscillator, and
-- so modulate the frequency of the transmission.
	Frequency_Modulation : process is
	begin
		-- Frequency modulation runs off the 10-MHz FCK clock. This clock is
		-- synchronous with TCK. It presents a rising edge over 10 ns after 
		-- both the rising and falling edges of TCK. Thus, when we see a
		-- rising edge on FCK, the value of TCK and TXB are both established.
		wait until (FCK = '1');
	
		-- When we are not transmitting RF power, we set the DAC output to
		-- zero so as to eliminate current consumption by the DAC resistors.
		if not TXA then
			xdac <= (others => '0');
			FHI <= false;
			
		-- If TXB is asserted, we want the modulation frequency to go from low
		-- to high on the falling edge of TCK. When TXB is unasserted, we want
		-- the modulation frequency to go from high to low on the falling edge of
		-- TCK.
		elsif (TXB xor (TCK = '1')) then
			xdac <= std_logic_vector(to_unsigned(frequency_low + frequency_step,5));
			FHI <= true;
		else
			xdac <= std_logic_vector(to_unsigned(frequency_low,5));
			FHI <= false;
		end if;
	end process;
		
-- Test Point One appears on P3-1 after the programming connector has been removed.
	TP1 <= to_std_logic(FHI);
	
-- Test Point Two appears on P3-2 after the programming connector has been removed.
	TP2 <= RESET;
	
-- Test Point Three appears on P1-6 after the programming connector is removed.
	TP3 <= tp_reg(0);

-- Test point Four appears on P1-8 after the programming connector is removed. 
-- Note that P1-8 is tied LO with 8 kOhm on the programming extension, so if 
-- this output is almost always HI, and the programming extension is still 
-- attached, quiescent current increases by 250 uA.
	TP4 <= tp_reg(1);
end behavior;