-- <pre> Blood Pressure Monitor (A3051B) Firmware, Toplevel Unit

-- Version 2.1 [21-NOV-24] Transmission, reading zero byte from sensor 
-- interface. Start converting sensor interface to I2C.

-- Version 2.2 [04-DEV-24] I2C interface consisting of six write-only
-- control registers and a read-only data byte.

library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity main is 
	port (
		RCK, -- Reference Clock
		IDY -- Interrupt/Data Ready
		: in std_logic; 
		SCL, -- Serial Clock Output
		SDA -- Serial Data Access	
		: inout std_logic;
		XEN, -- Transmit Enable, for data transmission
		TP1, -- Test Point One, available on P3-1
		TP2, -- Test Point Two, available on P3-2
		SA0  -- LSB of sensor address
		: out std_logic;
		xdac -- Transmit DAC Output, to set data transmit frequency
		: out std_logic_vector(4 downto 0));
		
-- Configuration of OSR8 CPU.
	constant prog_addr_len : integer := 12;
	constant cpu_addr_len : integer := 11;
	constant start_pc : integer := 0;
	constant interrupt_pc : integer := 3;

-- Configuration of peripherals.
	constant ram_addr_len : integer := 10;
	
-- Memory map sizes and base addresses in units of 512 bytes.
	constant ram_base : integer := 0;
	constant ram_range : integer := 2;
	constant ctrl_base : integer := 3;
	constant ctrl_range : integer := 1;	

-- Control space locations, offset from control space base address.
	constant mmu_i2c00 : integer := 16#00#; -- i2c SDA=0 SCL=0 (Write)
	constant mmu_i2c01 : integer := 16#01#; -- i2c SDA=0 SCL=1 (Write)
	constant mmu_i2cA0 : integer := 16#02#; -- i2c SDA=A SCL=0 (Write)
	constant mmu_i2cA1 : integer := 16#03#; -- i2c SDA=A SCL=1 (Write)
	constant mmu_i2cZ0 : integer := 16#04#; -- i2c SDA=Z SCL=0 (Write)
	constant mmu_i2cZ1 : integer := 16#05#; -- i2c SDA=Z SCL=1 (Write)
	constant mmu_i2cMR : integer := 16#06#; -- i2C Most Recent Eight Bits (Read)
	constant mmu_sr    : integer := 16#0F#; -- Status Register (Read)
	constant mmu_irqb  : integer := 16#10#; -- Interrupt Request Bits (Read)
	constant mmu_imsk  : integer := 16#12#; -- Interrupt Mask Bits (Read/Write)
	constant mmu_irst  : integer := 16#14#; -- Interrupt Reset Bits (Write)
	constant mmu_itp   : integer := 16#18#; -- Interrupt Timer Period (Read/Write)
	constant mmu_rst   : integer := 16#19#; -- System Reset (Write)
	constant mmu_xhb   : integer := 16#20#; -- Transmit HI Byte (Write)
	constant mmu_xlb   : integer := 16#21#; -- Transmit LO Byte (Write)
	constant mmu_xcn   : integer := 16#22#; -- Transmit Channel Number (Write)
	constant mmu_xcr   : integer := 16#24#; -- Transmit Control Register (Write)
	constant mmu_xfc   : integer := 16#26#; -- Transmit Frequency Calibration (Write)
	constant mmu_etc   : integer := 16#30#; -- Enable Transmit Clock (Write)
	constant mmu_tcf   : integer := 16#32#; -- Transmit Clock Frequency (Read)
	constant mmu_tcd   : integer := 16#34#; -- Transmit Clock Divider (Write)
	constant mmu_bcc   : integer := 16#36#; -- Boost CPU Clock (Write)
	constant mmu_dfr   : integer := 16#38#; -- Diagnostic Flag Register (Read/Write)
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
	
-- Clock Generation
	signal TCK, FCK, CK : std_logic;
	attribute syn_keep of TCK, FCK, CK : signal is true;
	attribute nomerge of TCK, FCK, CK : signal is "";  

-- Sensor Readout
	signal i2c_in : std_logic_vector(7 downto 0); -- I2C Serial Byte
	
-- Message Transmission.
	signal TXI, -- Transmit Initiate
		TXA, -- Transmit Active
		TXB, -- Transmit Bit
		FHI -- Frequency High
		: boolean := false;
	attribute syn_keep of TXI, TXA : signal is true;
	attribute nomerge of TXI, TXA : signal is "";  
	signal xmit_bits -- Sixteen bits to be transmitted as a message.
		: std_logic_vector(15 downto 0) := (others => '0');
	signal tx_channel : integer range 0 to 255 := 1; -- Transmit channel number
	signal frequency_low : integer range 0 to 31 := 6; -- Low frequency for transmission
	constant frequency_step : integer := 1; -- High minus low frequency
		
-- Clock Calibrator
	signal ENTCK : boolean; -- Enable the Transmit Clock
	signal tck_frequency : integer range 0 to 255; -- Transmit Clock Counter
	constant default_tck_divisor : integer := 11;
	signal tck_divisor : integer range 0 to 15 := default_tck_divisor;
	
-- Boost Controller
	signal BOOST : boolean;
	
-- CPU-Writeable Diagnostic Flags
	signal df_reg : std_logic_vector(7 downto 0) := (others => '0');
	
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
	
	
-- Ring Oscillator. This oscillator turns on when the microprocessor asserts
-- Enable Transmit Clock (ENTCK). The transmit clock must be running during a
-- sample transmission in order for the timing of the transmission to be correct.
-- The transmit clock should be turned on during a sensor access as well, so that
-- the sensor access will be quick and the sensor can power down again sooner. The
-- ring oscillator will produce FCK at 10 MHz.
	Fast_CK : entity ring_oscillator port map (
		ENABLE => to_std_logic(ENTCK), 
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
	
-- The processor itself. We instantiate the OSR8 microprocessor entity.
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
-- Sensor Interface , Sample Transmitter, Random Access Memory, and 
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
		
		-- These signals develop after the CPU asserts a new address
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
				when mmu_sr => 
					cpu_data_in(0) <= SDA; -- Sensor Data Access
					cpu_data_in(1) <= IDY; -- Sensor Interrupt Data Reacy
					cpu_data_in(2) <= SCL; -- Sensor Clock
					cpu_data_in(3) <= to_std_logic(ENTCK);  -- Transmit Clock Enabled
					cpu_data_in(4) <= to_std_logic(BOOST); -- CPU Boost Enabled
				when mmu_irqb => cpu_data_in <= int_bits;
				when mmu_imsk => cpu_data_in <= int_mask;
				when mmu_itp => cpu_data_in <= int_period;
				when mmu_tcf =>
					cpu_data_in <= std_logic_vector(to_unsigned(tck_frequency,8));
				when mmu_dfr => cpu_data_in <= df_reg;
				when mmu_i2cMR => cpu_data_in <= i2c_in;
				-- This others statement stabilizes the code. It also has the
				-- effect of making the non-existent register read return a zero.
				when others => cpu_data_in <= (others => '0');
				end case;
			end if;
		end case;
		
		-- We use RESET to clear some registers and signals, but not all. We do not clear the
		-- software reset signal, SWRST, on RESET, since we want SWRST to assert RESET for one
		-- CK period. After a reset, the cpu address will not select the SWRST location, so
		-- SWRST will be cleared on the next falling edge of CK.
		if (RESET = '1') then
			SDA <= 'Z';
			SCL <= '1';
			TXI <= false;
			ENTCK <= false;
			BOOST <= false;
			tck_divisor <= default_tck_divisor;
			int_period <= (others => '0');
			int_mask <= (others => '0');
			i2c_in <= (others => '0');
		-- We use the falling edge of CK to write to registers. Some signals we assert 
		-- only for one CK period, and these we assert as false by default.
		elsif falling_edge(CK) then
			SWRST <= false;
			TXI <= false;
			int_rst <= (others => '0');
			int_set <= (others => '0');
			if CPUDS and CPUWR then 
				if (top_bits >= ctrl_base) 
						and (top_bits <= ctrl_base+ctrl_range-1) then
					case bottom_bits is
					when mmu_i2c00 => 
						SDA <= '0';
						SCL <= '0';
					when mmu_i2c01 =>
						SDA <= '0';
						SCL <= '1';
					when mmu_i2cA0 => 
						if (cpu_data_out(7) = '0') then
							SDA <= '0';
						else
							SDA <= 'Z';
						end if;
						SCL <= '0';
					when mmu_i2cA1 =>
						if (cpu_data_out(7) = '0') then
							SDA <= '0';
						else
							SDA <= 'Z';
						end if;
						SCL <= '1';
					when mmu_i2cZ0 => 
						SDA <= 'Z';
						SCL <= '0';
					when mmu_i2cZ1 => 
						SDA <= 'Z';
						SCL <= '1';
						i2c_in(7 downto 1) <= i2c_in(6 downto 0);
						i2c_in(0) <= SDA;
					when mmu_xlb => xmit_bits(7 downto 0) <= cpu_data_out;
					when mmu_xhb => xmit_bits(15 downto 8) <= cpu_data_out;
					when mmu_xcn => tx_channel <= to_integer(unsigned(cpu_data_out));
					when mmu_xcr => TXI <= true;
					when mmu_xfc => frequency_low <= to_integer(unsigned(cpu_data_out));
					when mmu_imsk => int_mask <= cpu_data_out;
					when mmu_itp => int_period <= cpu_data_out;
					when mmu_irst => int_rst <= cpu_data_out;
					when mmu_rst => SWRST <= (cpu_data_out(0) = '1');
					when mmu_etc => ENTCK <= (cpu_data_out(0) = '1');
					when mmu_tcd => tck_divisor <= to_integer(unsigned(cpu_data_out));
					when mmu_bcc => BOOST <= (cpu_data_out(0) = '1');
					when mmu_dfr => df_reg <= cpu_data_out;
					-- The following others statement stabilizes code.
					when others => df_reg <= df_reg;
					end case;
				end if;
			end if;
		end if;
	end process;
	
	-- The Clock Calibrator counts cycles of TCK for one half-period of RCK after the
	-- assertion of Enable Transmit Clock (ENTCK) and makes the count available to the 
	-- CPU in the tck_frequency register. If TCK is 5.00 MHz and RCK is 32.768 kHz, 
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
	
		-- Eight-bit repeating timer. It nevers stops, and it generates an interrupt
		-- every time it reaches zero.
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
					
			-- The timer interrupt is set when the counter reaches zero.
			-- We reset when we write of 1 to int_rst(0).
			INTCTRZ <= (counter = 0);
			if (int_rst(0) = '1') then
				int_bits(0) <= '0';				
			elsif ((counter = 0) and (not INTCTRZ)) then
				int_bits(0) <= '1';
			end if;
			
			-- Disable all other interrupts.
			for i in 1 to 7 loop
				int_bits(i) <= '0';
			end loop;
			
			-- We generate an interrupt if any one interrupt bit is 
			-- set and unmasked.
			CPUIRQ <= (int_bits and int_mask) /= "00000000";
		end if;
	end process;
	
-- The Sample Transmitter responds to Transmit Initiate (TXI) by turning on the 
-- radio-frequency oscillator, reading sixteen bits from one of the sensors and
-- transmitting the bits.
	Sample_Transmitter : process (TCK) is
		variable channel_num : integer range 0 to 15; -- channel number
		variable set_num : integer range 0 to 15; -- set number 
		variable completion_code : integer range 0 to 15; -- completion code
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
		
-- Sensor Address Zero we hold LO to indicate that the sensor address is 1011101b.
	SA0 <= '0';
		
-- Test Point One appears on P3-1 after the programming connector has been removed. 
	TP1 <= df_reg(1);
	
-- Test Point Two appears on P3-2 after the programming connector has been removed.
	TP2 <= to_std_logic(FHI);

end behavior;