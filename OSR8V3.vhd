-- Open-Source Reconfigurable Eight-Bit (OSR8) Central Processing Unit (CPU)
--
-- Copyright (C) 2020-2024 Kevan Hashemi, Open Source Instruments Inc.
--
-- This program is free software; you can redistribute it and/orpr
-- modify it under the terms of the GNU General Public License
-- as published by the Free Software Foundation; either version 2
-- of the License, or (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program; if not, write to the Free Software
-- Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

-- [11-APR-22] Working on P3041. Introduce constants that calculate correct 
-- vector sizes in the processor so as to adapt automatically to new program 
-- and cpu memory sizes. Improve implementation of stack overflow flag, which 
-- now remains asserted until RESET.

-- [18-JUN-22] Remove stack overflow flag. Stack pointer always resets to
-- zero, so CPU must have RAM at address zero to use as initial stack during
-- initialization. The initial stack will allow the CPU program to load the
-- stack pointer with a new value.

-- [06-DEC-24] Searching for boot bug, eliminate prog_addr in code, use
-- prog_addr everywhere.

-- [13-DEC-24] Add others clauses to all cases in the hope that this will 
-- stabilized the compile process. Each others clause is a repeat of some
-- or all of a previous clause.

library ieee;  
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- The CPU requires separate program and process memory. We set the size of both
-- memories by typing in their top address bit index in the declaration of the
-- OSR8 entity. Process and program memory can be anything from 1 kByte to 64 kByte.
-- All our efforts to use generic constants to define the memory sizes have failed
-- due to what we believe is a compiler error, so we cannot re-configure the OSR8
-- from the instantiating entity. 

-- Both memories must be clocked on the falling edge of CK. The CPU increments the
-- program counter on the rising edge of CK and expects the next instruction byte
-- to be present for decoding well before the next rising edge, which in practice
-- means we must fetch it on the falling edge of CK. The WR and DS control signals
-- along with the cpu_addr and cpu_data_out will be asserted on the rising edge of
-- CK and persist until the next rising edge, so they are always valid on the
-- falling edge of CK for the data memory to use. On a read cycle, the CPU expects
-- the data to be ready before the next rising edge of CK after it asserts DS. On
-- a write cycle, the CPU expects the write to take place on the falling edge of
-- CK after it asserts DS and WR. 

-- The IRQ signal is for an interrupt request. The external memory management unit (MMU) 
-- must provide a way for the CPU to clear the IRQ signal so it can get back to its main
-- program.

-- The RESET signal will put the CPU back to program location zero. Execution begins
-- at location start_pc, where we expect a three-byte un-conditional jump instruction. 
-- Interrupt execution begins at interrupt_pc, where we expect another jp_nn instruction. 

-- The stack pointer is upward-going, initialized to zero. Byte ordering for pointers is 
-- big-endian. The most significant, or HI byte is at the lower address and the lease 
-- significant, or LO byte, is at the higher address. The HI byte of the pointer will 
-- contain from one to eight neccessary bits, depending upon the memory available to the CPU. 

entity OSR8_CPU is 
	generic (
		prog_addr_len : integer := 12;
		cpu_addr_len : integer := 11;
		start_pc : integer := 0;
		interrupt_pc : integer := 3	);
	port (
		prog_data : in std_logic_vector(7 downto 0); -- Program Data
		prog_addr : inout std_logic_vector(prog_addr_len-1 downto 0); -- Program Address
		cpu_data_out : out std_logic_vector(7 downto 0); -- Outgoing CPU Data Bus
		cpu_data_in : in std_logic_vector(7 downto 0); -- Incoming CPU Data Bus
		cpu_addr : out std_logic_vector(cpu_addr_len-1 downto 0); -- Outgoing CPU Address Bus
		WR : out boolean; -- Write Cycle
		DS : out boolean; -- Data Strobe
		IRQ : in boolean; -- Interrupt Request
		SIG : out std_logic_vector(2 downto 0); -- Signals for Debugging
		RESET : in std_logic; -- Hard Reset
		CK : in std_logic -- The clock, duty cycle 50%.
	); 

-- Program location constants in bytes.
	constant pa_top : integer := prog_addr_len-1;
	constant ca_top : integer := cpu_addr_len-1;
end;

architecture behavior of OSR8_CPU is 

-- Definition of operation codes. We have the constant name we will be using
-- in the VHDL definition of the CPU to refer to an operation, followed by 
-- the hexadecimal byte value by which this operation will be invoked as a 
-- program is being executed, and the instruction pneumonic that shows what
-- the instruction does, and how it is to be written in the OSR8 assembly 
-- language. We use "n" for an eight-bit constant, "nn" for a sixteen-bit
-- constant, "(nn)" for the location with address nn, "(IX)" for the location
-- with address given by index register IX, "jp nz" for jump if not zero,
-- "ld" for "load", and so on, as in Z80 assembly language. The set of OSR8
-- instructions is less numberous than Z80, but neither is it a subset of
-- the Z80 instruction set. For example, Z80 does not provide "push A",
-- while OSR8 does not provide "push BC".

	constant nop      : integer := 16#00#; -- nop
	constant jp_nn    : integer := 16#01#; -- jp nn
	constant jp_nz_nn : integer := 16#02#; -- jp nz,nn
	constant jp_z_nn  : integer := 16#03#; -- jp z,nn
	constant jp_nc_nn : integer := 16#04#; -- jp nc,nn
	constant jp_c_nn  : integer := 16#05#; -- jp c,nn
	constant jp_np_nn : integer := 16#06#; -- jp np,nn
	constant jp_p_nn  : integer := 16#07#; -- jp p,nn
	constant call_nn  : integer := 16#08#; -- call nn
	constant sw_int   : integer := 16#09#; -- int
	constant ret_cll  : integer := 16#0A#; -- ret
	constant ret_int  : integer := 16#0B#; -- rti
	constant cpu_wt   : integer := 16#0C#; -- wait
	constant clr_iflg : integer := 16#0D#; -- clri
	constant set_iflg : integer := 16#0E#; -- seti
	
	constant ld_A_n   : integer := 16#10#; -- ld A,n
	constant ld_IX_nn : integer := 16#11#; -- ld IX,nn
	constant ld_IY_nn : integer := 16#12#; -- ld IY,nn
	constant ld_HL_nn : integer := 16#13#; -- ld HL,nn
	constant ld_A_mm  : integer := 16#14#; -- ld A,(nn)
	constant ld_mm_A  : integer := 16#15#; -- ld (nn),A
	constant ld_A_ix  : integer := 16#16#; -- ld A,(IX)
	constant ld_A_iy  : integer := 16#17#; -- ld A,(IY)
	constant ld_ix_A  : integer := 16#18#; -- ld (IX),A
	constant ld_iy_A  : integer := 16#19#; -- ld (IY),A
	constant ld_HL_SP : integer := 16#1A#; -- ld HL,SP
	constant ld_SP_HL : integer := 16#1B#; -- ld SP,HL
	constant ld_HL_PC : integer := 16#1C#; -- ld HL,PC
	constant ld_PC_HL : integer := 16#1D#; -- ld PC,HL	
			
	constant push_A   : integer := 16#20#; -- push A
	constant push_B   : integer := 16#21#; -- push B
	constant push_C   : integer := 16#22#; -- push C
	constant push_D   : integer := 16#23#; -- push D
	constant push_E   : integer := 16#24#; -- push E
	constant push_H   : integer := 16#25#; -- push H
	constant push_L   : integer := 16#26#; -- push L
	constant push_F   : integer := 16#27#; -- push F
	constant push_IX  : integer := 16#28#; -- push IX
	constant push_IY  : integer := 16#29#; -- push IY
	
	constant pop_A    : integer := 16#30#; -- pop A
	constant pop_B    : integer := 16#31#; -- pop B
	constant pop_C    : integer := 16#32#; -- pop C
	constant pop_D    : integer := 16#33#; -- pop D
	constant pop_E    : integer := 16#34#; -- pop E
	constant pop_H    : integer := 16#35#; -- pop H
	constant pop_L    : integer := 16#36#; -- pop L
	constant pop_F    : integer := 16#37#; -- pop F
	constant pop_IX   : integer := 16#38#; -- pop IX
	constant pop_IY   : integer := 16#39#; -- pop IY

	constant add_A_B  : integer := 16#40#; -- add A,B
	constant add_A_n  : integer := 16#41#; -- add A,n
	constant adc_A_B  : integer := 16#42#; -- adc A,B
	constant adc_A_n  : integer := 16#43#; -- adc A,n
	constant sub_A_B  : integer := 16#44#; -- sub A,B
	constant sub_A_n  : integer := 16#45#; -- sub A,n
	constant sbc_A_B  : integer := 16#46#; -- sbc A,B
	constant sbc_A_n  : integer := 16#47#; -- sbc A,n
	constant clr_aflg : integer := 16#4F#; -- clrf
	
	constant inc_A    : integer := 16#50#; -- inc A
	constant inc_B    : integer := 16#51#; -- inc B
	constant inc_C    : integer := 16#52#; -- inc C
	constant inc_D    : integer := 16#53#; -- inc D
	constant inc_E    : integer := 16#54#; -- inc E
	constant inc_H    : integer := 16#55#; -- inc H
	constant inc_L    : integer := 16#56#; -- inc L
	constant inc_SP   : integer := 16#57#; -- inc SP
	constant inc_IX   : integer := 16#59#; -- inc IX
	constant inc_IY   : integer := 16#5A#; -- inc IY

	constant dec_A    : integer := 16#60#; -- dec A
	constant dec_B    : integer := 16#61#; -- dec B
	constant dec_C    : integer := 16#62#; -- dec C
	constant dec_D    : integer := 16#63#; -- dec D
	constant dec_E    : integer := 16#64#; -- dec E
	constant dec_H    : integer := 16#65#; -- dec H
	constant dec_L    : integer := 16#66#; -- dec L
	constant dly_A    : integer := 16#67#; -- dly A
	constant dec_SP   : integer := 16#68#; -- dec SP
	constant dec_IX   : integer := 16#69#; -- dec IX
	constant dec_IY   : integer := 16#6A#; -- dec IY
	
	constant and_A_B  : integer := 16#70#; -- and A,B
	constant and_A_n  : integer := 16#71#; -- and A,n
	constant or_A_B   : integer := 16#72#; -- or A,B
	constant or_A_n   : integer := 16#73#; -- or A,n
	constant xor_A_B  : integer := 16#74#; -- xor A,B
	constant xor_A_n  : integer := 16#75#; -- xor A,n
	
	constant rl_A     : integer := 16#78#; -- rl A
	constant rlc_A    : integer := 16#79#; -- rlc A
	constant rr_A     : integer := 16#7A#; -- rr A
	constant rrc_A    : integer := 16#7B#; -- rrc A
	constant sla_A    : integer := 16#7C#; -- sla A
	constant sra_A    : integer := 16#7D#; -- sra A
	constant srl_A    : integer := 16#7E#; -- srl A
	
-- Attributes to guide the compiler.
	attribute syn_keep : boolean;
	attribute nomerge : string;

-- Arithmetic Logic Unit signals and constants
	signal alu_out : integer range 0 to 255;
	attribute syn_keep of alu_out : signal is true;
	attribute nomerge of alu_out : signal is "";
	signal alu_in_x, alu_in_y : integer range 0 to 255;
	signal alu_cin, alu_cout, alu_z, alu_v : boolean;
	signal alu_ctrl : integer range 0 to 15;
	constant alu_cmd_add  : integer := 0; -- Add X and Y
	constant alu_cmd_sub  : integer := 1; -- Subtract Y from X
	constant alu_cmd_and  : integer := 2; -- Bitwise AND of X and Y
	constant alu_cmd_xor  : integer := 3; -- Bitwise XOR of X and Y
	constant alu_cmd_or   : integer := 4; -- Bitwise OR of X and Y
	constant alu_cmd_rl   : integer := 5; -- Rotate Left of X
	constant alu_cmd_rlc  : integer := 6; -- Rotate Left Circuiar of X
	constant alu_cmd_rr   : integer := 7; -- Rotate Right of X
	constant alu_cmd_rrc  : integer := 8; -- Rotate Right Circular of X
	constant alu_cmd_sla  : integer := 9; -- Shift Left Arithmetic of X
	constant alu_cmd_sra  : integer := 10; -- Shift Right Arithmetic of X
	constant alu_cmd_srl  : integer := 11; -- Shift Right Logical of X
	constant alu_cmd_inc  : integer := 12; -- Incrment X
	constant alu_cmd_dec  : integer := 13; -- Decrement X
	constant alu_cmd_nop  : integer := 15; -- Direct X to A

-- The Accumulator, or Register A, in which we get the result of eight-bit
-- arithmetic operations, logical operations, and shifts and rotations. 
-- When such an operation requires a second operand, we can use either 
-- Register B or a one-byte constant.
	signal reg_A : integer range 0 to 255;

-- Register B will operate with the accumulator for eight-bit operatins that
-- require a second operand, such as addition, subtraction, and logical AND.
	signal reg_B  : integer range 0 to 255;

-- General-purpose eight-bit registers C, D, E, H, and L. The last two have 
-- one special function: to act as a loading platform for the stack pointer.
	signal reg_C, reg_D, reg_E, reg_H, reg_L : integer range 0 to 255;
	
-- The flags are bits in the Flag Register, but we work with them as separate
-- boolean signals so as to simplify our code. We have the Zero, Carry, Sign,
-- and Interrupt flags.
	signal flag_Z, flag_C, flag_S, flag_I : boolean;

-- The index registers IX and IY are intended for use as pointers to bytes
-- in memory. We can increment them and push them onto the stack, as well
-- as load them with a constant.
	signal reg_IX, reg_IY : std_logic_vector(ca_top downto 0);

-- The Stack Pointer (SP) we use to manage an upward-growing stack. The
-- Stack Pointer points to the top of the stack, which is the byte most
-- recently pushed onto the stack, and at the highest address of all the 
-- bytes on the stack. When we push a byte onto the stack, we increment
-- the stack pointer, then perform the write. When we pop from the stack, we 
-- read from the stack and then decrement the stack pointer.
	signal reg_SP : std_logic_vector(ca_top downto 0);
			
-- Functions and Procedures	
	function to_std_logic (v: boolean) return std_ulogic is
	begin if v then return('1'); else return('0'); end if; end function;

begin 

-- The Arithmetic Logic Unit provides an eight-bit adder-subtractor with carry 
-- in and carry out, as well as logical operations AND, OR, and XOR.
	CPU_ALU : process (alu_ctrl,alu_cin,alu_in_x,alu_in_y) is
	variable result : integer range 0 to 511;
	variable result_vec : std_logic_vector(8 downto 0);
	variable X_vec : std_logic_vector(8 downto 0);
	begin 
		X_vec(7 downto 0) := std_logic_vector(to_unsigned(alu_in_x,8));
		X_vec(8) := to_std_logic(alu_cin);
		
		case alu_ctrl is
		
		-- Add X and Y with Carry
		when alu_cmd_add =>
			if alu_cin then result := alu_in_x + alu_in_y + 1;
			else result := alu_in_x + alu_in_y; end if;
			
		-- Subtract Y from X with Borrow
		when alu_cmd_sub =>
			if alu_cin then result := alu_in_x - alu_in_y - 1;
			else result := alu_in_x - alu_in_y; end if;
			
		-- Bit-Wise Exclusive OR of A and B
		when alu_cmd_xor =>
			result := to_integer(unsigned(
					std_logic_vector(to_unsigned(alu_in_x,8))
					xor std_logic_vector(to_unsigned(alu_in_y,8)) ));
			
		-- Bit-Wise OR of A and B
		when alu_cmd_or =>
			result := to_integer(unsigned(
					std_logic_vector(to_unsigned(alu_in_x,8))
					or std_logic_vector(to_unsigned(alu_in_y,8)) ));
					
		-- Bit-Wise Logical AND of X and Y
		when alu_cmd_and =>
			result := to_integer(unsigned(
					std_logic_vector(to_unsigned(alu_in_x,8))
					and std_logic_vector(to_unsigned(alu_in_y,8)) ));
					
		 -- Rotate Left of X	
		when alu_cmd_rl  => 
			result_vec(8 downto 1) := X_vec(7 downto 0);
			result_vec(0) := X_vec(8);	
			result := to_integer(unsigned(result_vec));
			
		 -- Rotate Left Circuiar of X
		when alu_cmd_rlc => 
			result_vec(7 downto 1) := X_vec(6 downto 0);
			result_vec(0) := X_vec(7);	
			result_vec(8) := x_vec(7);
			result := to_integer(unsigned(result_vec));
			
		-- Rotate Right of X
		when alu_cmd_rr  =>  
			result_vec(7 downto 0) := X_vec(8 downto 1);
			result_vec(8) := X_vec(0);		
			result := to_integer(unsigned(result_vec));
			
		-- Rotate Right Circular of X
		when alu_cmd_rrc =>  
			result_vec(6 downto 0) := X_vec(7 downto 1);
			result_vec(7) := X_vec(0);		
			result_vec(8) := X_vec(0);		
			result := to_integer(unsigned(result_vec));
			
		-- Shift Left Arithmetic of X
		when alu_cmd_sla =>  
			result_vec(8 downto 1) := X_vec(7 downto 0);
			result_vec(0) :='0';	
			result := to_integer(unsigned(result_vec));
			
		-- Shift Right Arithmetic of X
		when alu_cmd_sra =>  
			result_vec(6 downto 0) := X_vec(7 downto 1);
			result_vec(7) := X_vec(7);		
			result_vec(8) := X_vec(0);		
			result := to_integer(unsigned(result_vec));
			
		-- Shift Right Logical of X
		when alu_cmd_srl => 
			result_vec(6 downto 0) := X_vec(7 downto 1);
			result_vec(7) := '0';		
			result_vec(8) := X_vec(0);		
			result := to_integer(unsigned(result_vec));
		
		-- An others clause identical to another instruction.
		when others =>
			result_vec(6 downto 0) := X_vec(7 downto 1);
			result_vec(7) := '0';		
			result_vec(8) := X_vec(0);		
			result := to_integer(unsigned(result_vec));
			
		end case;
		
		alu_out <= (result rem 256);
		alu_cout <= (result >= 256);
	end process;

-- The Central Processing Unit reads and writes from an external memory space with
-- its address bus, cpu_addr, and its eight-bit data busses, cpu_data_in and 
-- cpu_data_out, and its control signals CPU Data Strobe (DS), and CPU Write (WR). 
-- Its program memory is separate and private. The CPU instruction set is sufficient 
-- for all integer arithmetic, register exchanges, recursive subroutines, concurrent 
-- processes, and efficient block moves. 
	CPU : process (CK,RESET) is
	
		-- The next_r variables we use to set up the value of A that will be asserted 
		-- on the next rising edge of CK.
		variable next_A, next_B, next_C, next_D, 
			next_E, next_H, next_L : integer range 0 to 255;
		
		-- The next values of the flag bits.
		variable next_flag_Z, next_flag_C, next_flag_S, next_flag_I : boolean;
		
		-- The next values of the pointer registers.
		variable next_IX, next_IY, next_SP : std_logic_vector(ca_top downto 0);
				
		-- Variables for the microcode state machine, and constants for the state
		-- names.
		variable opcode, opcode_now : integer range 0 to 127;
		variable first_operand, second_operand : integer range 0 to 255;
		variable state, next_state : integer range 0 to 255;
		
		-- The state values. We use single-bit values to simplify the state
		-- machine logic. We get faster and more compact logic even though we
		-- use eight bits for the state rather than three.
		constant read_opcode : integer := 1;
		constant read_first_operand : integer := 2;
		constant read_second_operand : integer := 4;
		constant read_first_byte : integer := 8;
		constant read_second_byte : integer := 16;
		constant write_second_byte : integer := 32;
		constant read_second_opcode : integer := 64;
		constant incr_pc : integer := 128;
			
		-- We have next_pa to set up the next program counter value.
		variable next_pa : std_logic_vector(pa_top downto 0);
			
		-- We uset the jump variable to determine if any of the various jump
		-- conditions have been satisfied.
		variable jump : boolean;		
						
	begin
		-- Reset the cpu state and program counter until we enter standby mode.
		if (RESET ='1') then 
			state := read_opcode;
			prog_addr <= std_logic_vector(to_unsigned(start_pc,prog_addr_len)); 
			reg_SP <= (others => '0'); 
			flag_Z <= false;
			flag_C <= false;
			flag_S <= false;
			flag_I <= false;
			WR <= false;
			DS <= false;
		
		-- Otherwise we repond to the rising edge of CK.
		elsif rising_edge(CK) then
		
			-- Define default next values.
			next_state := read_opcode;
			next_pa := std_logic_vector(unsigned(prog_addr)+1);
			next_A := reg_A;
			next_B := reg_B;
			next_C := reg_C;
			next_D := reg_D;
			next_E := reg_E;
			next_H := reg_H;
			next_L := reg_L;
			next_IX := reg_IX;
			next_IY := reg_IY;
			next_SP := reg_SP;
			next_flag_C := flag_C;
			next_flag_Z := flag_Z;
			next_flag_S := flag_S;
			next_flag_I := flag_I;
			WR <= false;
			DS <= false;
			
			-- Read the first byte of the instruction. If the instruction operates only 
			-- upon register, with no constants involved, it will execute here in one state. 
			-- If we have an interrupt request, we deal with that now, before we start 
			-- decoding this instruction, by executing a non-maskable interrupt instruction 
			-- instead of the instruction pointed to by the program counter.
			if (state = read_opcode) then
			
				-- We override the opcode provided by the program data when we have an
				-- interrupt request and we are not currently servicing an interrupt, this
				-- latter condition being indicated by flag_I.
				if IRQ and (not flag_I) then
					opcode := sw_int;
				else
					opcode := to_integer(unsigned(prog_data));
				end if;			
				
				-- Decode the instruction.
				case opcode is
				
				-- The no-operation instruction. Clock Cycles = 1.
				when nop => 
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- The wait operation stays in this one place until we receive an
				-- interrupt. It clears the interrupt flag. Clock Cycles = 1.
				when cpu_wt =>
					next_flag_I := false;
					next_pa := prog_addr;
					next_state := read_opcode;
					
				-- The delay instruction decrements A until it reaches zero, then moves on.
				-- It sets no flags because its end point is always a zero value in A.
				when dly_A => 
					next_A := alu_out;
					if (reg_A = 0) then 
						next_pa := std_logic_vector(unsigned(prog_addr)+1);
					else
						next_pa := prog_addr; 
					end if;
					next_state := read_opcode;
					
				-- We handle an interrupt by freezing the program counter and 
				-- pushing its value onto the stack before jumping to the hard-wired
				-- interrupt handler address. We begin by pushing the HI byte of 
				-- the program counter onto the stack. We set flag_I when IRQ is 
				-- asserted, so that we can begin normal instruction execution once 
				-- we have jumped to the interrupt routine: flag_I blocks the execution
				-- of interrupts. The interrupt routine must clear the Interrupt Request
				-- signal (IRQ), or else the main process will be interrupted again upon 
				-- return. The interrupt routine should use the return from interrupt 
				-- instruction (ret_int, pneumonic "rti"), which will clear flag_I upon 
				-- return and execute the interrupted instruction, in the case of an
				-- IRQ-provoked interrupt, or execute the next instruction, in the case
				-- of an interrupt provoked by an "int" instruction. Clock Cycles = 2.
				when sw_int =>
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					cpu_addr <= std_logic_vector(unsigned(reg_SP)+1);
					WR <= true;
					DS <= true;
					cpu_data_out <= (others => '0');
					cpu_data_out(pa_top-8 downto 0) <= prog_addr(pa_top downto 8);
					if IRQ then next_flag_I := true; end if;
					next_state := write_second_byte;
					next_pa := prog_addr;

				-- Return from subroutine, we pop the LO byte of the program counter 
				-- off the stack, then go on to pop the HI byte and load it into the
				-- program counter. Clock Cycles = 4 except for a return from interrupt
				-- initiated by IRQ, which takes only 3.
				when ret_cll| ret_int =>
					next_SP := std_logic_vector(unsigned(reg_SP)-1);
					cpu_addr <= reg_SP;
					WR <= false;
					DS <= true;
					next_state := read_first_byte;
					next_pa := prog_addr;
			
				-- Stack Pointer load to and from HL. We use H for the upper bits 
				-- of SP and L for the lower bits. Clock Cycles = 1.
				when ld_HL_SP =>
					next_H := to_integer(unsigned(reg_SP(ca_top downto 8)));
					next_L := to_integer(unsigned(reg_SP(7 downto 0)));
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when ld_SP_HL =>
					next_SP(ca_top downto 8) := 
						std_logic_vector(to_unsigned(reg_H,cpu_addr_len-8));
					next_SP(7 downto 0) := std_logic_vector(to_unsigned(reg_L,8));
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);

				-- Program Counter load to and from HL. We use H for the upper bits of 
				-- PC. These instructions allow us, in principle, to implement relative 
				-- jumps, by copying the program counter, adding to it, then loading the 
				-- program counter with the result, as in: ld HL,PC; push L; pop A; 
				-- add A,n; push A; pop L; push H; pop A; adc A,n; push A; pop H; 
				-- ld PC,HL; for a total of 16 clock cycles compared to 3 for an
				-- absolute, unconditional jump. Clock Cycles = 1.
				when ld_HL_PC =>
					next_H := to_integer(unsigned(prog_addr(pa_top downto 8)));
					next_L := to_integer(unsigned(prog_addr(7 downto 0)));
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when ld_PC_HL =>
					next_pa(pa_top downto 8) := 
						std_logic_vector(to_unsigned(reg_H,prog_addr_len-8));
					next_pa(7 downto 0) := std_logic_vector(to_unsigned(reg_L,8));
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Push operations for eight-bit registers. We immediately generate 
				-- the value SP-1 with our combinatorial decrementer. We drive SP+1 onto 
				-- the address lines on the next rising edge of the clock. On the same
				-- rising edge, SP will decrement to SP-1, and we drive the value we
				-- want to store in the stack onto the data lines. The value will be
				-- stored on the falling edge of the clock in the middle of the next
				-- clock period. By that time, the CPU will be processing the next
				-- instruction. The single-register push instruction, and its converse,
				-- the single-register pop instruction, together allow us to move any
				-- register into any other register, and indeed to re-arrange registers
				-- however we like. If we want to move B into C, C into D, and D into
				-- B we use: push B; push C; push D; pop B; pop D; pop C to perform
				-- the rotation in 9 clock cycles. Clock Cycles = 1.
				when push_A | push_B | push_C | push_D | push_E | push_H | push_L | push_F => 
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					cpu_addr <= std_logic_vector(unsigned(reg_SP)+1);
					WR <= true;
					DS <= true;
					case opcode is
						when push_A => cpu_data_out <= std_logic_vector(to_unsigned(reg_A,8));
						when push_B => cpu_data_out <= std_logic_vector(to_unsigned(reg_B,8));
						when push_C => cpu_data_out <= std_logic_vector(to_unsigned(reg_C,8));
						when push_D => cpu_data_out <= std_logic_vector(to_unsigned(reg_D,8));
						when push_E => cpu_data_out <= std_logic_vector(to_unsigned(reg_E,8));
						when push_H => cpu_data_out <= std_logic_vector(to_unsigned(reg_H,8));
						when push_L => cpu_data_out <= std_logic_vector(to_unsigned(reg_L,8));
						when push_F =>
							cpu_data_out <= (others => '0');
							cpu_data_out(7) <= to_std_logic(flag_I);
							cpu_data_out(2) <= to_std_logic(flag_C);
							cpu_data_out(1) <= to_std_logic(flag_S);
							cpu_data_out(0) <= to_std_logic(flag_Z);
					end case;
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				
				-- Push operations for index registers. We start by pushing the HI byte onto
				-- the stack, then we push the LO byte. Clock Cycles = 2.
				when push_IX | push_IY => 
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					cpu_addr <= std_logic_vector(unsigned(reg_SP)+1);
					WR <= true;
					DS <= true;
					cpu_data_out <= (others => '0');
					case opcode is
						when push_IX => 
							cpu_data_out(ca_top-8 downto 0) <= reg_IX(ca_top downto 8);
						when push_IY => 
							cpu_data_out(ca_top-8 downto 0) <= reg_IY(ca_top downto 8);
					end case;
					next_state := write_second_byte;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Pop instructions for eight-bit registers. We immediately generate SP+1
				-- at the output of our combinatorial incrementer. On the next rising 
				-- edge of the clock, we drive SP onto the address lines, and at the same
				-- time increment SP to SP+1. But is is SP that remains on the address lines,
				-- even as the stack pointer register itself increments. On that same rising 
				-- edge we assert DS and unassert WR. The stack will respond on the next 
				-- falling edge by driving a byte onto the data lines. We read the byte on 
				-- the rising edge after that, so we need another clock cycle to complete the 
				-- pop. We move to read_first_byte to perform the read. Clock Cycles = 2.
				when pop_A | pop_B | pop_C | pop_D | pop_E | pop_H | pop_L | pop_F =>
					next_SP := std_logic_vector(unsigned(reg_SP)-1);
					cpu_addr <= reg_SP;
					WR <= false;
					DS <= true;
					next_state := read_first_byte;	
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Pop operations for index registers. We begin with an eight-bit pop of the
				-- LO byte of the register, and then POP the HI byte in read_second_byte.
				-- Clock Cycles = 3.
				when pop_IX | pop_IY =>
					next_SP := std_logic_vector(unsigned(reg_SP)-1);
					cpu_addr <= reg_SP;
					WR <= false;
					DS <= true;
					next_state := read_first_byte;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Eight-bit indirect write operations with index register. We drive the index
				-- register onto the address lines on the next rising edge of the clock and
				-- assert DS and WR. We drive the value of A onto the data lines. On the falling
				-- edge of the clock in the middle of the next clock cycle, the target location
				-- will be written. By that time, we will be processing the next instruction.
				-- Clock Cycles = 1.
				when ld_ix_A =>
					cpu_data_out <= std_logic_vector(to_unsigned(reg_A,8));
					cpu_addr <= reg_IX;
					WR <= true;
					DS <= true;
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when ld_iy_A =>
					cpu_data_out <= std_logic_vector(to_unsigned(reg_A,8));
					cpu_addr <= reg_IY;
					WR <= true;
					DS <= true;
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Eight-bit indirect read operations with index register. We drive the index
				-- register onto the address lines on the next rising edge of the clcok and
				-- assert DS and unassert WR. On the falling edge in the middle of the next
				-- clock cycle, the target location's value will be driven onto the data bus.
				-- We read that value in a subsequent state, read_first_byte. Clock Cycles = 2.
				when ld_A_ix =>
					cpu_addr <= reg_IX;
					WR <= false;
					DS <= true;
					next_state := read_first_byte;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when ld_A_iy =>
					cpu_addr <= reg_IY;
					WR <= false;
					DS <= true;
					next_state := read_first_byte;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Eight-bit register increments and decrements, which use the combinatorial
				-- eight-bit adder to performm the adjustment. These operations set the zero and
				-- sign flags, but not the carry flag. Clock cycles = 1.
				when dec_A | inc_A => 
					next_A := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when dec_B | inc_B => 
					next_B := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when dec_C | inc_C => 
					next_C := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when dec_D | inc_D => 
					next_D := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when dec_E | inc_E => 
					next_E := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when dec_H | inc_H => 
					next_H := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when dec_L | inc_L => 
					next_L := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Address pointer increments and decrements. These do not affect the
				-- carry, sign, or zero flags. We use our combinatorial incrementer to
				-- add or subtract one from the pointer value. Clock Cycles = 1.
				when inc_IX => 
					next_IX := std_logic_vector(unsigned(reg_IX)+1);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when dec_IX => 
					next_IX := std_logic_vector(unsigned(reg_IX)-1);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when inc_IY => 
					next_IY := std_logic_vector(unsigned(reg_IY)+1);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when dec_IY => 
					next_IY := std_logic_vector(unsigned(reg_IY)-1);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when inc_SP => 
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when dec_SP => 
					next_SP := std_logic_vector(unsigned(reg_SP)-1);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Arithmetic operations, in which wse can add B register to the 
				-- accumulator, A, or subtract B, with carry or without. The carry, 
				-- sign, and zero flags are all set or cleared by these operations. 
				-- The calculation is performed by our combinatorial eight-bit 
				-- adder. Clock Cycles = 1.
				when add_A_B | sub_A_B | adc_A_B | sbc_A_B =>
					next_A := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_C := alu_cout;
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);

				-- Eight-bit logical operations in the accumulator with register
				-- B as the second operand. Sets the Z flag only. Clock Cycles = 1.
				when and_A_B | or_A_B | xor_A_B => 
					next_A := alu_out;
					next_flag_Z := (alu_out = 0);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Clear arithmetic flags: a single instruction clears the sign (S), 
				-- zero (Z), and carry (C) flags, but not the interrupt (I) flag;
				when clr_aflg =>
					next_flag_Z := false;
					next_flag_C := false;
					next_flag_S := false;
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);

				-- Clear interrupt flag, so as to enable interrupts.
				when clr_iflg =>
					next_flag_I := false;
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);

				-- Set interrupt flag, so as to disable interrupts.
				when set_iflg =>
					next_flag_I := true;
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Shift and rotate operations in the accumulator. All affect the
				-- carry bit, but none affect the zero or sign bits. Clock Cycles = 1.
				when rl_A | rlc_A | rr_A | rrc_A | sla_A | sra_A | srl_A =>
					next_A := alu_out;
					next_flag_C := alu_cout;
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- All other instructions need an operand. We don't bother to decode them
				-- separately at this point.
				when others => 
					next_state := read_first_operand;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				end case;
						
			-- Instructions have either one or two operands, which will be provided HI-byte
			-- first, as the program counter increments through the program space, and our
			-- byte ordering is big-endian. Here we read the first operand. If we have enough
			-- information to perform a write cycle, we do so immediately. We increment
			-- the program counter, either to point to the first byte of the next instruction, 
			-- or to point to the second operand, which will be the LO byte of a parameter.
			elsif (state = read_first_operand) then
				first_operand := to_integer(unsigned(prog_data));
				
				case opcode is
				
				-- We load a constant into the accumulator. Clock Cycles = 2.
				when ld_A_n => 
					next_A := first_operand;
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				
				-- We use our combinatorial adder to combine a constant and the
				-- accumulator value, and clock this new value into the accumulator
				-- on the next rising edge of the clock. Sets the Z, C, and S 
				-- flags. Clock Cycles = 2.
				when add_A_n | sub_A_n | adc_A_n | sbc_A_n =>
					next_A := alu_out;
					next_flag_Z := (alu_out = 0);
					next_flag_C := alu_cout;
					next_flag_S := (alu_out >= 128);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Logical operations upon the accumulator with a constant as the
				-- second operand. Sets the Z flag only. Clock Cycles = 2.
				when and_A_n | or_A_n | xor_A_n => 
					next_A := alu_out;
					next_flag_Z := (alu_out = 0);
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);

				-- All others require a second operand, so we don't decode them yet.
				when others => 
					next_state := read_second_operand;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				end case;
			
			-- Read a second operand, which will be the LO byte of a sixteen-bite 
			-- value. If we have enough information to begin a write cycle, we do so.
			-- We increment the program counter to point to the next instruction opcode.
			elsif (state = read_second_operand) then
				second_operand := to_integer(unsigned(prog_data));
				
				case opcode is 
				
				-- Indirect eight-bit read operations using operands as address. The first
				-- operand, being the one we loaded from a lower address, is the HI byte
				-- of the address, and the second operand is the LO byte. We drive the 
				-- address onto the address lines on the next rising edge, along with 
				-- DS and not WR. We read the value of the target location in the next
				-- clock cycle, in the read_first_byte state. Clock Cycles = 4.
				when ld_A_mm =>
					cpu_addr(ca_top downto 8) <= 
						std_logic_vector(to_unsigned(first_operand,cpu_addr_len-8));
					cpu_addr(7 downto 0) <= 
						std_logic_vector(to_unsigned(second_operand,8));
					WR <= false;
					DS <= true;
					next_state := read_first_byte;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Indirect eight-bit write operations using operands as address. Same as
				-- the read cycle, except we assert WR on the next rising edge, and drive
				-- the data lines with the value of A, and then leave the targe location
				-- to be written in the middle of the next cycle, while we get on with
				-- decoding the next instruction. Clock Cycles = 3.
				when ld_mm_A =>
					cpu_addr(ca_top downto 8) <= 
						std_logic_vector(to_unsigned(first_operand,cpu_addr_len-8));
					cpu_addr(7 downto 0) <= 
						std_logic_vector(to_unsigned(second_operand,8));
					cpu_data_out <= std_logic_vector(to_unsigned(reg_A,8));
					WR <= true;
					DS <= true;
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Direct load of constants into index registers. Clock Cycles = 3.
				when ld_IX_nn =>
					next_IX(ca_top downto 8) := 
						std_logic_vector(to_unsigned(first_operand,cpu_addr_len-8));
					next_IX(7 downto 0) := 
						std_logic_vector(to_unsigned(second_operand,8));
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
				when ld_IY_nn =>
					next_IY(ca_top downto 8) := 
						std_logic_vector(to_unsigned(first_operand,cpu_addr_len-8));
					next_IY(7 downto 0) := 
						std_logic_vector(to_unsigned(second_operand,8));
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Direct load of sixteen-bit constant into HL. Clock Cycles = 3;
				when ld_HL_nn =>
					next_H := first_operand;
					next_L := second_operand;
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
					
				-- Call a subroutine. We now have the full address to jump to, and we don't
				-- have to increment the program counter to read any more constants, so we
				-- can push the LO byte onto the stack, and push the HI byte in the next
				-- clock cycle, in the write_second_byte state. We save the program counter
				-- HI byte to push onto the stack in the next clock cycle. Clock Cycles = 4.
				when call_nn =>
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					cpu_addr <= std_logic_vector(unsigned(reg_SP)+1);
					WR <= true;
					DS <= true;
					cpu_data_out <= (others => '0');
					cpu_data_out(pa_top-8 downto 0) <= prog_addr(pa_top downto 8);
					next_state := write_second_byte;
					next_pa := prog_addr;
				
				-- Decide if we should jump to the address specified by the two operands. If
				-- so, we will jump immediately by loading the program counter with a new 
				-- value. Clock Cycles = 3.
				when jp_nn | jp_z_nn | jp_nz_nn | jp_nc_nn 
						| jp_c_nn | jp_np_nn | jp_p_nn =>
					case opcode is 		
					when jp_nn => jump := true;
					when jp_z_nn => jump := flag_Z;
					when jp_nz_nn => jump := not flag_Z;
					when jp_nc_nn => jump := not flag_C;
					when jp_c_nn => jump := flag_C;
					when jp_np_nn => jump := flag_S;
					when jp_p_nn => jump := not flag_S;
					when others => jump := false;
					end case;
					-- If we are supposed to jump, do so by setting the program counter to
					-- the specified absolute value. The first operand is the HI byte, the
					-- second the LO byte.
					if jump then
						next_pa(pa_top downto 8) := 
							std_logic_vector(to_unsigned(first_operand,prog_addr_len-8));
						next_pa(7 downto 0) := 
							std_logic_vector(to_unsigned(second_operand,8));
					else
						next_pa := std_logic_vector(unsigned(prog_addr)+1);
					end if;
					next_state := read_opcode;
					
				-- An others clause identical to one of the other opcodes.
				when others => 
					next_state := read_opcode;
					next_pa := std_logic_vector(unsigned(prog_addr)+1);
	
				end case;
				
			-- Read the first byte of data from the data bus. This could be a user memory access
			-- or a stack access. We never increment the program counter from this state because
			-- we have already obtained the full instruction.
			elsif (state = read_first_byte) then			
				case opcode is
				
				-- By now, the address lines have selected the target location, and half-way
				-- through this clock cycle, the target byte will be driven onto the data
				-- bus. On the next rising edge, even as we start decoding the next instruction,
				-- we will load the new value into the accumulator. Clock cycles = 2.
				when ld_A_mm | ld_A_ix | ld_A_iy => 
					next_A := to_integer(unsigned(cpu_data_in));
					next_pa := prog_addr;
					next_state := read_opcode;
					
				-- We have selected a byte in the stack with the stack pointer, and half-way
				-- through thic cycle it will appear on the data bus. We store it in the
				-- destination register on the next rising edge of the clock. 
				when pop_A | pop_B | pop_C | pop_D | pop_E | pop_H | pop_L | pop_F =>
					case opcode is
						when pop_A => next_A := to_integer(unsigned(cpu_data_in));
						when pop_B => next_B := to_integer(unsigned(cpu_data_in));
						when pop_C => next_C := to_integer(unsigned(cpu_data_in));
						when pop_D => next_D := to_integer(unsigned(cpu_data_in));
						when pop_E => next_E := to_integer(unsigned(cpu_data_in));
						when pop_H => next_H := to_integer(unsigned(cpu_data_in));
						when pop_L => next_L := to_integer(unsigned(cpu_data_in));
						when pop_F => 
							next_flag_I := (cpu_data_in(7) = '1');
							next_flag_C := (cpu_data_in(2) = '1');		
							next_flag_S := (cpu_data_in(1) = '1');
							next_flag_Z := (cpu_data_in(0) = '1');
						when others => next_A := to_integer(unsigned(cpu_data_in));
					end case;
					next_pa := prog_addr;
					next_state := read_opcode;
					
				-- Read the LO byte of the index register from the stack, and prepare to read
				-- the HI byte in the next clock cycle, in the read_second_byte state. 
				when pop_IX | pop_IY =>
					next_SP := std_logic_vector(unsigned(reg_SP)-1);
					cpu_addr <= reg_SP;
					WR <= false;
					DS <= true;
					case opcode is
						when pop_IX => next_IX(7 downto 0) := cpu_data_in(7 downto 0);
						when pop_IY => next_IY(7 downto 0) := cpu_data_in(7 downto 0);
						when others => next_IX(7 downto 0) := cpu_data_in(7 downto 0);
					end case;
					next_pa := prog_addr;
					next_state := read_second_byte;	
					
				-- Read the LO byte of the program counter from the stack and prepare to 
				-- read the HI byte in read_second_byte.
				when ret_cll | ret_int =>
					next_SP := std_logic_vector(unsigned(reg_SP)-1);
					cpu_addr <= reg_SP;
					WR <= false;
					DS <= true;
					next_pa := (others => '0');
					next_pa(7 downto 0) := cpu_data_in;
					next_state := read_second_byte;	
				
				-- An others clause identical to one of the above clauses.
				when others => 
					next_pa := (others => '0');
					next_pa(7 downto 0) := cpu_data_in;
					next_state := read_second_byte;	
	
				end case;
	
			-- Read the second byte of data. We never increment the program counter from
			-- this state, but we will set it to a the value popped off the stack by a
			-- return instruction.
			elsif (state = read_second_byte) then
				case opcode is
				
				-- Read the HI byte of index pointer from the stack.
				when pop_IX => 
					next_IX(ca_top downto 8) := cpu_data_in(ca_top-8 downto 0);
					next_pa := prog_addr;
					next_state := read_opcode;
				when pop_IY => 
					next_IY(ca_top downto 8) := cpu_data_in(ca_top-8 downto 0);
					next_pa := prog_addr;
					next_state := read_opcode;
	
				-- We are returning from a subroutine. We have already read the
				-- LO byte of the program counter from the stack, decremented
				-- the stack pointer, and now we are reading the HI byte. We use the 
				-- incr_pc state to increment the program counter so as to select
				-- the next instruction when we are returning from call_nn or
				-- sw_int, but if this is an interrupt return with flag_I set,
				-- we leave the program counter as it is and clear flag_I so we 
				-- can execute the instruction that was interrupted.
				when ret_cll | ret_int =>
					next_pa(pa_top downto 8) := cpu_data_in(pa_top-8 downto 0);
					next_pa(7 downto 0) := prog_addr(7 downto 0);	
					if (not flag_I) or (opcode = ret_cll) then
						next_state := incr_pc;
					else 
						next_flag_I := false;
						next_state := read_opcode;
					end if;
		
				-- An others clause identical to one of the above.
				when others => 
					next_pa(pa_top downto 8) := cpu_data_in(pa_top-8 downto 0);
					next_pa(7 downto 0) := prog_addr(7 downto 0);	

			end case;
			
			-- Write the second byte of data. We have no "write first byte" state because
			-- we write the first byte immediately upon decoding the instruction. We never
			-- increment the program counter in this state, but we will set it to the value
			-- specified by the two operands in a call instruction, having saved the previous
			-- program counter to the stack.
			elsif (state = write_second_byte) then
				case opcode is
				
				-- We increment the stack pointer and write the LO byte of the index 
				-- pointers to the stack, having already pushed the HI byte to the
				-- lower address. 
				when push_IX | push_IY =>
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					cpu_addr <= std_logic_vector(unsigned(reg_SP)+1);
					WR <= true;
					DS <= true;
					case opcode is
						when push_IX => cpu_data_out(7 downto 0) <= reg_IX(7 downto 0);
						when push_IY => cpu_data_out(7 downto 0) <= reg_IY(7 downto 0);
						when others => cpu_data_out(7 downto 0) <= reg_IX(7 downto 0);
					end case;
					next_pa := prog_addr;
					next_state := read_opcode;
					
				-- Here we are completing a CALL or sw_int instruction by pushing the LO byte 
				-- of the program counter onto the stack, and setting the next value of the
				-- program counter to the address given by the first operand (HI byte) and second
				-- operand (LO byte).
				when call_nn | sw_int =>
					next_SP := std_logic_vector(unsigned(reg_SP)+1);
					cpu_addr <= std_logic_vector(unsigned(reg_SP)+1);
					WR <= true;
					DS <= true;
					cpu_data_out <= prog_addr(7 downto 0);
					if (opcode = call_nn) then
						next_pa(pa_top downto 8) := 
							std_logic_vector(to_unsigned(first_operand,prog_addr_len-8));
						next_pa(7 downto 0) := 
							std_logic_vector(to_unsigned(second_operand,8));
					else
						next_pa(pa_top downto 8) := 
							std_logic_vector(to_unsigned((interrupt_pc / 256),prog_addr_len-8));
						next_pa(7 downto 0) := 
							std_logic_vector(to_unsigned((interrupt_pc rem 256),8));
					end if;
					next_state := read_opcode;
				
				-- An others clause identical to one of the above.
				when others => 
					next_pa := prog_addr;
					next_state := read_opcode;
										
				end case;
				
			-- The increment-program-counter state increments the program counter and moves
			-- to the read-opcode state. We use it when returning from a subroutine or a
			-- software interrupt, to increment the program counter we popped off the stack
			-- so that it points to the next instruction after the call or software interrupt.
			elsif (state = incr_pc) then
				next_state := read_opcode;
				next_pa := std_logic_vector(unsigned(prog_addr)+1);	
			end if;
			
			-- Assert the new state, program counter, and register values for the next clock 
			-- cycle.
			prog_addr <= next_pa; 
			state := next_state;					
			reg_A <= next_A;
			reg_B <= next_B;
			reg_C <= next_C;
			reg_D <= next_D;
			reg_E <= next_E;
			reg_H <= next_H;
			reg_L <= next_L;
			reg_IX <= next_IX;
			reg_IY <= next_IY;
			reg_SP <= next_SP;
			flag_C <= next_flag_C;
			flag_S <= next_flag_S;
			flag_Z <= next_flag_Z;
			flag_I <= next_flag_I;
		end if;
				-- Here we have combinatorial logic that applies the Arithmetic Logic Unit to eight-bit 
		-- registers and constants. One of the two inputs to the ALU is always the accumulator,
		-- which we name reg_A in the code. We use the ALU to increment and decrement registers
		-- as well as add, subtract, shift, rotate, and combine them logically. 
		opcode_now := to_integer(unsigned(prog_data));
		
		-- We begin with the behavior of the ALU when we are in the read_opcode state. We use 
		-- the current value of the program data as the opcode that controls the function of 
		-- the ALU. If we are in the read_opcode state and we receive an interrupt request, the 
		-- CPU will service the interrupt by overriding the value of prog_data with an nm_int 
		-- opcode. The ALU ignores this will override and behaves as if the value on prog_data 
		-- is the opcode. Because the nm_int command does not use the ALU, no error results from 
		-- ignoring the override.
		if (state = read_opcode) then
			case opcode_now is
			
			-- Eight-bit register increment and decrement.
			when inc_A | dec_A =>
				alu_in_x <= reg_A;
				alu_in_y <= 1;
				alu_cin <= false;
				if (opcode_now = inc_A) then alu_ctrl <= alu_cmd_add;
				else alu_ctrl <= alu_cmd_sub; end if;	
			when inc_B | dec_B =>
				alu_in_x <= reg_B;
				alu_in_y <= 1;
				alu_cin <= false;
				if (opcode_now = inc_B) then alu_ctrl <= alu_cmd_add; 
				else alu_ctrl <= alu_cmd_sub; end if;	
			when inc_C | dec_C =>
				alu_in_x <= reg_C;
				alu_in_y <= 1;
				alu_cin <= false;
				if (opcode_now = inc_C) then alu_ctrl <= alu_cmd_add; 
				else alu_ctrl <= alu_cmd_sub; end if;	
			when inc_D | dec_D =>
				alu_in_x <= reg_D;
				alu_in_y <= 1;
				alu_cin <= false;
				if (opcode_now = inc_D) then alu_ctrl <= alu_cmd_add;
				else alu_ctrl <= alu_cmd_sub; end if;	
			when inc_E | dec_E =>
				alu_in_x <= reg_E;
				alu_in_y <= 1;
				alu_cin <= false;
				if (opcode_now = inc_E) then alu_ctrl <= alu_cmd_add; 
				else alu_ctrl <= alu_cmd_sub; end if;	
			when inc_H | dec_H =>
				alu_in_x <= reg_H;
				alu_in_y <= 1;
				alu_cin <= false;
				if (opcode_now = inc_H) then alu_ctrl <= alu_cmd_add; 
				else alu_ctrl <= alu_cmd_sub; end if;	
			when inc_L | dec_L =>
				alu_in_x <= reg_L;
				alu_in_y <= 1;
				alu_cin <= false;
				if (opcode_now = inc_L) then alu_ctrl <= alu_cmd_add; 
				else alu_ctrl <= alu_cmd_sub; end if;	
				
			-- The delay instruction decrements A until it is zero.
			when dly_A =>
				alu_in_x <= reg_A;
				alu_in_y <= 1;
				alu_cin <= false;
				alu_ctrl <= alu_cmd_sub;
			
			-- Mathematical operations with A and B.
			when add_A_B | sub_A_B | adc_A_B | sbc_A_B =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_cin <= flag_C and ((opcode_now = adc_A_B) or (opcode_now = sbc_A_B));
				if (opcode_now = add_A_B) or (opcode_now = adc_A_B) then
					alu_ctrl <= alu_cmd_add;
				else 
					alu_ctrl <= alu_cmd_sub;
				end if;
				
			-- Logical operations on A with B.
			when and_A_B =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_ctrl <= alu_cmd_and;
			when or_A_B =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_ctrl <= alu_cmd_or;
			when xor_A_B =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_ctrl <= alu_cmd_xor;
			
			-- Shift and Rotate Operations on A. We specify reg_B for alu_in_y
			-- because it reduces our logic footprint.
			when rl_A | rlc_A | rr_A | rrc_A | sla_A | sra_A | srl_A =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_cin <= flag_C;
				case opcode_now is 
					when rl_A  => alu_ctrl <= alu_cmd_rl;
					when rlc_A => alu_ctrl <= alu_cmd_rlc;
					when rr_A  => alu_ctrl <= alu_cmd_rr;
					when rrc_A => alu_ctrl <= alu_cmd_rrc;
					when sla_A => alu_ctrl <= alu_cmd_sla;
					when sra_A => alu_ctrl <= alu_cmd_sra;
					when srl_A => alu_ctrl <= alu_cmd_srl;	
					when others => alu_ctrl <= alu_cmd_srl;
				end case;
				
			when others =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_ctrl <= alu_cmd_nop;
				
			end case;
		-- If we are not in the read_opcode state, we have a variable "opcode" that holds
		-- the value of the opcode that was presented in the most recent read_opcode state.
		-- We use this variable to control the behavior of the ALU. The operand we are 
		-- going to use for the Y-input of the ALU will always be the current value of the
		-- program data, so we do not use the state machine's "first_operand" variable, but
		-- the program data directly.
		else 
			case opcode is 
			
			-- Operations with A and a constant.
			when add_A_n | sub_A_n | adc_A_n | sbc_A_n =>
				alu_in_x <= reg_A;
				alu_in_y <= to_integer(unsigned(prog_data));
				alu_cin <= flag_C and ((opcode = adc_A_n) or (opcode = sbc_A_n));
				if (opcode = add_A_n) or (opcode = adc_A_n) then
					alu_ctrl <= alu_cmd_add;
				else 
					alu_ctrl <= alu_cmd_sub;
				end if;		
				
			-- Logical operations on A and a constant. We specify reg_B for alu_in_y
			-- because it reduces our logic footprint.
			when and_A_n =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_in_y <= to_integer(unsigned(prog_data));
				alu_ctrl <= alu_cmd_and;
			when or_A_n =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_in_y <= to_integer(unsigned(prog_data));
				alu_ctrl <= alu_cmd_or;
			when xor_A_n =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_in_y <= to_integer(unsigned(prog_data));
				alu_ctrl <= alu_cmd_xor;
			
			when others =>
				alu_in_x <= reg_A;
				alu_in_y <= reg_B;
				alu_ctrl <= alu_cmd_nop;
			
			end case;
		end if;
		
		-- Assign the diagnostic signal vector, which may or may not be used in the 
		-- main program to drive test poins or interrupts.
		case state is
			when read_opcode => SIG(2 downto 0) <= "001";
			when read_first_operand => SIG(2 downto 0) <= "010";
			when read_second_operand => SIG(2 downto 0) <= "011";
			when read_first_byte => SIG(2 downto 0) <= "100";
			when read_second_byte => SIG(2 downto 0) <= "101";
			when write_second_byte => SIG(2 downto 0) <= "110";
			when incr_pc => SIG(2 downto 0) <= "111";
			when others => SIG(2 downto 0) <= "000";
		end case;
	end process;
end behavior;


