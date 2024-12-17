; Blood Pressure Monitor (BPM) Program (C3051A-V2)
; ------------------------------------------------------

; This code runs in the OSR8 microprocessor of the A3051A.

; Calibration Constants
const tx_frequency      5  ; Transmit frequency calibration
const device_id        51  ; Will be used as the first channel number.
const P_sample_period   0  ; P sample period, use 0 for 256.
const T_sub_sample      2  ; The number of P samples per T sample.

; Address Map Boundary Constants
const mmu_vmem 0x0000 ; Base of Variable Memory
const mmu_ctrl 0x0600 ; Base of Control Space
const mmu_sba  0x0300 ; Stack Base Address

; Address Map Constants
const mmu_i2c00 0x0600 ; i2c SDA=0 SCL=0
const mmu_i2c01 0x0601 ; i2c SDA=0 SCL=1 
const mmu_i2cA0 0x0602 ; i2c SDA=A SCL=0
const mmu_i2cA1 0x0603 ; i2c SDA=A SCL=1 
const mmu_i2cZ0 0x0604 ; i2c SDA=Z SCL=0
const mmu_i2cZ1 0x0605 ; i2c SDA=Z SCL=1 
const mmu_i2cMR 0x0606 ; i2c Most Recent Eight Bits
const mmu_sr    0x060F ; Status Register
const mmu_irqb  0x0610 ; Interrupt Request Bits
const mmu_imsk  0x0612 ; Interrupt Mask Bits
const mmu_irst  0x0614 ; Interrupt Reset Bits
const mmu_itp   0x0618 ; Interrupt Timer Period 
const mmu_rst   0x0619 ; System Reset
const mmu_xhb   0x0620 ; Transmit HI Byte
const mmu_xlb   0x0621 ; Transmit LO Byte
const mmu_xcn   0x0622 ; Transmit Channel Number
const mmu_xcr   0x0624 ; Transmit Control Register
const mmu_xfc   0x0626 ; Transmit Frequency Calibration
const mmu_etc   0x0630 ; Enable Transmit Clock
const mmu_tcf   0x0632 ; Transmit Clock Frequency
const mmu_tcd   0x0634 ; Transmit Clock Divider
const mmu_bcc   0x0636 ; Boost CPU Clock
const mmu_dfr   0x0638 ; Diagnostic Flag Resister

; Configuration Constants
const min_tcf       75 ; Minimum TCK periods per half RCK period.
const max_tcd       31 ; Maximum possible value of transmit clock divisor.

; Timing Constants.
const tx_delay      50 ; Wait time for sample transmission, TCK periods.
const boot_delay     3 ; Boot delay, multiples of 7.8 ms.
const main_delay    21 ; Delay in main loop.

; Sensor Addresses
const ps_SAD      0x5C ; Pressure sensor I2C address (SAD).
const ps_IF_CTRL  0x0E ; Interface Control
const ps_WHO_AM_I 0x0F ; Fixed value 0xB4
const ps_CTRL1    0x10 ; Control Register One
const ps_CTRL2    0x11 ; Control Register Two
const ps_CTRL3    0x12 ; Control Register Three
const ps_CTRL4    0x13 ; Control Register Four
const ps_P_XL     0x28 ; Pressure Extra LO byte.
const ps_P_L      0x29 ; Pressure LO byte.
const ps_P_H      0x2A ; Pressure HI byte.
const ps_TEMP_L   0x2B ; Temperature LO byte.
const ps_TEMP_H   0x2C ; Temperature HI byte.

; Sensor Constants
const ps_1s_config 0x00 ; Configure sensor for one-shot.
const ps_meas_req  0x41 ; Request a new measurement.

; Math constants.
const off_16bs    0x80  ; Convert sixteen bit signed to unsigned.
const rand_taps   0xB4  ; Determines which taps to XOR.

; Process variables.
const counter_T   0x00  ; A sub-sample counter for temperature.
const rand_1      0x01  ; Random number hight byte.
const rand_0      0x02  ; Random number low byte.

; ------------------------------------------------------------
; The CPU reserves two locations 0x0000 for the start of program
; execution, and 0x0003 for interrupt execution. We put jumps at
; both locations. A jump takes exactly three bytes.

start: 
jp initialize
jp interrupt

; ------------------------------------------------------------
; The interrupt routine. Reads the sensor, transmits measurements,
; and initiates new measurements.

interrupt:

; Save A on the stack. Set the zero bit of the diagnostic flag 
; register. We can direct this bit to a test pin to see if the 
; interrupt is happening.

push A 
ld A,(mmu_dfr)     
or A,0x01          
ld (mmu_dfr),A 

; Delay a random number of clock cycles. 

ld A,(rand_0)
and A,0x0F
dly A

; Enable the transmit clock and boost the CPU to TCK. 
      
ld A,0x01          
ld (mmu_etc),A    
ld (mmu_bcc),A

; Push the flags as well as registers B, C, and D onto the stack.

push F  
push B  
push C
push D

; Read the XL, L, and H pressure bytes with a three-byte i2c
; read. They will be returned in C, B, and A respectively.

ld A,ps_P_XL
call i2c_rd24

; Shift the entire twenty-four bit measurement one bit to
; the left and write the top two bytes of the shifted data
; into the transmit data registers.

push A
pop D
push C
pop A
rl A
push B
pop A
rl A
ld (mmu_xlb),A
push D
pop A
rl A
ld (mmu_xhb),A

; Initiate the transmission of the pressure measurement. The transmit
; clock must continue to run for tx_delay clock cycles in order for
; the transmit to complete. Because we are going to access the sensor
; immediately after initiating the transmission, we do not need to 
; include an explicit delay as in, "ld A,tx_delay; dly A".

ld A,device_id
ld (mmu_xcn),A
ld (mmu_xcr),A

; If it's time to report temperature, proceed to do so.

ld A,(counter_T)
dec A
ld (counter_T),A
jp nz,int_measure
ld A,T_sub_sample
ld (counter_T),A

; Read the sixteen-bit temperature measurement. We will read two bytes, 
; T_L and T_H. These bytes will be returned C and B respectively.

ld A,ps_TEMP_L
call i2c_rd16

; Add an offset to the two's compliment temperature measurement so that it
; is now centered at 32678. Write the two bytes to the transmit registers.

push B
pop A
add A,off_16bs
ld (mmu_xhb),A
push C
pop A
ld (mmu_xlb),A

; Initiate the transmission on a new channel number. We do not need an
; explicit transmission delay because we are going to write to the 
; sensor, which takes enough time for the transmission to complete.

ld A,device_id
inc A
ld (mmu_xcn),A
ld (mmu_xcr),A

; Tell the sensor to measure temperature and pressure again. We'll read
; out the values on our next interrupt. We pass the eight-bit I2C write
; routine the value to be written in register C and the address to be 
; written in A.

int_measure:
ld A,ps_meas_req
push A
pop C
ld A,ps_CTRL2
call i2c_wr8 

; Reset all interrupts.

ld A,0xFF   
ld (mmu_irst),A   

; Clear bit zero of the diagnostic flag register to indicate the end
; of the interrupt.

ld A,(mmu_dfr)  
and A,0xFE   
ld (mmu_dfr),A 

; Restore D, C, B, and F.

pop D
pop C
pop B
pop F 

; Move the CPU out of boost, stop the transmit clock, pop A, and 
; return from interrupt.

ld A,0x00        
ld (mmu_bcc),A    
ld (mmu_etc),A 
pop A
rti  

; ------------------------------------------------------------
; Calibrate the transmit clock frequency. Will leave the
; transmit clock disabled and cpu boost turned off. The 
; transmit clock should run at 5 MHz. We are going to determine
; which value of transmit clock divisor, when applied to the
; ring oscillator frequency, gives us a frequency close to
; 5 MHz. We will write that value to the transmit clock divisor
; register, and so calibrate the ring oscillator.

calibrate_tck:

; Push the registers we are going to use.

push F
push A           
push B   

; Disable the CPU boost clock, to enter non-boost, where we are
; running off 32.768 kHz.

ld A,0x00    
ld (mmu_bcc),A   

; Turn off the transmit clock (TCK) by writing a zero to the
; enable transmit clock location in our control space. 

ld (mmu_etc),A

; Pick an initial value for the divisor that we know will be 
; too high, so the transmit clock frequency will certainly
; be too low. Store this value in both A and B registers. There
; is no instruction to move A directly into B. Instead, we push
; A onto the stack, then pop it off into B.

ld A,max_tcd 
push A  
pop B

; We enter a loop in which we decrement the divisor stored in
; B, then try it out and see if the resulting frequency is 
; above the minimum frequency we are prepared to accept. We
; measure the frequency using the transmit clock frequency
; register, which counts the number of TCK cycles in one
; half of an RCK cycle immediately after we enable the 
; transmit clock. When we have this number of cycles, we 
; subtract a minimum, and if the result is negative, we know
; we have to decrease the divisor further. If the result is
; positive, we are done.

cal_tck_1:
dec B           
push B           
pop A            
ld (mmu_tcd),A   
ld A,0x01        
ld (mmu_etc),A   
ld A,(mmu_tcf)  
sub A,min_tcf   
ld A,0x00       
ld (mmu_etc),A   
jp np,cal_tck_1 

; Pop and return.

pop B            
pop A          
pop F
ret              

; ------------------------------------------------------------
; Initialize the sensor by writing to its configuration registers.

sensor_init:

push A       
push F  
push B  
push C

ld A,0x02
ld (mmu_dfr),A

ld A,ps_1s_config
push A
pop C
ld A,ps_CTRL1
call i2c_wr8

ld A,ps_meas_req
push A
pop C
ld A,ps_CTRL2
call i2c_wr8

ld A,0x00
ld (mmu_dfr),A

pop C
pop B
pop F 
pop A
ret

; ------------------------------------------------------------
; Initialize the processor, clock, memory, and sensor.

initialize:

; Start a pulse on DF1.

ld A,0x02
ld (mmu_dfr),A

; Disable and reset interrupts.

ld A,0x00 
ld (mmu_imsk),A 
ld A,0xFF           
ld (mmu_irst),A

; Initialize the stack pointer.

ld HL,mmu_sba
ld SP,HL

; Implement a boot delay sufficient to allow the power supplies to
; settle and the sensor to power up before we start calibrating and
; initializing.

ld A,boot_delay 
push A
pop B
pwr_up_lp:
ld A,0xFF
dly A
dec B
jp nz,pwr_up_lp

; End the pulse on DF1.

ld A,0x00
ld (mmu_dfr),A

; Calibrate the transmit clock.

call calibrate_tck

; Set the low radio frequency for sample transmission

ld A,tx_frequency
ld (mmu_xfc),A

; Initialize variables.

ld A,T_sub_sample
ld (counter_T),A
ld A,0xFF
ld (rand_0),A
ld (rand_1),A

; Initialize the pressure sensor.

call sensor_init

; Set interrupt timer interval and enable the timer interrupt to implement
; the sample period. The value we want to load into the interrup timer period 
; register is the sample period minus one, because the interrupt timer counts
; the value down to zero. So we load A with sample_period, then decrement. If
; sample_period is zero (0x00), the value we write is 255 (0xFF).

ld A,0xFF           
ld (mmu_irst),A  
ld A,P_sample_period 
dec A 
ld (mmu_itp),A 
ld A,0x01 
ld (mmu_imsk),A 

; Done with initialization, jump to main loop.

jp main


; ------------------------------------------------------------
; The main program loop. The interrupts will be running 
; in the background, and they do all the work. The main
; routine generates a pulse on bit one of the diagnostic
; flag register. It's only task is to update the random 
; number, and implement a delay.

main:

ld A,(mmu_dfr) 
or A,0x02
ld (mmu_dfr),A

; Rotate rand_1 to the right, filling the top bit with
; zero and placing the bottom bit in carry. 

ld A,(rand_1)    
srl A       
ld (rand_1),A 

; Rotate rand_0 to the right, filling top bit with carry 
; and placing the bottom bit in carry.

ld A,(rand_0)
rr A
ld (rand_0),A

; If carry is set, XOR rand_1 with tap bits. We have now
; updated our random number.

jp nc,rand_tz 
ld A,(rand_1)  
xor A,rand_taps
ld (rand_1),A
rand_tz:

ld A,(mmu_dfr)
and A,0xFD
ld (mmu_dfr),A

ld A,main_delay
dly A      
jp main


; ------------------------------------------------------------
; I2C Eight-Bit Write. Write to one register location on the
; sensor. The address of the register should be passed into
; the routine in the accumulator and the byte to be written 
; should be passed in C.

i2c_wr8:

; Store the sub-address address in B.

push A            ; 1
pop B             ; 2
       
; I2C: Start code (ST)

ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2c00),A  ; 3

; I2C: Write seven-bit device address and !WRITE flag (SAD+W).

ld A,ps_SAD       ; 2
sla A             ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Write eight-bit sub address (SUB), which is
; stored in B.

push B            ; 1
pop A             ; 2
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Write the eight data bits.

push C            ; 1
pop A             ; 2
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Stop code (SP).

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ1),A  ; 3
     
ret


; ------------------------------------------------------------
; I2C Sixteen-Bit Read. Read two consecutive bytes from the sensor
; address map. The address of the first byte should be passed into
; the routine in the accumulator. The first byte read will be returned
; in C, the second in B.

i2c_rd16:

; Store the sub-address address in B.

push A            ; 1
pop B             ; 2

; I2C: Start code (ST)

ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2c00),A  ; 3

; I2C: Write seven-bit device address and !WRITE flag (SAD+W).

ld A,ps_SAD       ; 2
sla A             ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Write eight-bit sub address (SUB), which is
; stored in B.

push B            ; 1
pop A             ; 2
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Repeat start code (RS)

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2c00),A  ; 3

; I2C: Write seven-bit device address again, this time with
; a READ flag (SAD+R).

ld A,ps_SAD       ; 2
sla A             ; 1
or A,0x01         ; 2
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Read eight data bits from slave (DATA).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Transfer the first data byte to C.

ld A,(mmu_i2cMR)  ; 4
push A            ; 1
pop C             ; 2

; I2C: Transmit master acknowledgement (MAK).

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Read eight data bits from slave (DATA).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Transfer the second data byte to B.

ld A,(mmu_i2cMR)  ; 4
push A            ; 1
pop B             ; 2

; I2C: Transmit not master acknowledgement (NMAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Stop code (SP).

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ1),A  ; 3
     
ret


; ------------------------------------------------------------
; I2C Twenty-Four-Bit Read. Read three consecutive bytes from the sensor
; address map. The address of the first byte should be passed into
; the routine in the accumulator. The first byte read will be returned
; in C, the second in B, the third in A.

i2c_rd24:

; Store the sub-address address in B.

push A            ; 1
pop B             ; 2

; I2C: Start code (ST)

ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2c00),A  ; 3

; I2C: Write seven-bit device address and !WRITE flag (SAD+W).

ld A,ps_SAD       ; 2
sla A             ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Write eight-bit sub address (SUB), which is
; stored in B.

push B            ; 1
pop A             ; 2
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Repeat start code (RS)

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2c00),A  ; 3

; I2C: Write seven-bit device address again, this time with
; a READ flag (SAD+R).

ld A,ps_SAD       ; 2
sla A             ; 1
or A,0x01         ; 2
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

rl A              ; 1
ld (mmu_i2cA0),A  ; 3
ld (mmu_i2cA1),A  ; 3
ld (mmu_i2cA0),A  ; 3

; I2C: Accept slave acknowledgement (SAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Read eight data bits from slave (DATA).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Transfer the first data byte to C.

ld A,(mmu_i2cMR)  ; 4
push A            ; 1
pop C             ; 2

; I2C: Transmit master acknowledgement (MAK).

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Read eight data bits from slave (DATA).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Transfer the second data byte to B.

ld A,(mmu_i2cMR)  ; 4
push A            ; 1
pop B             ; 2

; I2C: Transmit master acknowledgement (MAK).

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Read eight data bits from slave (DATA).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; Transfer the third data byte to A.

ld A,(mmu_i2cMR)  ; 4

; I2C: Transmit not master acknowledgement (NMAK).

ld (mmu_i2cZ0),A  ; 3
ld (mmu_i2cZ1),A  ; 3
ld (mmu_i2cZ0),A  ; 3

; I2C: Stop code (SP).

ld (mmu_i2c00),A  ; 3
ld (mmu_i2c01),A  ; 3
ld (mmu_i2cZ1),A  ; 3
     
ret

