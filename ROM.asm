; Blood Pressure Monitor (BPM) Program (C3051A-V2)
; ------------------------------------------------------

; This code runs in the OSR8 microprocessor of the A3051A.

; Calibration Constants
const tx_frequency      5  ; Transmit frequency calibration
const device_id        52  ; Will be used as the first channel number.
const sample_period     0  ; Sample period in units of RCK periods, use 0 for 256.

; Address Map Boundary Constants
const mmu_vmem 0x0000 ; Base of Variable Memory
const mmu_ctrl 0x0600 ; Base of Control Space
const mmu_sba  0x0300 ; Stack Base Address

; Address Map Constants
const mmu_sda  0x0600 ; Sensor Data Access
const mmu_scl  0x0601 ; Sensor Clock
const mmu_sr   0x0605 ; Status Register
const mmu_irqb 0x0610 ; Interrupt Request Bits
const mmu_imsk 0x0612 ; Interrupt Mask Bits
const mmu_irst 0x0614 ; Interrupt Reset Bits
const mmu_itp  0x0618 ; Interrupt Timer Period 
const mmu_rst  0x0619 ; System Reset
const mmu_sph  0x061B ; Initial Stack Pointer HI Byte
const mmu_spl  0x061C ; Initial Stack Pointer LO Byte
const mmu_xhb  0x0620 ; Transmit HI Byte
const mmu_xlb  0x0621 ; Transmit LO Byte
const mmu_xcn  0x0622 ; Transmit Channel Number
const mmu_xcr  0x0624 ; Transmit Control Register
const mmu_xfc  0x0626 ; Transmit Frequency Calibration
const mmu_etc  0x0630 ; Enable Transmit Clock
const mmu_tcf  0x0632 ; Transmit Clock Frequency
const mmu_tcd  0x0634 ; Transmit Clock Divider
const mmu_bcc  0x0636 ; Boost CPU Clock
const mmu_dfr  0x0638 ; Diagnostic Flag Resister

; Configuration Constants
const min_tcf       75  ; Minimum TCK periods per half RCK period.
const max_tcd       31  ; Maximum possible value of transmit clock divisor.

; Timing constants.
const tx_delay      50  ; Wait time for sample transmission, TCK periods.
const sa_delay      70  ; Wait time for sensor access, TCK periods.
const boot_delay    10  ; Boot delay, multiples of 7.8 ms.

; Sensor Bit Masks
const sda_mask     0x01 ; Sensor Data Access (SDA) in status register.
const idy_mask     0x02 ; Interrupt Data Ready (IDY) in status register.
const scl_mask     0x04 ; Sensor Clock (SCL) in status register.

; Variables
const xmit_hi    0x0000 ;
const xmit_lo    0x0001 ;


; ------------------------------------------------------------

; The CPU reserves two locations 0x0000 for the start of program
; execution, and 0x0003 for interrupt execution. We put jumps at
; both locations. A jump takes exactly three bytes.
start: 
jp initialize
jp interrupt
            
; ------------------------------------------------------------
; i2c_ST generates a start token on SDA and SCL. It assumes that
; the SDA and SCL lines are both HI before it executes.
i2c_ST:
push A

ld A,0
ld (mmu_sda),A
ld (mmu_scl),A

pop A
ret

; ------------------------------------------------------------
; i2c_SP generates a stop token on SDA and SCL. It assumes SCL is
; LO before it executes, but does not assume SDA is LO.
i2c_SP:
push A

ld A,0
ld (mmu_sda),A
ld A,1
ld (mmu_scl),A
ld (mmu_sda),A

pop A
ret

; ------------------------------------------------------------
; i2c_RS generates a repeat-start token on SDA and SCL. It assumes
; SCL is LO before it executes, but does not assume SDA is HI.
i2c_RS:
push A

ld A,1
ld (mmu_sda),A
ld A,1
ld (mmu_scl),A
ld A,0
ld (mmu_sda),A

pop A
ret

; ------------------------------------------------------------
; i2c_WR writes a bit to the bus by driving SDA and generating 
; an SCL pulse. It assumes that SCL is LO before execution. The
; bit to be transmitted is the zero bit in A.
i2c_WR:

push F
push A

and A,0x01
ld (mmu_sda),A
ld A,1
ld (mmu_scl),A
ld A,0
ld (mmu_scl),A

pop A
pop F
ret



; ------------------------------------------------------------
; i2c_RD reads a bit from the bus by generating an SCL pulse.
; It returns the state of the bit in the Z flag. It assumes
; SCL is LO before execution. It starts by putting SDA into high-Z.
i2c_RD:

push A

ld A,0x03
ld (mmu_sda),A
ld A,1
ld (mmu_scl),A
ld A,(mmu_sr)
and A,0x01
ld A,0
ld (mmu_scl),A

pop A
ret



; ------------------------------------------------------------
; The interrupt routine. Reads the sensor and transmits the
; measurement with the sample transmitter.
interrupt:

; Save A on the stack, set bit zero to one, use to enable
; the transmit clock and then boost the CPU to TCK.
push A       
ld A,0x01          
ld (mmu_etc),A    
ld (mmu_bcc),A
push F    

; Set the zero bit of the diagnostic flag register. We can
; direct this bit to a test pin to see if the interrupt is
; happening. We read the register and OR with 0x01 to set
; the zero bit.
ld A,(mmu_dfr)     
or A,0x01          
ld (mmu_dfr),A  

; Temporary code: exercise i2c token routines.
call i2c_ST
ld A,0
call i2c_WR
ld A,1
call i2c_WR
ld A,0
call i2c_WR
ld A,1
call i2c_WR
ld A,0
call i2c_WR
ld A,1
call i2c_WR
ld A,0
call i2c_WR
ld A,1
call i2c_WR
call i2c_SP

; Temporary code: increment a sixteen-bit counter variable
; and transmit.
ld A,(xmit_lo)
add A,7
ld (xmit_lo),A
ld (mmu_xlb),A
ld A,(xmit_hi)
adc A,0
ld (xmit_hi),A
ld (mmu_xhb),A
          
; Write the device identifier to the transmit channel number register.
ld A,device_id   
ld (mmu_xcn),A 

; Initiate transmission with any write to the transmission control
; register.
ld (mmu_xcr),A 

; Wait for transmission to complete. We must keep the transmit clock going
; so the state machine will finish its transmission.
ld A,tx_delay    
dly A            

; Reset all interrupts.
ld A,0xFF   
ld (mmu_irst),A   

; Clear bit zero of the diagnostic flag register to indicate the end
; of the interrupt.
ld A,(mmu_dfr)  
and A,0xFE   
ld (mmu_dfr),A 

; Move the CPU out of boost, then stop the transmit clock.
ld A,0x00        
ld (mmu_bcc),A    
ld (mmu_etc),A 

; Restore F, then A, then return from interrupt.
pop F 
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
; Sensor initialization routine. Write to all registers we need to
; configure to obtain pressure and temperature measurements of the
; desired range and precision.
sensor_init:

; Push the two registers we are going to use onto the stack. We'll
; pop them off later to recover them, just before we return from
; this subroutine.
push F
push A  

; Restore the accumulator and flags register before returning.
pop A            
pop F 
ret   


; ------------------------------------------------------------
; Initialize the device. We will be setting up the stack, calibrating
; the ring oscillator, and configuring the sensor.
initialize:

; Initialize the stack pointer.
ld HL,mmu_sba
ld SP,HL

; The boot delay allows the power supplies to settle before we calibrate
; the ring oscillator.
ld A,boot_delay  ; We want boot_delay
push A           ; multiplied by 256 cycles of RCK
pop B            ; before proceeding.
pwr_up_lp:
ld A,0xFF        ; Load A with 255
dly A            ; and delay,
dec B            ; then decrement B.
jp nz,pwr_up_lp

; Calibrate the transmit clock.
call calibrate_tck

; Set the low radio frequency for sample transmission
ld A,tx_frequency
ld (mmu_xfc),A
ld A,0
ld (xmit_hi),A
ld (xmit_lo),A

; Initialize the sensor.
call sensor_init

; Set interrupt timer interval and enable the timer interrupt to implement
; the sample period. The value we want to load into the interrup timer period 
; register is the sample period minus one, because the interrupt timer counts
; the value down to zero. So we load A with sample_period, then decrement. If
; sample_period is zero (0x00), the value we write is 255 (0xFF).
ld A,0xFF           
ld (mmu_irst),A  
ld A,sample_period 
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
; flag register, and it implements a delay so these 
; pulses are rare.
main:

ld A,(mmu_dfr) 
or A,0x02
ld (mmu_dfr),A
and A,0xFD
ld (mmu_dfr),A
ld A,255  
dly A      
jp main

; ------------------------------------------------------------
