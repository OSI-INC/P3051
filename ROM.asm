; Blood Pressure Monitor (BPM) Program (C3051A-V2)
; ------------------------------------------------------

; This code runs in the OSR8 microprocessor of the A3051A.

; Calibration Constants
const tx_frequency      6  ; Transmit frequency calibration
const device_id        17  ; Will be used as the first channel number.
const sample_period     0  ; Sample period in units of RCK periods, use 0 for 256.

; Address Map Boundary Constants
const mmu_vmem 0x0000 ; Base of Variable Memory
const mmu_ctrl 0x0600 ; Base of Control Space
const mmu_sba  0x0300 ; Stack Base Address

; Address Map Constants
const mmu_shb  0x0600 ; Sensor Data HI Byte 
const mmu_slb  0x0601 ; Sensor Data LO Byte
const mmu_sar  0x0602 ; Sensor Address Register
const mmu_scr  0x0604 ; Sensor Control Register
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
const init_cntr 	    0x0C00 ; Value for HL to count down from during initialization.
const min_tcf       	75     ; Minimum TCK periods per half RCK period.
const max_tcd           15     ; Maximum possible value of transmit clock divisor.

; Timing consstants.
const tx_delay      50  ; Wait time for sample transmission, TCK periods.
const sa_delay      70  ; Wait time for sensor access, TCK periods.
const boot_delay    10  ; Boot delay, multiples of 7.8 ms.

; Sensor interface commands.
const sensor_rd16   0x04 ; Read Sixteen-Bit Word from Sensor
const sensor_wr16   0x06 ; Write Sixteen-Bit Word to Sensor
const sensor_rd8    0x00 ; Read Eight-Bit Byte from Sensor
const sensor_wr8    0x02 ; Write Eight-Bit Byte to Sensor

; Math constants.
const off_16bs    0x80  ; Convert sixteen bit signed to unsigned.

; ------------------------------------------------------------

; The CPU reserves two locations 0x0000 for the start of program
; execution, and 0x0003 for interrupt execution. We put jumps at
; both locations. A jump takes exactly three bytes.
start: 
jp initialize
jp interrupt


; ------------------------------------------------------------
; Prototype sensor initialization routine, showing how to load
; the sensor interface address, HI and LO data bytes, and then
; initiate the exchange with a write to the sensor control 
; register. The sensor interface supports both single and double-
; byte reads and writes, so we have four commands that we can
; write to the control register. The routine assumes that the 
; CPU is running in non-boost, so its clock period is 30 us.
; A serial access begun in one CPU operation will be complete
; before the next CPU operation.

sensor_init:

; Push the two registers we are going to use onto the stack. We'll
; pop them off later to recover them, just before we return from
; this subroutine.
push F
push A  

; We load a sensor register address into A, then write to the 
; sensor interface.
ld A,0x00
ld (mmu_sar),A 

; We load into A the data byte that we want to write to the sensor
; register, then we write the byte to the sensor interface LO byte.
ld A,0xAA   
ld (mmu_slb),A 

; Load A with the sensor write byte command and write to the sensor
; control register. The write will begin immediately, but it will
; take about ten microseconds. 
ld A,sensor_wr8	 
ld (mmu_scr),A 

; Load another sensor register address into A, then write to the 
; sensor interface. We don't have to delay before setting up this
; next sensor access because our clock period is 30 us. The sensor
; access we began in the previous operation is already complete.
ld A,0x01
ld (mmu_sar),A 

; We load into A the data byte that we want to write to the sensor
; register, then we write the byte to the sensor interface LO byte.
ld A,0xBB   
ld (mmu_slb),A 

; Load A with the sensor write byte command and write to the sensor
; control register. The write will begin immediately, but it will
; take about ten microseconds. 
ld A,sensor_wr8	 
ld (mmu_scr),A 

; Restore the accumulator and flags register before returning.
pop A            
pop F 
ret               

; ------------------------------------------------------------
; Prototype sensor transmit routines. Here we assume we are 
; running in Boost Mode within an interrupt routine, so we 
; insert the sensor access delay to wait for the access to
; complete.

xmit_sensor:

; We'll be using the accumulator, and we'll be setting the
; flags with an addition, so push flags and accumulator
; onto the stack.
push F
push A          

; Write the device identifier to the transmit channel number register.
ld A,device_id   
ld (mmu_xcn),A 

; Read sixteen bits from sensor address zero. 
ld A,0x00        
ld (mmu_sar),A 
ld A,sensor_rd16 
ld (mmu_scr),A 

; Wait for the read to complete.
ld A,sa_delay    
dly A            

; Load the HI byte into A and offset to make the most negative output
; value zero. We are tranmitting positive values only. We write the
; offset HI byte to the HI byte of the our transmitter.
ld A,(mmu_shb)  
add A,off_16bs   
ld (mmu_xhb),A   

; Read the LO byte and write to the transmitter.
ld A,(mmu_slb) 
ld (mmu_xlb),A 

; Initiate transmission with any write to the transmission control
; register.
ld (mmu_xcr),A 

; Wait for the transmission to complete before returning. We may not
; have to do this, if we are sure no other transmission will be started
; immediately by the calling routine.
ld A,tx_delay  
dly A 

; Restore registers and return.
pop A           
pop F
ret                         

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
; too high, so the transmit clock frequencyu will certainly
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
; The interrupt routine. Transmits the sensor measurement. It 
; generates a pulse on bit zero of the diagnostic flag register.

interrupt:

push F              
push A              

ld A,(mmu_dfr)      ; Load the diagnostic flag register.
or A,0x01           ; set bit zero and
ld (mmu_dfr),A      ; write to diagnostic flag register.

ld A,0x01           ; Set bit zero to one.
ld (mmu_etc),A      ; Enable the transmit clock, TCK.
ld (mmu_bcc),A      ; Boost the CPU clock to TCK.

call xmit_sensor    ; Transmit pressure measurement.

ld A,(mmu_dfr)      ; Load the diagnostic flag register.
and A,0xFE          ; set bit zero and
ld (mmu_dfr),A      ; write to diagnostic flag register.

ld A,0xFF           ; Set all bits to one and use to
ld (mmu_irst),A     ; reset all interrupts.

pop A               ; Restore A
pop F               ; Restore the flags.
rti                 ; Return from interrupt.

; ------------------------------------------------------------
; Initialize the device.
initialize:

; Initialize the stack pointer.
ld HL,mmu_sba
ld SP,HL

; Wait for a while. The power supplies must settle after entering
; standby mode, and the ring oscillator frequency is sensitive to
; the power supply voltage. We must let the gyroscope and accelerometer
; settle down also.
ld A,boot_delay  ; We want start_delay x 256 
push A           ; cycles of RCK = 32.768 kHz
pop B            ; before proceeding with execution.
pwr_up_lp:
ld A,0xFF        ; Load A with 255 to give the maximum eight bit
dly A            ; count, and wait this number of RCK periods.
dec B            ; Decremnent B until zero.
jp nz,pwr_up_lp

; Calibrate the transmit clock.
call calibrate_tck

; Set the low radio frequency for sample transmission
ld A,tx_frequency
ld (mmu_xfc),A

; Initialize the sensor.
call sensor_init

; Set interrupt timer interval and enable the timer interrupt to implement
; the sample period. The value we want to load into the interrup timer period 
; register is the sample period minus one, because the interrupt timer counts
; the value down to zero. So we load A with sample_period, then decrement. If
; sample_period is zero (0x00), the value we write is 255 (0xFF).
ld A,0xFF            ; Load A with ones
ld (mmu_irst),A      ; and reset all interrupts.
ld A,sample_period   ; Load A with the sample period,
dec A                ; and decrement to get the value
ld (mmu_itp),A       ; we write to timer period register.
ld A,0x01            ; Set bit zero of A to one and use
ld (mmu_imsk),A      ; to enable the timer interrupt.


; ------------------------------------------------------------

; The main program loop. The interrupts will be running 
; in the background, and they do all the work.

main:

ld A,(mmu_dfr)      ; Load the test point register.
or A,0x02           ; Set bit one and
ld (mmu_dfr),A      ; write to test point register.
and A,0xFD          ; Clear bit one and
ld (mmu_dfr),A      ; write to test point register.
ld A,255            ; Delay for 255 clock cycles.
dly A               ; so as to make the test pulse rare.
jp main             ; Repeat the main loop.

; ------------------------------------------------------------
