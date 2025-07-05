	
.equ TIMER_LOAD_ADDR, 0xFFFEC600
.equ TIMER_COUNTER_ADDR, 0xFFFEC604
.equ TIMER_CONTROL_ADDR, 0xFFFEC608
.equ TIMER_INTERRUPT_ADDR, 0xFFFEC60C

.equ HEX0_ADDR,  0xFF200020
.equ HEX1_ADDR,  0xFF200021
.equ HEX2_ADDR,  0xFF200022
.equ HEX3_ADDR,  0xFF200023
.equ HEX4_ADDR,  0xFF200030
.equ HEX5_ADDR,  0xFF200031

.equ TIMER_LOAD_1, 0x03938700 // 200 MHz times 0.30 = 60000000
.equ TIMER_LOAD_2, 0x05F5E100 // 200 MHz times 0.50 = 100,000,000
.equ TIMER_LOAD_3, 0x0BEBC200 // 200 MHz times 1
.equ TIMER_CTRL, 0x7

.equ LED_ADDR, 0xFF200000
.equ SW_ADDR, 0xFF200040

.equ PUSH_DATA_REG, 0xFF200050
.equ PUSH_INTER_REG, 0xFF200058
.equ PUSH_EDGE_REG, 0xFF20005C

.equ val_1, 0x06
.equ val_2, 0x5B
.equ val_3, 0x4F
.equ val_4, 0x66
.equ val_5, 0x6D
.equ val_A, 0x77
.equ val_C, 0x39
.equ val_d, 0x5E
.equ val_E, 0x79
.equ val_F, 0x71
//.equ val_o, 0x10 // huh???
.equ val_o, 0x5C
.equ space, 0x00

.data
PB_int_flag:	.word 0x0
tim_int_flag:	.word 0x0

message:		 .space 32 // stores the CURRENT message
message_length: .word 0 // stores the current message length
message_counter: .word 0 //  stores the current index we're at in a rotation
// ex: if oFF1CE should display 1CEoFF counter is 3.
// counter resets when it reaches the current message length

paused:				.word 0
speed:				.word 0
direction:			.word 1

office:    .byte val_o, val_F, val_F, val_1, val_C, val_E
default_length: .word 0

ecse_324: .byte val_E, val_C, val_5, val_E, space, val_3, val_2, val_4
ecse_325: .byte val_E, val_C, val_5, val_E, space, val_3, val_2, val_5
length_1: .word 8

code:	   .byte val_C, val_o, val_d, val_E
cafe:	   .byte val_C, val_A, val_F, val_E
length_2:  .word 4

space_word: .byte 0x00






.section .vectors, "ax"
B _start            // reset vector
B SERVICE_UND       // undefined instruction vector
B SERVICE_SVC       // software interrupt vector
B SERVICE_ABT_INST  // aborted prefetch vector
B SERVICE_ABT_DATA  // aborted data vector
.word 0             // unused vector
B SERVICE_IRQ       // IRQ interrupt vector
B SERVICE_FIQ       // FIQ interrupt vector


.text
.global _start

_start:
    /* Set up stack pointers for IRQ and SVC processor modes */
    MOV R1, #0b11010010      // interrupts masked, MODE = IRQ
    MSR CPSR_c, R1           // change to IRQ mode
    LDR SP, =0xFFFFFFFF - 3  // set IRQ stack to A9 on-chip memory
    /* Change to SVC (supervisor) mode with interrupts disabled */
    MOV R1, #0b11010011      // interrupts masked, MODE = SVC
    MSR CPSR, R1             // change to supervisor mode
    LDR SP, =0x3FFFFFFF - 3  // set SVC stack to top of DDR3 memory
    BL  CONFIG_GIC           // configure the ARM GIC
    // NOTE: write to the pushbutton KEY interrupt mask register
    // Or, you can call enable_PB_INT_ASM subroutine from previous task
    // to enable interrupt for ARM A9 private timer, 
    // use ARM_TIM_config_ASM subroutine
	
	LDR R0, =TIMER_LOAD_1
	LDR R1, =TIMER_CTRL
	BL ARM_TIM_config_ASM
	
	MOV R0, #0xF
	BL enable_PB_INT_ASM
	
    LDR R0, =0xFF200050      // pushbutton KEY base address
    MOV R1, #0xF             // set interrupt mask bits
    STR R1, [R0, #0x8]       // interrupt mask register (base + 8)
    // enable IRQ interrupts in the processor
    MOV R0, #0b01010011      // IRQ unmasked, MODE = SVC
    MSR CPSR_c, R0
	
	BL read_slider_switches_ASM // read initial switch state
	MOV R10, A1
	

	BL update_message
	BL update_HEX
	
	
IDLE:

	BL read_slider_switches_ASM // read switch state again
    CMP A1, R10 // poll switches
    BEQ switches_unchanged // no change, same message
    MOV R10, A1 // update switch state
	
	// subroutine check which switches are on
	BL update_message

switches_unchanged:
	LDR R0, =PB_int_flag // check interrupt for pbs
	LDR R1, [R0]
	CMP R1, #0
	BEQ pb_unchanged // now check timer
	
	
	BL pb_event // subroutine to act accordingly to pb changes
	// MOV R1, =PUSH_EDGE_REG // maybe do something
	


pb_unchanged:
	LDR R1, =tim_int_flag
	LDR R2, [R1]
	CMP R2, #0
	BEQ timer_unchanged 
	// timer interrupt below

	
	LDR R3, =paused 
    LDR R9, [R3]
    CMP R9, #1 // check if in pause state
    BEQ pause_state // dont rotate message if so
	
	BL update_LED
    BL rotate_message // else, rotate to update message pointer
    BL update_HEX // and update with new message
	
	LDR R1, =tim_int_flag
    MOV R2, #0
    STR R2, [R1]
	B timer_unchanged

pause_state:
	BL update_LED

	
	LDR R1, =tim_int_flag
    MOV R2, #0
    STR R2, [R1]
    B timer_unchanged

timer_unchanged:
	// BL update_LED // clear leds because in pause state. need to add functionality
	B IDLE // loop back



// drivers START HERE ---------------------------------
pb_event:
	PUSH {R0-R4, LR}
	TST R1, #0x8
	BNE pause

	TST R1, #0x4
	BNE reverse

	TST R1, #0x2
	BNE call_speed

	TST R1, #0x1
	BNE call_slow_down
	
	B end_pb

pause:
	LDR R0, =paused
    LDR R1, [R0]
    EOR R1, R1, #1
    STR R1, [R0]
    B end_pb

reverse:
	LDR R0, =direction
	LDR R1, [R0]
	RSB R1, R1, #0	// R1 <- 0-R1
	STR R1, [R0]
	B end_pb

call_speed:
	BL speed_up
	B end_pb
	
speed_up:
	PUSH {R0, R1, LR}
    LDR  R0, =speed  
    LDR  R1, [R0]

    CMP  R1, #0   
    BEQ  end_update_speed

    SUB  R1, R1, #1 
    STR  R1, [R0]   

    CMP  R1, #0
    BEQ  set_load1
    CMP  R1, #1
    BEQ  set_load2

call_slow_down:
	BL slow_down
	B end_pb
	
slow_down:
	PUSH {R0, R1, LR}
    LDR R0, =speed         
    LDR R1, [R0] 

    CMP R1, #2 			   // already at slowest	
    BEQ end_update_speed

    ADD R1, R1, #1        
    STR R1, [R0]


    CMP  R1, #1
    BEQ  set_load2       
    CMP  R1, #2
    BEQ  set_load3 

set_load1:
	PUSH {R0-R3, LR}
	LDR R0, =TIMER_LOAD_1
	LDR R1, =TIMER_CTRL
	BL ARM_TIM_config_ASM
	POP {R0-R3, LR}
	B end_update_speed
	
set_load2:
	PUSH {R0-R3, LR}
	LDR R0, =TIMER_LOAD_2
	LDR R1, =TIMER_CTRL
	BL ARM_TIM_config_ASM
	POP {R0-R3, LR}
	B end_update_speed
	
set_load3:
	PUSH {R0-R3, LR}
	LDR R0, =TIMER_LOAD_3
	LDR R1, =TIMER_CTRL
	BL ARM_TIM_config_ASM
	POP {R0-R3, LR}
	B end_update_speed

end_update_speed:
	BL update_LED
	POP {R0, R1, LR}
	BX LR

end_pb:
	LDR R0, =PB_int_flag
    MOV R1, #0
    STR R1, [R0]

	POP {R0-R4, LR}
	BX LR

update_HEX:
	PUSH {R0-R8, LR}
	
	LDR R3, =message
	LDR R4, =message_counter
	LDR R5, [R4]
	LDR R6, =message_length
	LDR R6, [R6]
	
	MOV R2, #0 // counter i, represents the current hex display
update_HEX_loop:
	CMP R2, #6 // done hex displays?
	BEQ update_hex_done
	
	// effective index to display calculation
	ADD   R7, R5, R2	// message counter + i => index
	// message counter represents the starting index of the message currently (HEX5)
	// i represents the current hex we're on
	

	// example
	// string office. counter = 3, i = 3
	// since counter is 3, start at letter i (index 3)
	// at the 3rd iteration, ice has already been printed
	// need to print o next which is at index 0, but counter + i = 6.
	// so 6-message_length (6) = 0 which is index of o
	// repeats until i = 6 (hex 5 done)
wrap_loop:
	CMP R7, R6
	BLT no_wrap	// R7 < message length? 
	SUB R7, R7, R6 // if its greater, subtract by the length to get the next char to print
	B wrap_loop

no_wrap:
	ADD   R8, R3, R7 // R8 <- address of message[R7] 
    LDRB  R1, [R8]   // value from memory
	
	// computing bitmask using i 
    MOV   R0, #1
    LSL   R0, R0, R2

    BL    HEX_write_ASM 

	// update counter
    ADD   R2, R2, #1 
    B     update_HEX_loop

update_hex_done:
    POP {R0-R8, LR}
    BX LR
	
update_LED:
	PUSH {R0-R4, LR}

	LDR R0, =paused
	LDR R1, [R0]
	CMP R1, #1
	BEQ clear_LED
	LDR R2, =speed
	LDR R3, [R2]

	CMP R3, #0 // highest speed, 0.3 sec
	BEQ three_led
	CMP R3, #1
	BEQ five_led
	CMP R3, #2
	BEQ ten_led

ten_led:
	LDR R0, =1023 // bit mask pattern
	BL write_LEDs_ASM
	B end

five_led:
	MOV R0, #0x1F
	BL write_LEDs_ASM
	B end
	
three_led:
	MOV R0, #0x7
	BL write_LEDs_ASM
	B end

clear_LED:
	MOV R0, #0
	LDR R1, =LED_ADDR  
    STR R0, [R1] 
    B end

end:
	POP {R0-R4, LR}
	BX LR
	
write_LEDs_ASM:
    LDR R1, =LED_ADDR  
    STR R0, [R1] 
    BX  LR


update_message:
	PUSH {R0-R12, LR}	// needed registers
	
	
	//LDR R0, =SW_ADDR	// switch state address
	//LDR R5, [R0]		// switch state value in R5
	//CMP R5, #0 			// if sw off
	//BEQ copy_default_start // mesage will be off1ce, default branch working
	
	LDR   R10, =message		// message base address pointer
    MOV   R2, #32			// message length to clear
	
clear_loop:
    CMP R2, #0		// clearing everytime we update message, ends when every bit is 0
    BEQ cleared
	MOV R3, #0
    STRB R3, [R10]
	ADD R10, R10, #1
	
    SUB R2, R2, #1
    B clear_loop
cleared:
	LDR R12, =message // pointer to message
	MOV R9, #0        // initialize message counter
	
	LDR R0, =SW_ADDR	// switch state address
	LDR R5, [R0]		// switch state value in R5
	CMP R5, #0 			// if sw off
	BEQ copy_default_start // mesage will be of	
	MOV R8, #0
	
	TST R5, #0x1
	ADDNE R8, R8, #1
	TST R5, #0x2
	ADDNE R8, R8, #1
	TST R5, #0x4
	ADDNE R8, R8, #1
	TST R5, #0x8
	ADDNE R8, R8, #1
	
	// R8 is a count for how many words there are
	// R9 is a counter for which word we're on
	

slider_0:

	TST R5, #0x1	// to check, if switch 1 is on
	BEQ slider_1 // z = 1, its not on	
	
no_space_0:
	BL copy_message0 //	else, append ecse 324 to message

	ADD R9, R9, #1

	CMP R8, #1
	BEQ slider_1
	
	CMP R9, R8

	BNE slider_1
	BL insert_space

	

slider_1:
	TST R5, #0x2 // bit 1
	BEQ slider_2 // z=1; and returned 0
	//BL copy_message1
	CMP R9, #0
	BEQ no_space_1
	BL insert_space


no_space_1:
	BL copy_message1
	ADD R9, R9, #1
	

	CMP R8, #1
	BLE slider_2
	CMP R9, R8

	BNE slider_2
	BL insert_space

slider_2:
	TST R5, #0x4	// bit 2
	BEQ slider_3
	CMP R9, #0
	BEQ no_space_2
	BL insert_space
	
no_space_2:
	BL copy_message2
	ADD R9, R9, #1
	
	CMP R8, #1
	BLE slider_3
	CMP R9, R8
	BNE slider_3
	BL insert_space
	
slider_3:
	TST R5, #0x8
	BEQ updating_done
	CMP R9, #0
	BEQ no_space_3
	BL insert_space
no_space_3:
	BL copy_message3
	ADD R9, R9, #1
	

	CMP R8, #1
	BLE updating_done
	CMP R9, R8
	BNE updating_done
	BL insert_space
	
	

updating_done:
	//LDR R2, =message

	LDR R2, =message
	SUB R6, R12, R2	// difference between current pointer and base address =>length of message
	
	//MOV R6, #13
	LDR R7, =message_length
	STR R6, [R7]
	POP {R0-R12, LR}
	BX LR



insert_space:
	PUSH {R0-R2, LR}

	LDR R0, =space_word
	LDRB R2, [R0] // space value
	STRB R2, [R12], #1 // add space into current pointer, then increment
	POP {R0-R2, LR}
	BX LR

copy_message0:
	PUSH {R0-R3, LR}
	LDR R0, =ecse_324
	//LDR R1, =message
	
	MOV R2, #8
	BL copy_loop
	POP {R0-R3, LR}
    BX LR
	
copy_message1:
	PUSH {R0-R3, LR}
	LDR R0, =ecse_325
	//LDR R1, =message // too much setting message. pointer might get messed up
	MOV R2, #8
	BL copy_loop 
	POP {R0-R3, LR} 
    BX LR
copy_message2:
	PUSH {R0-R3, LR}
	LDR R0, =code
	//LDR R1, =message // too much setting message. pointer might get messed up
	MOV R2, #4
	BL copy_loop
	POP {R0-R3, LR}
    BX LR
	
copy_message3:
	PUSH {R0-R3, LR}
	LDR R0, =cafe
	//LDR R1, =message // too much setting message. pointer might get messed up
	MOV R2, #4
	BL copy_loop
	POP {R0-R3, LR}
    BX LR

copy_default_start:
	BL copy_default
	POP {R0-R12, LR}
    BX LR
copy_default:
	PUSH {R0-R3, LR} 
	LDR R0, =office
	//LDR R11, =message
	MOV R2, #6 // initialize for loop
	BL copy_loop
	
	LDR R1, =message
    SUB R3, R12, R1
	LDR R0, =message_length

    STR R3, [R0] 
    POP {R0-R3, LR} // no more messages to keep track of
    BX LR

copy_loop:
	PUSH {R3, LR} // not pushing R0-R2 for pointers
copy_looping:
	CMP R2, #0
	BEQ copy_done
	LDRB R3, [R0], #1
	STRB R3, [R12], #1
	SUB R2, R2, #1
	B copy_looping
copy_done:
	POP {R3, LR}
	BX LR

	
rotate_message:
	PUSH {R0-R3, LR}
	
	LDR R0, =message_counter 
	LDR R1, [R0]
	
	LDR  R2, =direction 
    LDR  R2, [R2] 
	
	LDR  R3, =message_length   
    LDR  R3, [R3]           
	
	// R1 <- counter plus direction
	// basically decrementing if going in reverse
	ADD R1, R1, R2
	
	CMP R1, #0
	BLT reset_reverse
	
	CMP   R1, R3
    BGE   reset_rotate
	
	B rotate_done

	// reached end, reset counter to beginning
//    MOV  R1, R3
//   SUB  R1, R1, #1
 //   B    rotate_done
	//ADD R1, R1, #1
	//CMP R1, R2
	//BLT rotate_done // if counter < length, finished rotating
	//MOV R1, #0 // counter reached length, full wrap around done
reset_reverse:
	MOV   R1, R3
    SUB   R1, R1, #1
    B     rotate_done
reset_rotate:
    MOV  R1, #0
rotate_done:
	STR R1, [R0] // update value in memory to preserve current rotation index
	POP {R0-R3, LR}
	BX LR

read_slider_switches_ASM:

    LDR A2, =SW_ADDR
    LDR A1, [A2]
    BX  LR
	
	
HEX_clear_ASM:
	PUSH {R4-R7, LR}
    MOV R3, #0      
    MOV R2, #0
    //BL loop
    POP {R4-R7, LR}
    BX LR
	
ARM_TIM_config_ASM:
	LDR R2, =TIMER_LOAD_ADDR
	STR R0, [R2]
	
	LDR R3, =TIMER_CONTROL_ADDR
	STR R1, [R3]
	
	BX LR
ARM_TIM_read_INT_ASM:
	
	LDR R2, =TIMER_INTERRUPT_ADDR
	LDR R3, [R2]
	
	AND R0, R3, #1
	BX LR
ARM_TIM_clear_INT_ASM:
	LDR R2, =TIMER_INTERRUPT_ADDR
	
	MOV R3, #1
	STR R3, [R2]
	
	BX LR
	
enable_PB_INT_ASM:
	LDR R1, =PUSH_DATA_REG
	LDR R2, [R1]
	ORR R2, R2, R0
	STR R2, [R1]
	BX LR
	
disable_PB_INT_ASM:
	LDR R1, =PUSH_DATA_REG
	LDR R2, [R1]
	BIC R2, R2, R0
	STR R2, [R1]
	BX LR
read_PB_data_ASM: 

	LDR R0, =LED_ADDR   
    LDR R0, [R0]
    BX LR
	
PB_data_is_pressed_ASM:

	LDR R1, =PUSH_DATA_REG
    LDR R2, [R1]
    
	AND R2, R2, R0        
    CMP R2, #0
    
	MOVEQ R0, #0     
    MOVNE R0, #1
    BX LR


read_PB_edgecp_ASM:

	LDR R0, =PUSH_EDGE_REG
    LDR R0, [R0]
    BX LR

	
PB_edgecp_is_pressed_ASM:

	LDR R1, =PUSH_EDGE_REG
    LDR R2, [R1]
    
	AND R2, R2, R0
	CMP R2, #0
	
    MOVEQ R0, #0
    MOVNE R0, #1
    BX LR

PB_clear_edgecp_ASM:

	LDR R0, =PUSH_EDGE_REG
    LDR R1, [R0]
    STR R1, [R0]
    BX LR
	
HEX_write_ASM:
    PUSH {R0-R7, LR}
    MOV   R2, R1        

    CMP   R2, #0x0
    MOVEQ R1, #0x00
    CMP   R2, #0x1
    MOVEQ R1, #val_1   
    CMP   R2, #0x2
    MOVEQ R1, #val_2
    CMP   R2, #0x3
    MOVEQ R1, #val_3
    CMP   R2, #0x4
    MOVEQ R1, #val_4
    CMP   R2, #0x5
    MOVEQ R1, #val_5
    CMP   R2, #0xA   
    MOVEQ R1, #val_A
    CMP   R2, #0xC
    MOVEQ R1, #val_C
    CMP   R2, #0xD
    MOVEQ R1, #val_d
    CMP   R2, #0xE
    MOVEQ R1, #val_E
    CMP   R2, #0xF
    MOVEQ R1, #val_F
    CMP   R2, #0x10
    MOVEQ R1, #val_o

   
    MOV   R3, #0  
write_loop:
    CMP   R3, #6
    BEQ   done_write

    MOV   R4, #1
    LSL   R4, R4, R3


    TST   R0, R4
    BEQ   skip_hex

    CMP   R3, #0
    BEQ   hex5
    CMP   R3, #1
    BEQ   hex4
    CMP   R3, #2
    BEQ   hex3
    CMP   R3, #3
    BEQ   hex2
    CMP   R3, #4
    BEQ   hex1
    CMP   R3, #5
    BEQ   hex0
    B     store_hex

hex0:
    LDR   R5, =HEX0_ADDR
    B     store_hex
hex1:
    LDR   R5, =HEX1_ADDR
    B     store_hex
hex2:
    LDR   R5, =HEX2_ADDR
    B     store_hex
hex3:
    LDR   R5, =HEX3_ADDR
    B     store_hex
hex4:
    LDR   R5, =HEX4_ADDR
    B     store_hex
hex5:
    LDR   R5, =HEX5_ADDR
store_hex:
    STRB  R1, [R5]
skip_hex:
    ADD   R3, R3, #1 
    B     write_loop

done_write:
    POP   {R0-R7, LR}
    BX    LR
// drivers END HERE ---------------------------------
CONFIG_TIMER:
PUSH {LR}

    MOV R0, #29            // KEY port (Interrupt ID = 73)
    MOV R1, #1             // this field is a bit-mask; bit 0 targets cpu0
    BL CONFIG_INTERRUPT
CONFIG_GIC:
    PUSH {LR}
/* To configure the FPGA KEYS interrupt (ID 73):
* 1. set the target to cpu0 in the ICDIPTRn register
* 2. enable the interrupt in the ICDISERn register */
/* CONFIG_INTERRUPT (int_ID (R0), CPU_target (R1)); */
/* NOTE: you can configure different interrupts
   by passing their IDs to R0 and repeating the next 3 lines */
    MOV R0, #73            // KEY port (Interrupt ID = 73)
    MOV R1, #1             // this field is a bit-mask; bit 0 targets cpu0
    BL CONFIG_INTERRUPT

    MOV R0, #29            // TIMER port (Interrupt ID = 29)
    MOV R1, #1             // this field is a bit-mask; bit 0 targets cpu0
    BL CONFIG_INTERRUPT

/* configure the GIC CPU Interface */
    LDR R0, =0xFFFEC100    // base address of CPU Interface
/* Set Interrupt Priority Mask Register (ICCPMR) */
    LDR R1, =0xFFFF        // enable interrupts of all priorities levels
    STR R1, [R0, #0x04]
/* Set the enable bit in the CPU Interface Control Register (ICCICR).
* This allows interrupts to be forwarded to the CPU(s) */
    MOV R1, #1
    STR R1, [R0]
/* Set the enable bit in the Distributor Control Register (ICDDCR).
* This enables forwarding of interrupts to the CPU Interface(s) */
    LDR R0, =0xFFFED000
    STR R1, [R0]
    POP {PC}



/*
* Configure registers in the GIC for an individual Interrupt ID
* We configure only the Interrupt Set Enable Registers (ICDISERn) and
* Interrupt Processor Target Registers (ICDIPTRn). The default (reset)
* values are used for other registers in the GIC
* Arguments: R0 = Interrupt ID, N
* R1 = CPU target
*/
CONFIG_INTERRUPT:
    PUSH {R4-R5, LR}
/* Configure Interrupt Set-Enable Registers (ICDISERn).
* reg_offset = (integer_div(N / 32) * 4
* value = 1 << (N mod 32) */
    LSR R4, R0, #3    // calculate reg_offset
    BIC R4, R4, #3    // R4 = reg_offset
    LDR R2, =0xFFFED100
    ADD R4, R2, R4    // R4 = address of ICDISER
    AND R2, R0, #0x1F // N mod 32
    MOV R5, #1        // enable
    LSL R2, R5, R2    // R2 = value
/* Using the register address in R4 and the value in R2 set the
* correct bit in the GIC register */
    LDR R3, [R4]      // read current register value
    ORR R3, R3, R2    // set the enable bit
    STR R3, [R4]      // store the new register value
/* Configure Interrupt Processor Targets Register (ICDIPTRn)
* reg_offset = integer_div(N / 4) * 4
* index = N mod 4 */
    BIC R4, R0, #3    // R4 = reg_offset
    LDR R2, =0xFFFED800
    ADD R4, R2, R4    // R4 = word address of ICDIPTR
    AND R2, R0, #0x3  // N mod 4
    ADD R4, R2, R4    // R4 = byte address in ICDIPTR
/* Using register address in R4 and the value in R2 write to
* (only) the appropriate byte */
    STRB R1, [R4]
    POP {R4-R5, PC}
	BX LR

/*--- Undefined instructions --------------------------------------*/
SERVICE_UND:
    B SERVICE_UND
/*--- Software interrupts ----------------------------------------*/
SERVICE_SVC:
    B SERVICE_SVC
/*--- Aborted data reads ------------------------------------------*/
SERVICE_ABT_DATA:
    B SERVICE_ABT_DATA
/*--- Aborted instruction fetch -----------------------------------*/
SERVICE_ABT_INST:
    B SERVICE_ABT_INST
/*--- IRQ ---------------------------------------------------------*/
SERVICE_IRQ:
    PUSH {R0-R7, LR}
/* Read the ICCIAR from the CPU Interface */
    LDR R4, =0xFFFEC100
    LDR R5, [R4, #0x0C] // read from ICCIAR
/* NOTE: Check which interrupt has occurred (check interrupt IDs)
   Then call the corresponding ISR
   If the ID is not recognized, branch to UNEXPECTED
   See the assembly example provided in the DE1-SoC Computer Manual
   on page 46 */
TIMER_check:
	CMP R5, #29
	BNE KEY_check

	BL ARM_TIM_ISR
	B EXIT_IRQ
KEY_check:
    CMP R5, #73
	BNE UNEXPECTED
	
	BL KEY_ISR
	B EXIT_IRQ
UNEXPECTED:
    BNE UNEXPECTED      // if not recognized, stop here
    BL KEY_ISR
EXIT_IRQ:
/* Write to the End of Interrupt Register (ICCEOIR) */
    STR R5, [R4, #0x10] // write to ICCEOIR
    POP {R0-R7, LR}
SUBS PC, LR, #4
/*--- FIQ ---------------------------------------------------------*/
SERVICE_FIQ:
    B SERVICE_FIQ
ARM_TIM_ISR:
	LDR R0, =tim_int_flag
	LDR R1, =TIMER_INTERRUPT_ADDR
	MOV R2, #1
	
	STR R2, [R0]
	STR R2, [R1]

	B END_KEY_ISR


KEY_ISR:
    // LDR R0, =0xFF200050    // base address of pushbutton KEY port
    // LDR R1, [R0, #0xC]     // read edge capture register
	LDR R1, =PUSH_EDGE_REG     
	LDR R2, [R1]
	

	LDR R0, =PB_int_flag
	STR R2, [R0]

	MOV R2, #0xF
    STR R2, [R1]
	
	BX LR
    // LDR R0, =0xFF200020    // base address of HEX display
	
	
//CHECK_KEY0:
//    MOV R3, #0x1
//    ANDS R3, R3, R1        // check for KEY0
//    BEQ CHECK_KEY1
    // MOV R2, #0b00111111
    // STR R2, [R0]           // display "0"
//    B END_KEY_ISR
//CHECK_KEY1:
//    MOV R3, #0x2
//    ANDS R3, R3, R1        // check for KEY1
 //   BEQ CHECK_KEY2
//    MOV R2, #0b00000110
 //   STR R2, [R0]           // display "1"
 //   B END_KEY_ISR
//CHECK_KEY2:
//    MOV R3, #0x4
//    ANDS R3, R3, R1        // check for KEY2
//    BEQ IS_KEY3
//    MOV R2, #0b01011011
//    STR R2, [R0]           // display "2"
//    B END_KEY_ISR
//IS_KEY3:
 //   MOV R2, #0b01001111
//    STR R2, [R0]           // display "3"
END_KEY_ISR:
    BX LR