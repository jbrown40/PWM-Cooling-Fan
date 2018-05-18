;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Lab4.0.asm
;
;  Created: 3/5/2018 
; Due: 4/4/2018
;   Author: Jessica Brown, Skylar Chatman 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
.include "m88PAdef.inc"

.cseg
.org 0x00 ; PC points here after power up;
	rjmp reset ; hardware reset
.org 0x010 ; PC points here on timer 0
	rjmp tim0_ovf ; overflow interupt
.org 0x1a ; just past the last ISR vector

;create static strings in program memory 
msg1: .db "DC = ", 0x00
msg2: .db "(%)", 0x00

reset: ;setting up interrupts
	ldi r16, high(RAMEND)
	out SPH, r16
	ldi r16, low(RAMEND)
	out SPL, r16

	;interrupts
	ldi temp, 0b00001111
	sts EICRA, temp ;external interrupt control register

	ldi temp, 0b00000001 ;external interrupt mask register
	out EIMSK, temp

	ldi temp, 0b00000100 ;enable B lines
	sts PCMSK0, temp

	ldi temp, 0b00000010 ;enable D lines
	sts PCMSK2, temp

	ldi temp, 0b00000101 ;enable B and D lines
	sts PCICR, temp

	sei
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Variable definitions and I/O set-up
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; set PC0-PC3 as output for LCD
; also set E and RS lines
sbi DDRC,3 ; D7
sbi DDRC,2 ; D6
sbi DDRC,1 ; D5
sbi DDRC,0 ; D4
sbi DDRB,3 ; E
sbi DDRB,5 ; RS

;Defining variables 
;variables for timer0
.def temp=R26
.def duty=R17 ; temporary variable to be used for 8-bit timer
.def timeHigh=R18 ; variable to hold time that the wave is 1
.def timeLow=R19 ; variable to hold time that the wave is 0
.def PChA=R20 ; previous value held at Channel A
.def PChB=R21 ; previous value held at Channel B
.def ChA=R22 ; Channel A value
.def ChB=R23 ; Channel B value

;variables for LCD
.def write=R24
.def mode=R25

; timer0 set up
ldi duty, 0b11100011 ; fast PWM --> needs adjustment
out TCCR0A, duty
ldi duty, 0b00001001
out TCCR0B, duty

ldi duty, 200 ; Sets the frequency at 40KHz
out OCR0A, duty

ldi duty, 100 ; Sets the duty cycle at 50%
out OCR0B, duty

;Configure Pins
sbi DDRD,5
cbi DDRB,1 ; set PB1 as input
cbi DDRB,0 ; set PB0 as input

ldi mode,0

rcall clear ;clear LCD and prepare for init
rcall lcd_init ;init
rcall clear ;clear LCD and prepare for init

ldi R30,LOW(2*msg1)
ldi R31,HIGH(2*msg1)
rcall start
rcall main
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Interrupt Subroutines
; ext_int0, reset, tim0_ovf
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
tim0_ovf: ; timer0 overflow interrupt ISR
	push R26
	in R26,SREG ; save R25 and SREG
	push R26
	sbi PINC,5 ; toggle PORTD,5
	ldi R26,50 ; reload counter
	out TCNT0,R26
	pop R26 ; restore SREG and R25
	out SREG,R26
	pop R26
reti
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; RPG Subroutine 
; main, read, clockwise, clockwise_turn, counterclockwise,
; clounterclockwise_turn, store
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Main loop
main:
	out OCR0B, duty
	rcall delay_1ms
	rcall delay_1ms
	rcall read
	rjmp main
read:
	;Check PB0 input
	ldi ChA,1
	sbis PINB,0
	ldi ChA,0

	;Check PB1 input
	ldi ChB,1
	sbis PINB,1
	ldi ChB,0

	;Check for clockwise rotation
	cp PChA,ChB
	breq clockwise

	;Check for counterclockwise rotation
	cp PChB,ChA
	breq counterclockwise
	rjmp store
ret

clockwise: ; checking if the RPG has turned clockwise
	cp ChA,PChB
	brne clockwise_turn
	rjmp store

counterclockwise: ; checking if the RPG has been turned counterclockwise
	cp ChB,PChA
	brne counterclockwise_turn
	rjmp store

clockwise_turn:
	cpi duty,200
	brne rotate_right
ret
rotate_right:
	inc duty
	rjmp store
ret

counterclockwise_turn:
	cpi duty,1
	brne rotate_left
ret
rotate_left:
	dec duty
	rjmp store
ret

store:
	mov PChA,ChA
	mov PChB,ChB
ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Delay subroutines
; delay_1ms, delay_1us
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
delay_1ms:
ldi R28, 242
d1: dec R28
rcall delay_1us
brne d1

ldi R28, 242
d2: dec R28
rcall delay_1us
brne d2

ldi R28, 242
d3: dec R28
rcall delay_1us
brne d3
ret

delay_1us:
	nop
ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Mode A and B display routines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

modeA: ;;HOW TO MOVE TO SECOND LINE OF LCD?
	; Send the character 'A' to LCD
	;ldi mode, 1
	sbi PORTB, 5 ;set RS high
	ldi write, 'A' ;populate write variable with 'A' character
	swap write ;swap nibbles
	rcall lcd_write ;write upper four bits to LCD and strobe
	rcall delay_1ms
	swap write ;swap nibbles
	rcall lcd_write ;write lower four bits to LCD and strobe

	sbi PORTB, 5 ;set RS high
	ldi write, ':' ;populate write variable with ':' character
	swap write ;swap nibbles
	rcall lcd_write ;write upper four bits to LCD and strobe
	rcall delay_1ms
	swap write ;swap nibbles
	rcall lcd_write ;write lower four bits to LCD and strobe
	cpi mode,2
	brlt call_display_ALARM
	rcall display_OK

ret
modeB:
	;ldi mode, 0
	; Send the character 'B' to LCD
	sbi PORTB, 5 ;set RS high
	ldi write, 'B' ;populate write variable with 'B' character
	swap write ;swap nibbles
	rcall lcd_write ;write upper four bits to LCD and strobe
	rcall delay_1ms
	swap write ;swap nibbles
	rcall lcd_write ;write lower four bits to LCD and strobe
	sbi PORTB, 5 ;set RS high
	ldi write, ':' ;populate write variable with ':' character
	swap write ;swap nibbles
	rcall lcd_write ;write upper four bits to LCD and strobe
	rcall delay_1ms
	swap write ;swap nibbles
	rcall lcd_write ;write lower four bits to LCD and strobe
	;need logic for detecting low RPM
	rcall display_OK

	ret
	call_display_ALARM:
		rcall display_ALARM
ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; LCD diaply dynamic duty cycle subroutines
; delay_1ms, delay_1us
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Displaying a variable DC, subroutines
;.cseg
;.org 0x00 ; PC points here after reset

start:
rcall clear
; display “DC = “
ldi R30,LOW(2*msg1)
ldi R31,HIGH(2*msg1)
sbi PORTB, 5 ;set RS high
rcall displayCString
; display dynamic DC
ldi r25,low(256)
ldi r26,high(256)
sbi PORTB, 5
rcall displayDC
; display “ (%)”
ldi R30,LOW(2*msg2)
ldi R31,HIGH(2*msg2)
sbi PORTB, 5 ;set RS high
rcall displayCString
ret
displayCString: ; displays a static string in program memory
lpm r0,Z+
tst r0
breq done
swap r0
out PORTC,r0
rcall LCDStrobe
rcall delay_1ms
swap r0
out PORTC,r0
rcall LCDStrobe
rcall delay_1ms
rjmp displayCString

displayDString:
ld r0,Z+
tst r0
breq done
swap r0
out PORTC,r0
rcall LCDStrobe
rcall delay_1ms
swap r0
out PORTC,r0
rcall LCDStrobe
rcall delay_1ms
rjmp displayDString
done:
ret
displayDC:

.dseg
dtxt: .BYTE 5 ;Allocation of RAM to hold string
.cseg
mov dd16uL,r25 ;LSB of number to display
mov dd16uH,r26 ;MSB of number to display

ldi dv16uL,low(10) ;divisor
ldi dv16uH,high(10)  ;divisor

; store terminating for the string
ldi r20,0x00
sts dtxt+4,r20
; divide number by 10 and format remainder
rcall div16u ; remainder stored in R14
ldi r20,0x30
add r14,r20
sts dtxt+3,r14 ; store in RAM

;generate decimal point
ldi r20,0x2e
sts dtxt+2,r20 ; store in RAM
rcall div16u ; remainder stored in R14
ldi r20,0x30
add r14,r20
sts dtxt+1,r14 ; store in RAM

rcall div16u ; remainder stored in R14
ldi r20,0x30
add r14,r20
sts dtxt,r14 ; store in RAM
; continue until entire string is in place…
ldi R30,low(dtxt)
ldi R31, high(dtxt)
rcall displayDString
ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; LCD Initializing and writing subroutines
; lcd_init, lcd_write, LCDStrobe, clear
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;initialize the LCD/ prepare for writing
lcd_init:
cbi PORTB, 3 ;pull E low
cbi PORTB, 5 ;pull RS low

rcall delay_1ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms

;Write 0x03 to D7-4
ldi write, 0x03
rcall lcd_write
;wait 5ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms

;Write 0x03 to D7-4
ldi write, 0x03
rcall lcd_write
;wait 1ms
rcall delay_1ms

;Write 0x03 to D7-4
ldi write, 0x03
rcall lcd_write
;wait 1ms
rcall delay_1ms

;Write 0x02 to D7-4
ldi write, 0x02
rcall lcd_write
;wait 5ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms

;Write 0x28 to D7-4
ldi write, 0x02
rcall lcd_write
rcall delay_1ms
ldi write, 0x08
rcall lcd_write

rcall delay_1ms

;Write 0x08 to D7-4
ldi write, 0x00
rcall lcd_write
rcall delay_1ms
ldi write, 0x08
rcall lcd_write

rcall delay_1ms

;Write 0x01 to D7-4
ldi write, 0x00
rcall lcd_write
rcall delay_1ms
ldi write, 0x01
rcall lcd_write

rcall delay_1ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms

;Write 0x0C to D7-4
ldi write, 0x00
rcall lcd_write
rcall delay_1ms
ldi write, 0x0C
rcall lcd_write

rcall delay_1ms

;Write 0x06 to D7-4
ldi write, 0x00
rcall lcd_write
rcall delay_1ms
ldi write, 0x06
rcall lcd_write

;display is now ready to accept data
ret

lcd_write:
out PORTC,write ;write to D7-4
rcall LCDStrobe ;strobe E
ret 

LCDStrobe:
rcall delay_1ms
sbi PORTB,3 ;set E high
rcall delay_1ms
cbi PORTB,3 ;clear E
ret

clear:
cbi PORTB, 5 ;clear Rs
rcall delay_1ms

;clear command is 0x01
ldi write, 0x00
rcall lcd_write
rcall delay_1ms
ldi write, 0x01
rcall lcd_write

;delay 5ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms
rcall delay_1ms
ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;***************************************************************************
;*
;* "div16u" - 16/16 Bit Unsigned Division
;*
;* This subroutine divides the two 16-bit numbers 
;* "dd8uH:dd8uL" (dividend) and "dv16uH:dv16uL" (divisor). 
;* The result is placed in "dres16uH:dres16uL" and the remainder in
;* "drem16uH:drem16uL".
;*  
;* Number of words	:19
;* Number of cycles	:235/251 (Min/Max)
;* Low registers used	:2 (drem16uL,drem16uH)
;* High registers used  :5 (dres16uL/dd16uL,dres16uH/dd16uH,dv16uL,dv16uH,
;*			    dcnt16u)
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	drem16uL=r14
.def	drem16uH=r15
.def	dres16uL=r16
.def	dres16uH=r17
.def	dd16uL	=r16
.def	dd16uH	=r17
.def	dv16uL	=r18
.def	dv16uH	=r19
.def	dcnt16u	=r20

;***** Code

div16u:	clr	drem16uL	;clear remainder Low byte
	sub	drem16uH,drem16uH;clear remainder High byte and carry
	ldi	dcnt16u,17	;init loop counter
d16u_1:	rol	dd16uL		;shift left dividend
	rol	dd16uH
	dec	dcnt16u		;decrement counter
	brne	d16u_2		;if done
	ret			;    return
d16u_2:	rol	drem16uL	;shift dividend into remainder
	rol	drem16uH
	sub	drem16uL,dv16uL	;remainder = remainder - divisor
	sbc	drem16uH,dv16uH	;
	brcc	d16u_3		;if result negative
	add	drem16uL,dv16uL	;    restore remainder
	adc	drem16uH,dv16uH
	clc			;    clear carry to be shifted into result
	rjmp	d16u_1		;else
d16u_3:	sec			;    set carry to be shifted into result
	rjmp	d16u_1

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; display logic for mode switching
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
display_ALARM:
	sbi PORTB, 5 ;set RS high
	ldi write, '!' ;populate write variable with '!' character
	swap write ;swap nibbles
	rcall lcd_write ;write upper four bits to LCD and strobe
	rcall delay_1ms
	swap write ;swap nibbles
	rcall lcd_write ;write lower four bits to LCD and strobe
ret
display_OK:
	sbi PORTB, 5 ;set RS high
	ldi write, 'O' ;populate write variable with 'O' character
	swap write ;swap nibbles
	rcall lcd_write ;write upper four bits to LCD and strobe
	rcall delay_1ms
	swap write ;swap nibbles
	rcall lcd_write ;write lower four bits to LCD and strobe

	sbi PORTB, 5 ;set RS high
	ldi write, 'K' ;populate write variable with 'K' character
	swap write ;swap nibbles
	rcall lcd_write ;write upper four bits to LCD and strobe
	rcall delay_1ms
	swap write ;swap nibbles
	rcall lcd_write ;write lower four bits to LCD and strobe
ret
display_LOWRPM:
	sbi PORTB, 5 ;set RS high
	ldi write, 'R' ;populate write variable with 'R' character
	swap write ;swap nibbles
	rcall lcd_write ;write upper four bits to LCD and strobe
	rcall delay_1ms
	swap write ;swap nibbles
	rcall lcd_write ;write lower four bits to LCD and strobe

	sbi PORTB, 5 ;set RS high
	ldi write, 'P' ;populate write variable with 'P' character
	swap write ;swap nibbles
	rcall lcd_write ;write upper four bits to LCD and strobe
	rcall delay_1ms
	swap write ;swap nibbles
	rcall lcd_write ;write lower four bits to LCD and strobe
	sbi PORTB, 5 ;set RS high
	ldi write, 'M' ;populate write variable with 'M' character
	swap write ;swap nibbles
	rcall lcd_write ;write upper four bits to LCD and strobe
	rcall delay_1ms
	swap write ;swap nibbles
	rcall lcd_write ;write lower four bits to LCD and strobe

	sbi PORTB, 5 ;set RS high
	ldi write, '!' ;populate write variable with '!' character
	swap write ;swap nibbles
	rcall lcd_write ;write upper four bits to LCD and strobe
	rcall delay_1ms
	swap write ;swap nibbles
	rcall lcd_write ;write lower four bits to LCD and strobe
ret
