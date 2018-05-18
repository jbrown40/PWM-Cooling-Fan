/*
 * lab3.0.0.asm
 *
 *  Created: 3/5/2018 
 *   Author: Jessica Brown, Skylar Chatman
 */ 



;Defining variables 
.def temp1=R16 ; temporary variable to be used for timer
.def temp2=R17 ; temporary variable to be used for timer
.def count=R18 ; counter for timer--will be loaded into timer counter later
.def timeHigh=R19 ; variable to hold time that the wave is 1
.def timeLow=R20 ; variable to hold time that the wave is 0
.def PChA=R21 ; previous value held at Channel A
.def PChB=R22 ; previous value held at Channel B
.def ChA=R23 ; Channel A value
.def ChB=R24 ; Channel B value
.set lowThreshold=38 ; 30% duty cycle (min value of duty cycle)
.set highThreshold=162 ; 70% duty cycle (max value of duty cycle)

; starting at 50% duty cycle
ldi temp1,0x02 ; will be value to clock the timer at 1/8 the system clock
out TCCR0B,temp1 ; clock the timer at 1/8 the system clock
; set timeHigh and timeLow to 50% of the duty cycle, so we have a starting point
ldi timeHigh,100
ldi timeLow,100

;Configure Pins
sbi DDRB,2 ; set PB2 as output
cbi DDRB,1 ; set PB1 as input
cbi DDRB,0 ; set PB0 as input

ldi PChA,1
sbis PINB,0
ldi PChA,0

;Main loop
main:
	sbi PORTB,2
	mov count,timeHigh
	rcall timer
	rcall read
	cbi PORTB,2
	mov count,timeLow
	rcall timer
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
	;Another check for clockwise rotation
	cp ChA,PChB
	brne clockwise_turn
	rjmp store


counterclockwise: ; checking if the RPG has been turned counterclockwise
	cp ChB,PChA
	brne counterclockwise_turn
	rjmp store

clockwise_turn:
	; mid-lab checkoff
	; sbi PORTB,2 ; turn LED off
	; rcall store ; store the old values

	;Decrement high time and increment time low
	dec timeHigh
	inc timeLow

	;Check if high time has reached minimum bound
	cpi timeHigh, lowThreshold
	brlo resetTimeHigh
	rjmp store

resetTimeHigh:
	;Reset time
	ldi timeHigh, lowThreshold
	ldi timeLow, highThreshold
	rjmp store

counterclockwise_turn:
	; mid-lab checkoff
	; cbi PORTB,2 ; turn LED on
	; rcall store ; store the old values

	;Increment high time and decrement low time
	inc timeHigh
	dec timeLow

	;Check if low time has reached minimum bound
	cpi timeLow, lowThreshold
	brlo resetTimeLow
	rjmp store

resetTimeLow:
	ldi timeHigh, highThreshold
	ldi timeLow, lowThreshold
	rjmp store

store:
	mov PChA,ChA
	mov PChB,ChB
ret

timer:
	in temp1,TCCR0B	; Save configuration from Timer/Counter Control Register B into temp variable for later use
	ldi temp2,0x00	; loading temp variable with 0
	out TCCR0B,temp2 ; Stop timer by loading Timer/Counter Control Register B with 0

	;Clear overflow flag
	in temp2,TIFR ; load Timer Counter Interupt Flag Register values into temp vairable
	sbr temp2,1<<TOV0 ; Load register value to temp variable
	out TIFR,temp2 ; now load the shifter value into the Timer Counter Interupt Flag Register

	;Start timer with new initial count
	out TCNT0,count ; load timer counter 0 (8-bit reg.) with count
	out TCCR0B,temp1 ; reload initial configuration of Timer/Counter Control Register B

	wait:
		in temp2,TIFR ; load Timer Counter Interupt Flag Register values into temp vairable
		sbrs temp2,TOV0	; check overflow flag, exit wait if set
		rjmp wait ; else wait until overflow flag is set
ret