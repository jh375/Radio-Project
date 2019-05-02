;**********************************************************************
;   This file is a basic code template for assembly code generation   *
;   on the PIC16F886. This file contains the basic code               *
;   building blocks to build upon.                                    *
;                                                                     *
;   Refer to the MPASM User's Guide for additional information on     *
;   features of the assembler (Document DS33014).                     *
;                                                                     *
;   Refer to the respective PIC data sheet for additional             *
;   information on the instruction set.                               *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Filename:	    xxx.asm                                           *
;    Date:                                                            *
;    File Version:                                                    *
;                                                                     *
;    Author:                                                          *gffgfduyiuyyuiyuyui
;    Company:                                                         *
;                                                                     *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Files Required: P16F886.INC                                      *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Notes:                                                           *
;                                                                     *
;**********************************************************************


	list	p=16f886	; list directive to define processor
	#include	<p16f886.inc>	; processor specific variable definitions


; '__CONFIG' directive is used to embed configuration data within .asm file.
; The labels following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.

	__CONFIG	_CONFIG1, _LVP_OFF & _FCMEN_ON & _IESO_OFF & _BOR_OFF & _CPD_OFF & _CP_OFF & _MCLRE_ON & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT
	__CONFIG	_CONFIG2, _WRT_OFF & _BOR21V

	cblock	0x020
  		COUNTERL
  		COUNTERH
		INDEX
		
		
	endc


;***** VARIABLE DEFINITIONS
w_temp	EQU	0x7D	; variable used for context saving
status_temp	EQU	0x7E	; variable used for context saving
pclath_temp	EQU	0x7F	; variable used for context saving

timecounter0	EQU	0x21
timecounter1	EQU	0x22
timecounter2	EQU	0x23
timecounter3	EQU	0x24
timecounter4	EQU	0x25
timecounter5	EQU	0x26
timecounter6	EQU	0x27
timecounter7	EQU	0x28

variable1	EQU	0x30
counter0	EQU	0x31
counter1	EQU	0x32
counter2	EQU	0x33
whatbutton	EQU	0x34

preset_number	EQU	0x35
preset_channel	EQU	0x36
lcd_data	EQU	0x37
preset_numberp1	EQU	0x38
preset_numberp2	EQU	0x39


;******************************************************************************

read0_H	EQU	0x40	; 0
read0_L	EQU	0x41
read1_H	EQU	0x42	; 1
read1_L	EQU	0x43
read2_H	EQU	0x44	; 2
read2_L	EQU	0x45
read3_H	EQU	0x46	; 3
read3_L	EQU	0x47
read4_H	EQU	0x48	; 4
read4_L	EQU	0x49
read5_H	EQU	0x4A	; 5
read5_L	EQU	0x4B
read6_H	EQU	0x4C	; 6
read6_L	EQU	0x4D
read7_H	EQU	0x4E
read7_L	EQU	0x4F
read8_H	EQU	0x50
read8_L	EQU	0x51
read9_H	EQU	0x52
read9_L	EQU	0x53
readA_H	EQU	0x54
readA_L	EQU	0x55



a	EQU	0x60
regnum	EQU	0x61
whichchannel	EQU	0x65	

PC	EQU	h'02'


LCD_COMMAND	EQU	0x63
LCD_LINE_COUNTER	EQU	0x64

;****************************************************************************************************************************
	ORG	0x000	; processor reset vector

	nop
  	goto	main	; go to beginning of program


	ORG	0x004	; interrupt vector location

	movwf	w_temp	; save off current W register contents
	movf	STATUS,w	; move status register into W register
	movwf	status_temp	; save off contents of STATUS register
	movf	PCLATH,w	; move pclath register into w register
	movwf	pclath_temp	; save off contents of PCLATH register

; isr code can go here or be located as a call subroutine elsewhere

	movf	pclath_temp,w	; retrieve copy of PCLATH register
	movwf	PCLATH	; restore pre-isr PCLATH register contents
	movf	status_temp,w	; retrieve copy of STATUS register
	movwf	STATUS	; restore pre-isr STATUS register contents
	swapf	w_temp,f
	swapf	w_temp,w	; restore pre-isr W register contents
	retfie		; return from interrupt





main
	BANKSEL	ANSEL		
	CLRF 	ANSEL		; set all I/O to digital
	BANKSEL 	ANSELH		
	CLRF 	ANSELH		; set all I/O to digital
	
	BANKSEL 	TRISB
	MOVLW 	b'00001111'	; set PA0,PA1,PA2,PA3 as inputs, PA4 PA5 PA6 PA7 as outputs
	MOVWF 	TRISB		; set PORTA pins as above

	BANKSEL 	TRISA
	MOVLW 	h'00'
	MOVWF 	TRISA		; set PORTC pins to digital outputs
	BANKSEL	PORTA
	MOVWF	PORTA

	MOVLW 	h'11000000'
	MOVWF 	PORTA		; pattern '0' on ssd by default
	
	clrf	a
	goto	lcd_initialisation
;*****************************************************************************STARTING UP THE LCD************************************************
lcd_initialisation								;							;
	call	initial_table							;
	movwf	PORTA	;write the initialisation stuff to PORTC				;
	call	enable							;
	incf	a							;
	movlw	d'14'	;exist when it's the 14th step					;	
	subwf	a,0							;	
	btfss	STATUS,2							;
	goto	lcd_initialisation						;	;
	clrf	a							;
									;
	goto	program_setup							;
									;
initial_table									;
	movf	a, w	;before we get here, the a variable will contain a 			;
			;certain number that we copy into the w register			;
	addwf	PC	;add this number (from a) to the program counter 			;
	retlw	b'00111000'							;
	retlw	b'00111000'							;	
	retlw	b'00111000'							;
	retlw	b'00101000'							;
	retlw	b'00100000'	;step5						;
	retlw	b'10001000'							;
	retlw	b'00001000'							;
	retlw	b'10001000'							;
	retlw	b'00000000'							;
	retlw	b'11111000'	;step10						;
	retlw	b'00001000'							;
	retlw	b'01001000'	;increment on, shift off					;
	retlw	b'00001000'							;
	retlw	b'00011000'	;step14, last step					;
									;
enable									;	
	bsf	PORTA, 2							;	
	call	delay2							;
	bcf	PORTA,2							;
	return								;									;
;************************************************************************************************************************************************


program_setup 	clrf	whatbutton
	goto	start_settings
	


start_settings
	
	goto	button_checking


button_checking	
;	call	clear_display
	movf	whatbutton, w	;copy the last digit that was pressed into the w register again, so if we dont press
			;anything, the whatbutton variable will automatically go into the w register, which 
			;at the end of this subroutine, the w will be put back into whatbutton
	BANKSEL	TRISB
	MOVLW 	b'00001111'	; set PA0,PA1,PA2,PA3 as inputs, PA4 PA5 PA6 PA7 as outputs
	MOVWF 	TRISB		; set PORTA pins as above
	BANKSEL	PORTB


	bsf	PORTB,4	;scan the FIRST row 	
	btfsc	PORTB,0	;
;	call	clear_display
	call	prepare_preset1	;goto prepare_preset1
	btfsc	PORTB,1	
	call	prepare_preset2
	btfsc	PORTB,2	
	call	prepare_preset3
	btfsc	PORTB,3	
	movlw	h'0F'
	bcf	PORTB,4	;finished scanning FIRST row
	
		
	bsf	PORTB,5	;scan the SECOND row 	
	btfsc	PORTB,0	;if logic 0 detected, this means key got pressed, then writes d'04' to w
	call	prepare_preset4	;goto prepare_preset4
	btfsc	PORTB,1	
	call	prepare_preset5	;goto prepare_preset5
	btfsc	PORTB,2	
	movlw	h'06'
	btfsc	PORTB,3	
	movlw	h'0E'
	bcf	PORTB,5	;finished scanning SECOND row
	
;	movlw	b'00101111'	;channel ONE 95.3	
;	movwf	whichchannel
;	clrf	whichchannel
	goto	button_checking

;	movwf	whatbutton	;move w register value into whatbutton variable, it should be whatever number was pressed
			;this number is used to jump to a certain retlw instruction in digitdata, which is inside displaydigit
;	return		;return back to begin

;*******************************************************************NEW LCD************************************************************************************************
;			¦¯_ ¦¯¯_ ¦¯¯ _¯¯ ¦¯¯ ¯¦¯     ¦¯¯_ _¯_ ¦¦¦ ¯¦¯ ¯ ¦_¦¦ ¦¯¯ _¯¯ 
;			¦¦¦ ¦¦¦¯ ¦¯¯ ¦¯_ ¦¯¯ ¦¦¦     ¦¦¦¯ ¦¦¦ ¦¦¦ ¦¦¦ ¦ ¦¦¯¦ ¦¯¯ ¦¯_ 
;			¦¯¦ ¯¦¯¯ ¯¯¯ ¯¯¦ ¯¯¯ ¦¯¦     ¯¦¯¯ ¦¯¦ ¦¯¦ ¦¯¦ ¯ ¯¦¦¯ ¯¯¯ ¯¯¦ 

prepare_preset1
	movlw	h'31'	;to display preset1, saves '1' into preset_number variable
	movwf	preset_numberp1
	movlw	h'11'
	movwf	preset_numberp2

;	clrf	whichchannel
	movlw	b'00100111'	;channel ONE 95.3	
	movwf	whichchannel	

	goto	start_preset
	
	return
prepare_preset2
	movlw	h'31'	;to display preset1, saves '1' into preset_number variable
	movwf	preset_numberp1
	movlw	h'21'
	movwf	preset_numberp2
	
;	clrf	whichchannel
	movlw	b'00101111'	;channel TWO 96.9
	movwf	whichchannel	
	
	goto	start_preset
	
	return

prepare_preset3
	movlw	h'31'	;to display preset1, saves '1' into preset_number variable
	movwf	preset_numberp1
	movlw	h'31'
	movwf	preset_numberp2
	
	movlw	b'01010011'	;channel THREE 104.1
	movwf	whichchannel	
	
	goto	start_preset
	
	return

prepare_preset4
	movlw	h'31'	;to display preset1, saves '1' into preset_number variable
	movwf	preset_numberp1
	movlw	h'41'
	movwf	preset_numberp2
	
	movlw	b'01010111'	;channel FOUR	104.9
	movwf	whichchannel
	
	goto	start_preset
	
	return

prepare_preset5
	movlw	h'31'	;to display preset1, saves '1' into preset_number variable
	movwf	preset_numberp1
	movlw	h'51'
	movwf	preset_numberp2
	
	movlw	b'01011111'	;channel FIVE	106.5 
	movwf	whichchannel

	goto	start_preset
	
	return


start_preset		
	call	clear_display
	call	print_preset	;print "preset"

	movf	preset_numberp1,w	;move preset channel number into W	
	call	sendLCD
	movf	preset_numberp2,w
	call	sendLCD

	movlw	h'11'
	subwf	preset_numberp2,0
	btfsc	STATUS,2	
	call	channel95.3

	movlw	h'21'
	subwf	preset_numberp2,0
	btfsc	STATUS,2	
	call	channel96.9
	
	movlw	h'31'
	subwf	preset_numberp2,0
	btfsc	STATUS,2	
	call	channel104.1

	movlw	h'41'
	subwf	preset_numberp2,0
	btfsc	STATUS,2	
	call	channel104.9

	movlw	h'51'
	subwf	preset_numberp2,0
	btfsc	STATUS,2	
	call	channel106.5

;	movf	preset_channel,	;sets the radio frequencies
;	call	start_radio
	
	call	delay500
	call	bigradio

	return


print_preset
	movlw	h'41'
	call	sendLCD
	movlw	h'31'
	call	sendLCD
	movlw	h'41'
	call	sendLCD
	movlw	h'81'
	call	sendLCD
	movlw	h'41'
	call	sendLCD
	movlw	h'11'
	call	sendLCD
	movlw	h'41'
	call	sendLCD
	movlw	h'E1'
	call	sendLCD
	movlw	h'41'
	call	sendLCD
	movlw	h'E1'
	call	sendLCD
	movlw	h'41'
	call	sendLCD
	movlw	h'51'
	call	sendLCD
	movlw	h'41'
	call	sendLCD
	movlw	h'C1'
	call	sendLCD

	return

channel95.3	movlw	b'11000000'	;start line2
	call	sendLCD
	movlw	b'00000000'	
	call	sendLCD	
	movlw	h'21'	;space
	call	sendLCD
	movlw	h'01'
	call	sendLCD
	movlw	h'21'	;space
	call	sendLCD
	movlw	h'01'
	call	sendLCD
	movlw	h'31'	;9
	call	sendLCD
	movlw	h'91'
	call	sendLCD
	movlw	h'31'	;5
	call	sendLCD
	movlw	h'51'
	call	sendLCD
	movlw	h'21'	;.
	call	sendLCD
	movlw	h'E1'
	call	sendLCD
	movlw	h'31'	;3
	call	sendLCD
	movlw	h'31'
	call	sendLCD
	return

channel96.9	movlw	b'11000000'	;start line2
	call	sendLCD
	movlw	b'00000000'	
	call	sendLCD	
	movlw	h'21'	;space
	call	sendLCD
	movlw	h'01'
	call	sendLCD
	movlw	h'21'	;space
	call	sendLCD
	movlw	h'01'
	call	sendLCD
	movlw	h'31'	;9
	call	sendLCD
	movlw	h'91'
	call	sendLCD
	movlw	h'31'	;6
	call	sendLCD
	movlw	h'61'
	call	sendLCD
	movlw	h'21'	;.
	call	sendLCD
	movlw	h'E1'
	call	sendLCD
	movlw	h'31'	;9
	call	sendLCD
	movlw	h'91'
	call	sendLCD
	return

channel104.1	movlw	b'11000000'	;start line2
	call	sendLCD
	movlw	b'00000000'	
	call	sendLCD	
	movlw	h'21'	;space
	call	sendLCD
	movlw	h'01'
	call	sendLCD
	movlw	h'31'	;1
	call	sendLCD
	movlw	h'11'
	call	sendLCD
	movlw	h'31'	;0
	call	sendLCD
	movlw	h'01'
	call	sendLCD
	movlw	h'31'	;4
	call	sendLCD
	movlw	h'41'
	call	sendLCD
	movlw	h'21'	;.
	call	sendLCD
	movlw	h'E1'
	call	sendLCD
	movlw	h'31'	;1
	call	sendLCD
	movlw	h'11'
	call	sendLCD
	return

channel104.9	movlw	b'11000000'	;start line2
	call	sendLCD
	movlw	b'00000000'	
	call	sendLCD	
	movlw	h'21'	;space
	call	sendLCD
	movlw	h'01'
	call	sendLCD
	movlw	h'31'	;1
	call	sendLCD
	movlw	h'11'
	call	sendLCD
	movlw	h'31'	;0
	call	sendLCD
	movlw	h'01'
	call	sendLCD
	movlw	h'31'	;4
	call	sendLCD
	movlw	h'41'
	call	sendLCD
	movlw	h'21'	;.
	call	sendLCD
	movlw	h'E1'
	call	sendLCD
	movlw	h'31'	;9
	call	sendLCD
	movlw	h'91'
	call	sendLCD
	return

channel106.5	movlw	b'11000000'	;start line2
	call	sendLCD
	movlw	b'00000000'	
	call	sendLCD	
	movlw	h'21'	;space
	call	sendLCD
	movlw	h'01'
	call	sendLCD
	movlw	h'31'	;1
	call	sendLCD
	movlw	h'11'
	call	sendLCD
	movlw	h'31'	;0
	call	sendLCD
	movlw	h'01'
	call	sendLCD
	movlw	h'31'	;4
	call	sendLCD
	movlw	h'61'
	call	sendLCD
	movlw	h'21'	;.
	call	sendLCD
	movlw	h'E1'
	call	sendLCD
	movlw	h'31'	;1
	call	sendLCD
	movlw	h'51'
	call	sendLCD
	return
;*******************************************************************send to LCD****************************************************************************************

sendLCD			;takes in W register, and displays on the screen
	movwf 	lcd_data	;gotta create a variable called "lcd_data"
	movf	lcd_data,w
	movwf	PORTA	
	call	enable
	
	return

finish
	nop
	goto	finish


;**********************************************************************clear display********************************************************************************
clear_display
	movlw	b'10000000'
	movwf	PORTA
	call	enable
	movlw	b'00000000'
	movwf	PORTA
	call	enable
	return
;************************************************************************GIANT RADIO*****************************************************************************************
;-----------------------------------------------------------------
; MAIN PROGRAM FOR RADIO
;-----------------------------------------------------------------
bigradio
	BANKSEL	ANSEL
	clrf	ANSEL
	clrf	ANSELH
	BANKSEL	TRISB
	clrf	TRISB	

;-----------------------------------------------------------------
; CREATE SPACE FOR RF REGISTERS IN LOCAL MEMORY
;-----------------------------------------------------------------

	BANKSEL	TRISC
	clrf	TRISC	; PORTC currently set to outputs
	BANKSEL	PORTC
	bcf	PORTC,7	; initialise RST low
	bsf	PORTC,3	; initialise SCLK high
	bcf	PORTC,4	; initialise SDIO low

	bsf		PORTC,0
	nop
	nop
	bsf	PORTC,7	; set RST to high

;local variable intialization
	BANKSEL	read0_H
	movlw	0x08
	movwf	read0_H
	BANKSEL	read0_L
	movlw	0x08
	movwf	read0_L
	BANKSEL	read1_H
	movlw	0x08
	movwf	read1_H
	BANKSEL	read1_L
	movlw	0x08
	movwf	read1_L
	BANKSEL	read2_H
	movlw	0x08
	movwf	read2_H
	BANKSEL	read2_L
	movlw	0x08
	movwf	read2_L
	BANKSEL	read3_H
	movlw	0x08
	movwf	read3_H
	BANKSEL	read3_L
	movlw	0x08
	movwf	read3_L
	BANKSEL	read4_H
	movlw	0x08
	movwf	read4_H
	BANKSEL	read4_L
	movlw	0x08
	movwf	read4_L
	BANKSEL	read5_H
	movlw	0x08
	movwf	read5_H
	BANKSEL	read5_L
	movlw	0x08
	movwf	read5_L
	BANKSEL	read6_H
	movlw	0x08
	movwf	read6_H
	BANKSEL	read6_L
	movlw	0x08
	movwf	read6_L
	BANKSEL	read7_H
	movlw	0x08
	movwf	read7_H
	BANKSEL	read7_L
	movlw	0x08
	movwf	read7_L
	BANKSEL	read8_H
	movlw	0x08
	movwf	read8_H
	BANKSEL	read8_L
	movlw	0x08
	movwf	read8_L
	BANKSEL	read9_H
	movlw	0x08
	movwf	read9_H
	BANKSEL	read9_L
	movlw	0x08
	movwf	read9_L
	BANKSEL	readA_H
	movlw	0x08
	movwf	readA_H
	BANKSEL	readA_L
	movlw	0x08
	movwf	readA_L

	BANKSEL	PORTC
	bcf	PORTC,7	; initialise RST low
	bsf	PORTC,3	; initialise SCLK high
	bcf	PORTC,4	; initialise SDIO low
	nop
	nop
	bsf	PORTC,7	; set RST to high

;	movlw	D'27'	;  92.9 MHz (92.9-87.5)/0.2
;	movwf	R_PRESET1
	
;	movlw	D'39'	;  95.3 MHz
;	movwf	R_PRESET2
;	
;	movlw	D'47'	;  96.9 MHz
;	movwf	R_PRESET3
	
;	movlw	D'71'	; 101.7 MHz
;	movwf	R_PRESET4
	
;	movlw	D'83'	; 104.1 MHz
;	movwf	R_PRESET5
	
;	movlw	D'91'	; 105.7 MHz
;	movwf	R_PRESET6

;	movlw	D'95'	; 106.5 MHz
;	movwf	R_PRESET6

;-----------------------------------------------------------------
; SET UP MASTER MODE AND CLOCK CYCLE ON PIC
;-----------------------------------------------------------------

	BANKSEL	SSPCON	; set PIC to MASTER MODE
	movlw	b'00101000'
	movwf	SSPCON

	BANKSEL	SSPADD	; set up CLOCK CYCLE
	movlw	D'20'	; change this number
	movwf	SSPADD	; set clock to 200 kHz

	BANKSEL	SSPSTAT
	bcf	SSPSTAT,6	; clock bit edge setup
	bcf	SSPSTAT,7	; slew rate control enabled

	BANKSEL	TRISC	; set up clk and data lines for communcation
	movlw	b'00011000'	; set PORTC,3,4 as inputs
	movwf	TRISC

; ------------------ CHECK INITIAL VALUES ----------------------------

; ------------------- POWER UP CRYSTAL ----------------------

wrt_rf_reg	call	rf_start_write	;writing to rf: 02->0F->00
	
	call	wait4Ack	;wait for Acknowledge
	call	i2c_idle

	movlw	b'00000000'	; reg 02h
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	movlw	b'00000000'	; set everything to reset values
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	movlw	b'00000000'	; reg 03h
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	movlw	b'00000000'
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	movlw	b'00000000'	; reg 04h
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	movlw	b'00000000'
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	movlw	b'00000000'	; reg 05h
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	movlw	b'00000000'
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	movlw	b'00000000'	; reg 06h
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	movlw	b'00000000'
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	movlw	b'10000001'	; reg 07h
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	
	call	wait4Stop
	call	delay500
		
;;;WRITE TO REGISTER 02H
wrt_rf_reg1	call	rf_start_write	;writing to rf: 02->0F->00
	
	call	wait4Ack	; wait for Acknowledge	
	call	i2c_idle

	movlw	b'01000000'	; reg 02h
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle

	movlw	b'00000001'	; reg 02l
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle

	call	delay500
	call	wait4Stop	;Generate stop

 	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ChannelStart
	call	readStore	
	call	rf_start_write
	
	call	wait4Ack	; wait for Acknowledge	
	call	i2c_idle

	movlw	b'01000000'	; reg 02h
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle

	movlw	b'00000001'	; reg 02l
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle

	movlw	b'00000000'	; reg 03h
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle

	BANKSEL	whichchannel
	movf	whichchannel,0
;	movlw	b'00100111'	; reg 03l	;channel 106.5****************************************************
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle

	movlw	b'00000000'	; reg 04h
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle

	movlw	b'00000000'	; reg 04l
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle

	movlw	b'00000000'	; reg 05h	;defult band, spacing and min threshold
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle

	movlw	b'00000111'	; reg 05l	;max volume = 0dB when VOLEXT(06h)is by defult
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle

	
	call	delay500

	call	wait4Stop
;STC CHECK
;	call	STCcheck
;read

	goto	button_checking


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;






;**************************************************DELAY ROUTINES********************************************************
;-----------------------------------------------------------------
; SUBROUTINES: RF MODULE
;-----------------------------------------------------------------

;THIS SUBROUTINE ENABLES START CONDITION(SSPCON2,0), WAITS TO CLEAR, THEN SENDS SLAVE ADDRESS TO SSPBUF IN READ MODE
rf_start_read	
	nop
	BANKSEL	SSPCON2
	bsf	SSPCON2,0	; send START condition
waitstartrF	
	btfsc	SSPCON2,0	; wait to clear
	goto	waitstartrF
	BANKSEL	SSPBUF	; sending slave address
	movlw	b'00100001'	; address + R
	movwf	SSPBUF
	return
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
rf_start_write	
	nop
	BANKSEL	SSPCON2
	bsf	SSPCON2,0	; send START condition
waitstartw	
	btfsc	SSPCON2,0
	goto	waitstartw
	BANKSEL	SSPBUF	; sending slave address
	movlw	b'00100000'	; address + W
	movwf	SSPBUF
	return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;THIS SUBROUTINE SENDS ACKNOWLEDGE BIT FROM PIC TO RF, WAITS FOR ACKNOWLEDGE BIT (SSPCON2,4) TO CLEAR AND THEN ENABLES RECEIVE MODE (SSPCON2,3)
rf_ack	
	nop
;	call	delay5
	BANKSEL	SSPCON2
	bcf	SSPCON2,5	; prepare acknowledge bit to be sent
	bsf	SSPCON2,4	; send acknowledge bit
	nop
	nop
	nop	
	nop
waitrack	btfsc	SSPCON2,4	; check if acknowledge bit is cleared
	goto	waitrack


	bsf	SSPCON2,3	; enable receive mode for master
waitstartc	btfsc	SSPCON2,3
	goto	waitstartc
;	call	delay5
	return
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;THIS SUBROUTINE ENSURES THE LINES ARE IDLE
i2c_idle
 	banksel 	SSPSTAT 	; select SFR bank
  	btfsc 	SSPSTAT,R_W 	; test if transmit is progress
 	goto 	$-1 	; module busy so wait
 	banksel 	SSPCON2 	; select SFR bank
	movf 	SSPCON2,w 	; get copy of SSPCON2 for status bits
 	andlw 	0x1F 	; mask out non-status bits
 	btfss 	STATUS,Z 	; test for zero state, if Z set, bus is idle
 	goto 	$-3 	; bus is busy so test again
 	return
 	
;Wait for acknowledge 
wait4Ack
	BANKSEL	SSPCON2
waitack	btfsc	SSPCON2,6	; test for acknowledge from slave
	goto	waitack
	return

;Generate Stop
wait4Stop
	BANKSEL	SSPCON2
	bsf	SSPCON2,2	; send STOP
waitstop	btfsc	SSPCON2,2
	goto	waitstop
	return

;enable master receive mode and wait to clear
Wait4clr
	BANKSEL	SSPCON2
	bsf	SSPCON2,3	; enable master receive mode and wait to clear
waitclear	btfsc	SSPCON2,3
	goto	waitclear
	return

;store register read towrite back
storereg			
	BANKSEL	SSPBUF	; MUST STORE REGISTERS FROM RF CHIP INTO PIC
	movf	SSPBUF,0	; move data to w register
	movwf	INDF	; move WREG data to where FSR is pointing
	call	rf_ack
	call	i2c_idle
	incf	FSR,1	; increment FSR pointer
	BANKSEL	regnum
	decfsz	regnum,1
	goto	storereg
	return

; ---------------------Read & Store SUBROUTINES------------------------
;store RF register value to local registers by using FSR
readStore	
	nop
	BANKSEL	regnum	; 32 16-bit registers
	movlw	D'31'	; (16 higher 8 bits, 16 lower 8 bits)
	movwf	regnum

	call	rf_start_read	; subroutine to initialise start condition and send address of peripheral

	call	wait4Ack	; wait for Acknowledge	
	call	i2c_idle	; wait for idle
	call	Wait4clr	; enable master receive mode and wait to clear

	movlw	0x40	
	movwf	FSR	; pointer to GPR 0x40 

	call	storereg

	call	wait4Stop
	return

;****************************************************************************************************STC CHECK;;;;;


STCcheck		
	nop
	BANKSEL	regnum	; 32 16-bit registers
	movlw	D'31'	; (16 higher 8 bits, 16 lower 8 bits)
	movwf	regnum

	call	rf_start_read	; subroutine to initialise start condition and send address of peripheral

	call	wait4Ack	; wait for Acknowledge	
	call	i2c_idle	; wait for idle
	call	Wait4clr	; enable master receive mode and wait to clear

	movlw	0x40	
	movwf	FSR	; pointer to GPR 0x40 

	call	storereg

	call	wait4Stop

	btfss	readA_H,6	; check if seeking is complete, if not keep checking until complete			
	goto	STCcheck	; write to 02h,set 02h,D8 low to clear STC 
				
	call	rf_start_write
	call	wait4Ack	; wait for Acknowledge	
	call	i2c_idle

	movf	read2_H,0	; reg 02h, set 02h,D8 low to clear STC
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle

	movf	read2_L,0	; reg 02l
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	
	movlw	b'00000000'	; reg 03h
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle
	
	movf	read3_L	; reg 03l
	BANKSEL	SSPBUF
	movwf	SSPBUF
	call	wait4Ack
	call	i2c_idle

	call	wait4Stop
	return


;****************************DELAYS******************************************************
delay0.2	movlw	D'40'	;0.2ms delay
	movwf	timecounter6
delayms2	nop
	nop
	decfsz 	timecounter6,1
	goto	delayms2
	return


delay_1ms	movlw	D'200'	;1ms delay	OG
	movwf	timecounter0
delayms	nop
	nop
	decfsz 	timecounter0,1
	goto	delayms
	return

delay2	movlw	D'2'	;2ms delay
	movwf	timecounter2
dloop2	call	delay_1ms
	decfsz	timecounter2,1
	goto	dloop2
	return

delay120	movlw	D'6'	;300ms delay
	movwf	timecounter1
dloop1	call	delay20
	decfsz	timecounter1,1
	goto	dloop1
	return

delay20	movlw	D'20'	;20ms delay
	movwf	timecounter7
dloop6
	;call	button_checking	;so button checking is performed often enough
	call	delay_1ms
	decfsz	timecounter7
	goto	dloop6
	return




delay500	movlw	D'25'	;500ms delay
	movwf	timecounter3
dloop3	call	delay20
	decfsz	timecounter3,1
	goto	dloop3
	return

delay5	movlw	D'5'	;5ms delay
	movwf	timecounter4
dloop4	call	delay_1ms
	decfsz	timecounter4,1
	goto	dloop4
	return
	
delay160	movlw	D'8'	;160ms delay
	movwf	timecounter5
dloop5	call	delay20
	decfsz	timecounter5,1
	goto	dloop5
	return
	


	END	; directive 'end of program'

