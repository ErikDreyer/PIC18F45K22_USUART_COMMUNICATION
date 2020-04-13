;=========================================================
;IMPLEMENTAION NOTES
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="IMPLEMENTATION NOTES">

    ;------------------------------------------------
    ;USUART
    ;------------------------------------------------
    ; -> TX interrupts are disabled at all times to prevent undesired behaviour
    ; -> USUART_RX_ECHO_FLAG is used to echo all received data
    ; -> 8-byte commands will be used and testing for this length will be conducted
    ; -> using more than 8-bytes may result in undesired behaviour

    ;------------------------------------------------
    ;USUART BUFFER
    ;------------------------------------------------
    ; -> USUART uses an 8-byte buffer implemented with indirect addressing
    ; -> The first 8 bytes of the CBLOCK is reserved for this buffer

    ;------------------------------------------------
    ;USUART CLEAR BUFFER
    ;------------------------------------------------
    ; Clearing the buffer, or ensuring that it is empty is simple
    ; -> Indirect addressing is used to achieve this
    ; -> The FSR0 pointer is clear
    ; -> The lower byte of the byte is set to point to the 8th byte in the CBLOCK
    ; -> POSTDEC0 is used to clear the contents of the register that FSR0L is pointing to and then FSR0L is decremented
    ; -> The loop is repeated until FSR0L is equal to 0h, thus the start of the CBLOCK
    ; -> Resulting in the first 8-bytes to be cleared

    ;------------------------------------------------
    ;USUART TRANSMITION
    ;------------------------------------------------
    ; -> The PIC continiously loops until data is transmitted
    ; -> During this period other interrupts are disabled

    ;</editor-fold>
;=========================================================
;HEADERS
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="Headers">

    Title "USUART Communication"
    list p=PIC18F45K22
    #include "p18f45K22.inc"

    ;</editor-fold>
;=========================================================
;CONFIGURATION BITS
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="CONFIGURATION BITS">

    CONFIG  FOSC = INTIO67 ; Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
    CONFIG  WDTEN = OFF ; Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
    CONFIG  LVP	= ON ; Low voltage programming enabled

    ;</editor-fold>
;=========================================================
;C BLOCK
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="CBLOCK">

    CBLOCK 0x00 ; Start cblock at 0x00

    ;----------------------------------------------
    ; USUART Variables
    ;----------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="USUART_BUFFER">

    USUART_BUFFER_0
    USUART_BUFFER_1
    USUART_BUFFER_2
    USUART_BUFFER_3
    USUART_BUFFER_4
    USUART_BUFFER_5
    USUART_BUFFER_6
    USUART_BUFFER_7

    ;</editor-fold>

    ;<editor-fold defaultstate="collapsed" desc="USUART_FLAGS">

    USUART_ERROR_FLAG ; Flag used for checking errors that occured with USUART
    USUART_RX_ECHO_FLAG ; Flag used for echoing all received data

    ;</editor-fold>
    ;----------------------------------------------
    ; DELAY Variables
    ;----------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="DELAY VARIABLES">

    delay1
    delay2
    delay3

    ;</editor-fold>
    ;----------------------------------------------
    ; TEMP and DEBUG Variables
    ;----------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="TEMP AND DEBUG VARIABLES">

    TEMP_COUNTER

    ;</editor-fold>

    ENDC ; END the cblock

    ;</editor-fold>
;=========================================================
;VARIABLES
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="VARIABLES">

    ;----------------------------------------------
    ; USUART Variables and Flags
    ;----------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="USUART">


     ;</editor-fold>

    ;----------------------------------------------
    ; PINS
    ;----------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="PINS">

    ;----------------------------------------------
    ; USUART Pins
    ;----------------------------------------------
USUART_TX_PIN EQU RC6 ; Pin used for TX of USUART
USUART_RX_PIN EQU RC7 ; Pin usef for RX of USUART

    ;</editor-fold>

    ;</editor-fold>
;=========================================================
;RESET VECTORS
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="RESET VECTORS">

    ORG 0x00
    GOTO SETUP ; GOTO the initial SETUP of the PIC

    ORG 0x08
    GOTO ISR ; GOTO Interrupt Service Routine
    GOTO MAIN ; Once the ISR is completed, return to MAIN

    ;</editor-fold>
;=========================================================
;SETUP BLOCK
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="SETUP BLOCK">

SETUP
    ;----------------------------------------------
    ;           OSCILLATOR SETUP
    ;----------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="OSCILLATOR SETUP">

    ; Setup 4 MHz Oscillator
    BSF	OSCCON,IRCF0
    BCF	OSCCON,IRCF1
    BSF	OSCCON,IRCF2

    ;</editor-fold>

    ;------------------------------------------------
    ;		    INIT PORT A
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="Initialize Port A">

    MOVLB 0xF		; Set BSR for banked SFRs
    CLRF PORTA		; Initialize PORTA by clearing output data latches
    CLRF LATA		; Alternate method to clear output data latches
    CLRF TRISA		; clear bits for all pins
    CLRF ANSELA		; clear bits for all pins

    ;</editor-fold>
    ;------------------------------------------------
    ;		    INIT PORT B
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="Initialize Port B">

    MOVLB 0xF		; Set BSR for banked SFRs
    CLRF PORTB		; Initialize PORTB by clearing output data latches
    CLRF LATB		; Alternate method to clear output data latches
    CLRF TRISB		; clear bits for all pins
    CLRF ANSELB		; clear bits for all pins

        ;</editor-fold>
    ;------------------------------------------------
    ;		    INIT PORT C
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="Initialize Port C">

    MOVLB 0xF		; Set BSR for banked SFRs
    CLRF PORTC		; Initialize PORTC by clearing output data latches
    CLRF LATC		; Alternate method to clear output data latches
    CLRF TRISC		; clear bits for all pins
    CLRF ANSELC		; clear bits for all pins

        ;</editor-fold>
    ;------------------------------------------------
    ;		    INIT PORT D
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="Initialize Port D">

    MOVLB 0xF		; Set BSR for banked SFRs
    CLRF PORTD		; Initialize PORTD by clearing output data latches
    CLRF LATD		; Alternate method to clear output data latches
    CLRF TRISD		; clear bits for all pins
    CLRF ANSELD		; clear bits for all pins

    ;</editor-fold>
    ;------------------------------------------------
    ;		    ENABLE PERIPHERAL INTERRUPTS
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="Enable Periphiral Interrupts">

    ; Clear all
    CLRF INTCON		; Clear the INTCON register

    BSF	INTCON,PEIE	; Enable peripheral interrupts
    BSF	INTCON,GIE	; Enable global interrupts
    BSF	INTCON,INT0IE	; Enable external interrupts

    ; Interrupts for USUART communication
    BCF PIR1, RC1IF	; Clear USUART RX interrupt
    BCF PIR1, TX1IF	; Clear USUART TX interrupt
    BCF	PIE1,RC1IE	; Disable USUART RX interrupt
    BCF	PIE1,TX1IE	; Disable USUART TX interrupt

    ;</editor-fold>
    ;------------------------------------------------
    ;		    USUART SETUP
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="USUART">

    CLRF    FSR0	; Clear indirect addressing registers

    BSF TXSTA, TXEN	; Enable transmission
    BSF TXSTA, BRGH	; Enable high baud rate select

    BSF RCSTA1, SPEN	; Enable serial port
    BSF RCSTA1, CREN	; Enable continuous receive

    ; Setting the baud rate for USUART
    MOVLW D'25' ; 9600 bps baud rate
    MOVWF SPBRG1
    CLRF SPBRGH1
    BCF	BAUDCON1,BRG16

    ; For USUART RX and TX pins must be configured as inputs. The mode of the pins will automatically
    ; be controlled by the PIC
    BSF	TRISC,USUART_TX_PIN	; Set TX pin as input
    BSF	TRISC,USUART_RX_PIN	; Set RX pin as input

    ;</editor-fold>

    ;------------------------------------------------
    ; INITIALIZE VARIABLES
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="INITIALIZE VARIABLES">

    ;<editor-fold defaultstate="collapsed" desc="USUART VARIABLES">
    ;------------------------------------------------
    ; USUART FLAGS
    ;------------------------------------------------
    BSF USUART_RX_ECHO_FLAG, 0	; Enable or disable echo of received data

    ;------------------------------------------------
    ; USUART BUFFER
    ;------------------------------------------------
    CLRF FSR0 ; Clear the indirect addressing scheme
    MOVLW 8h ; Store the value of 8h in WREG
    MOVWF FSR0L ; Point FSR0L to the 8th address, the end of the USUART BUFFER
    MOVLW 0h ; MOV 0h to WREG in order to compare and skip the loop

CLEAR_USUART_BUFFER ; Set all values in the USUART BUFFER
    MOVLW 0h ; Store value of 0 in WREG
    MOVWF POSTDEC0 ; Post-decrement and MOV 0 to the INF0 register

    CPFSEQ FSR0L ; Check if the address is the first addrss 0x00 in the CBLOCK
    GOTO CLEAR_USUART_BUFFER ; Loop if not at the 0th address
    ;</editor-fold>

    ;</editor-fold>

    ;</editor-fold>
;=========================================================
;MAIN
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="MAIN">
MAIN
    BSF INTCON, GIE	; Enable Global interrupts
    BSF INTCON, PEIE	; Enable Peripher interrupts

    BCF PIR1, RC1IF	; Clear USUART RX interrupts
    BSF PIE1, RC1IE	; Enable USUART RX interrupts

    BCF PIR1, TX1IF	; Clear USUART TX interrupts
    BCF PIE1, TX1IE	; Disable USUART TX interrupts

    GOTO MAIN ; Loop MAIN

    ;</editor-fold>
;=========================================================
;DELAYS
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="DELAYS">

    ;<editor-fold defaultstate="collapsed" desc="DELAY">
delay1s
    MOVLW   0x0F
    MOVWF   delay3
Go_off0
    movlw	0xFF
    movwf	delay2
Go_off1
    movlw	0xFF
    movwf	delay1
Go_off2
    decfsz	delay1,f
    goto	Go_off2
    decfsz	delay2,f
    goto	Go_off1
    decfsz	delay3,f
    goto	Go_off0
    RETURN
;</editor-fold>

    ;</editor-fold>
;=========================================================
;USUART
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="USUART">

    ;<editor-fold defaultstate="collapsed" desc="USUART RECEIVE">

    ;------------------------------------------------
    ;USUART RECEIVE
    ;------------------------------------------------
USUART_RECEIVE
    BCF PIE1, RC1IE	; Disable RX interrupts

    CLRF FSR1	    ; Clear the indirect addressing scheme
    MOVLW 8h	    ; Store the value of 8h in WREG
    MOVWF FSR1L	    ; Point the FSR1L to the 8th byte in the CBLOCK

    CLRF FSR0	    ; Clear the indirect addressing scheme
    MOVLW 7h	    ; Store the value of 7h in WREG
    MOVWF FSR0L	    ; Point the FSR0L to the 7th byte in the CBLOCK

    MOVLW 0h	    ; MOV 0h to WREG to loop and skip

USUART_RX_SHIFT_REGISTER    ; USUART RX shift register for RX history
    MOVFF POSTDEC0, POSTDEC1	; MOV the contents of FSR0 to FSR1 and decrement

    CPFSEQ FSR1L    ; Compare with WREG, check if 0, thus start of CBLOCK
    GOTO USUART_RX_SHIFT_REGISTER ; Loop if FSR1L not ZERO

    MOVF RCREG1, W	; Move received byte to WREG
    MOVWF USUART_BUFFER_0 ; MOV the received data to the first byte in the USUART BUFFER

    BTFSS USUART_RX_ECHO_FLAG, 0    ; Check the echo flag
    GOTO USUART_RX_END

    ;------------------------------------------------
    ;USUART ECHO
    ;------------------------------------------------
USUART_RX_ECHO ; Echo the receive data
    GOTO USUART_TRANSMIT

    ;------------------------------------------------
    ;USUART RX END
    ;------------------------------------------------
USUART_RX_END ; End of the USUART_RECEIVE subroutine
    BCF PIR1, RC1IF	; Clear RX interrupt
    BSF PIE1, RC1IE	; Enable RX interrupt
    BSF INTCON, GIE	; Enable Global interrupts
    BSF INTCON, PEIE	; Enable Peripheral interrupts
    RETFIE

    ;</editor-fold>

    ;------------------------------------------------
    ;USUART_TRANSMIT
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="USUART_TRANSMIT">

USUART_TRANSMIT ; Transmit byte currently in WREG
    BCF PIE1, RC1IE	; Disable RX interrupts
    BCF PIR1, TX1IF	; Clear TX interrupts

    MOVWF TXREG1 ; Move WREG byte to TX register

    ;------------------------------------------------
    ;USUART TX WAIT FOR TRANSMIT
    ;------------------------------------------------
WAIT_FOR_TRANSMIT	    ; Wait for data to be sent
    BTFSS TXSTA1, TRMT	    ; Check if transmission is complete
    GOTO WAIT_FOR_TRANSMIT  ; Loop until complete

    BSF PIE1, RC1IE	; Enable RX interrupts
    BCF PIR1, TX1IF	; Clear TX interrupts
    RETFIE

    ;</editor-fold>

    ;------------------------------------------------
    ;USUART ERROR HANDELING
    ;------------------------------------------------
     ;<editor-fold defaultstate="collapsed" desc="USUART_ERROR_HANDELING">

    ;------------------------------------------------
    ;USUART OVERRUN ERROR
    ;------------------------------------------------
ErrSerialOverr
    BCF RCSTA1,CREN	    ; Reset the receiver logic
    BSF RCSTA1,CREN	    ; Enable reception again
    BSF USUART_ERROR_FLAG,0 ; Set the error flag
    RETURN

    ;------------------------------------------------
    ;USUART FRAMING ERROR
    ;------------------------------------------------
ErrSerialFrame
    MOVF RCREG1,W	    ; Discard received data that has error
    BSF	USUART_ERROR_FLAG,0 ; Set the error flag
    RETURN

    ;------------------------------------------------
    ;USUART NOTHING RECEIVED
    ;------------------------------------------------
EXIT_NO_RC
    CLRF    USUART_ERROR_FLAG
    CLRF    RCREG1
    RETFIE
   ;</editor-fold>

    ;</editor-fold>
;=========================================================
;ISR
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="ISR">

ISR
    ;------------------------------------------------
    ;USUART RX
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="USUART RX">

    BCF PIE1, RC1IE ; Disable RX interrupt
    BTFSC PIR1, RC1IF ; Check if the RX interrupt is set
    CALL USUART_RECEIVE ; If RX interupt is set, CALL USUART_RECEIVE

    ;</editor-fold>

    ;------------------------------------------------
    ;USUART TX
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="USUART TX">

    ; Normally the TX interrupt (PIE1, TX1IE) is disabled and is rather polled
    ; when data is transfered to the TXREG1 register in order to transmit
    ; The reason for this is for when something goes wrong and transmission
    ; doesn't behave as expected. If the interrupt is enabled when this happens,
    ; unpredictable and unexpected behaviour may occur. This comment field serves
    ; only as a reference and remider for this design choice.

    ;</editor-fold>

    ;------------------------------------------------
    ;USUART RX Error handeling
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="USUART RX ERROR">

    ;------------------------------------------------
    ;USUART OVERRUN ERROR
    ;------------------------------------------------
    BTFSC RCSTA1,OERR ; Check for overrun error
    BRA	ErrSerialOverr ; Handle overrun error

    ;------------------------------------------------
    ;USUART FRAMING ERROR
    ;------------------------------------------------
    BTFSC RCSTA1,FERR ; Check for framing error
    BRA	ErrSerialFrame ; Handle framing error

    ;------------------------------------------------
    ;USUART TEST FOR AN ERROR
    ;------------------------------------------------
    BTFSC USUART_ERROR_FLAG,0
    BRA	EXIT_NO_RC

    ;</editor-fold>

    RETFIE

    ;</editor-fold>
    END

;=========================================================
; TESTING SNIPPETS
;=========================================================

;<editor-fold defaultstate="collapsed" desc="FILL USUART SHIFT REGISTER">
    
    ; Fill USUART shift register in order to test the shifting
    ; Requires a counter variable called TEMP_COUNTER

    CLRF FSR0 ; Clear indirect addressing scheme
    MOVLW 8h ; MOV 0h to WREG
    MOVWF FSR0L ; Set the pointer to the first address in the CBLOCK

    MOVLW 8h ; MOV 0h to WREG
    MOVWF TEMP_COUNTER

FILL_SHIFT_REGISTER
    MOVWF POSTDEC0 ; MOV WREG to the pointer's register
    DECF TEMP_COUNTER
    MOVF TEMP_COUNTER, W; Decrement WREG
    BNZ FILL_SHIFT_REGISTER

;</editor-fold>