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
    ;<editor-fold defaultstate="collapsed" desc="Configuration Bits">

    CONFIG  FOSC = INTIO67 ; Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
    CONFIG  WDTEN = OFF ; Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
    CONFIG  LVP	= ON ; Low voltage programming enabled

    ;</editor-fold>
;=========================================================
;C BLOCK
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="CBlock">

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

    ENDC ; END the cblock

    ;</editor-fold>
;=========================================================
;VARIABLES
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="Variables">

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
    ;<editor-fold defaultstate="collapsed" desc="Reset Vectors">

    ORG 0x00
    GOTO SETUP ; GOTO the initial SETUP of the PIC

    ORG 0x08
    GOTO ISR ; GOTO Interrupt Service Routine
    GOTO MAIN ; Once the ISR is completed, return to MAIN

    ;</editor-fold>
;=========================================================
;SETUP BLOCK
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="Setup Block">

SETUP
    ;----------------------------------------------
    ;           OSCILLATOR SETUP
    ;----------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="Oscillator Setup">

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
    CLRF INTCON ; Clear the INTCON register

    BSF	INTCON,PEIE ; Enable peripheral interrupts
    BSF	INTCON,GIE ; Enable global interrupts
    BSF	INTCON,INT0IE ; Enable external interrupts

    ; Interrupts for USUART communication
    BCF PIR1, RC1IF ; Clear USUART RX interrupt
    BCF PIR1, TX1IF ; Clear USUART TX interrupt
    BCF	PIE1,RC1IE ; Disable USUART RX interrupt
    BCF	PIE1,TX1IE ; Disable USUART TX interrupt

    ;</editor-fold>
    ;------------------------------------------------
    ;		    USUART SETUP
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="USUART">

    CLRF    FSR0 ; Clear indirect addressing registers

    BSF TXSTA, TXEN ; Enable transmission
    BSF TXSTA, BRGH ; Enable high baud rate select

    BSF RCSTA1, SPEN ; Enable serial port
    BSF RCSTA1, CREN ; Enable continuous receive

    ; Setting the baud rate for USUART
    MOVLW D'25' ; 9600 bps baud rate
    MOVWF SPBRG1
    CLRF SPBRGH1
    BCF	BAUDCON1,BRG16

    ; For USUART RX and TX pins must be configured as inputs. The mode of the pins will automatically
    ; be controlled by the PIC
    BSF	TRISC,USUART_TX_PIN ; Set TX pin as input
    BSF	TRISC,USUART_RX_PIN ; Set RX pin as input

    ; USUART Flags
    BSF USUART_RX_ECHO_FLAG, 0 ; Enable or disable echo of received data

        ;</editor-fold>
    ;------------------------------------------------
    ;		Initialize Variables
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="Initialize Variables">


    ;</editor-fold>

    ;</editor-fold>
;=========================================================
;MAIN
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="MAIN">
MAIN
    BSF INTCON, GIE ; Enable Global interrupts
    BSF INTCON, PEIE ; Enable Peripher interrupts

    BCF PIR1, RC1IF ; Clear USUART RX interrupts
    BSF PIE1, RC1IE ; Enable USUART RX interrupts

    BCF PIR1, TX1IF ; Clear USUART TX interrupts
    BCF PIE1, TX1IE ; Disable USUART TX interrupts

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

    ;------------------------------------------------
    ;USUART_RECEIVE
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="USUART_RECEIVE">

USUART_RECEIVE
    BCF PIE1, RC1IE ; Disable RX interrupts
    MOVF RCREG1, W ; Move received byte to WREG

    BTFSS USUART_RX_ECHO_FLAG, 0 ; Check the echo flag
    GOTO USUART_RX_END

USUART_RX_ECHO ; Echo the receive data
    GOTO USUART_TRANSMIT

USUART_RX_END ; End of the USUART_RECEIVE subroutine
    BCF PIR1, RC1IF ; Clear RX interrupt
    BSF PIE1, RC1IE ; Enable RX interrupt
    BSF INTCON, GIE ; Enable Global interrupts
    BSF INTCON, PEIE ; Enable Peripheral interrupts
    RETFIE

    ;</editor-fold>

    ;------------------------------------------------
    ;USUART_TRANSMIT
    ;------------------------------------------------
    ;<editor-fold defaultstate="collapsed" desc="USUART_TRANSMIT">

USUART_TRANSMIT ; Transmit byte currently in WREG
    BCF PIE1, RC1IE ; Disable RX interrupts
    BCF PIR1, TX1IF ; Clear TX interrupts

    MOVWF TXREG1 ; Move WREG byte to TX register

WAIT_FOR_TRANSMIT ; Wait for data to be sent
    BTFSS TXSTA1, TRMT ; Check if transmission is complete
    GOTO WAIT_FOR_TRANSMIT ; Loop until complete

    BSF PIE1, RC1IE ; Enable RX interrupts
    BCF PIR1, TX1IF ; Clear TX interrupts
    RETFIE

    ;</editor-fold>

    ;------------------------------------------------
    ;USUART Error handeling
    ;------------------------------------------------
     ;<editor-fold defaultstate="collapsed" desc="USUART_ERROR_HANDELING">

    ; Overrun error
ErrSerialOverr
    BCF RCSTA1,CREN	;reset the receiver logic
    BSF RCSTA1,CREN	;enable reception again
    BSF USUART_ERROR_FLAG,0
    RETURN

    ; Framing error
ErrSerialFrame
    MOVF RCREG1,W		;discard received data that has error
    BSF	USUART_ERROR_FLAG,0
    RETURN

    ; Nothing received
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
    ;<editor-fold defaultstate="collapsed" desc="USUART RX">

    ; Error handling : overrun error
    BTFSC RCSTA1,OERR ; Check for overrun error
    BRA	ErrSerialOverr ; Handle overrun error

    ; Error handling : framing error
    BTFSC RCSTA1,FERR ; Check for framing error
    BRA	ErrSerialFrame ; Handle framing error

    ; Test if error occured
    BTFSC USUART_ERROR_FLAG,0
    BRA	EXIT_NO_RC
    ;</editor-fold>

    RETFIE

    ;</editor-fold>
    END

