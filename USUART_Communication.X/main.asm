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

  ;</editor-fold>
;=========================================================
;ISR
;=========================================================
    ;<editor-fold defaultstate="collapsed" desc="ISR">

    ;</editor-fold>
    END

