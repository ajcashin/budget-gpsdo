;
;		#######################################
;		##         Budget GPSDO V1.0         ##
;		#######################################
;
;   Discipline an OCXO with a filtered PWM as control voltage,
;   using input from a GPS receiver. The required GPS input is
;   a 1pps (one pulse per second) timing pulse and NMEA ( National
;   Marine Electronics Association) messages at 9600 baud. The
;   interface accepts 3.3V or 5V signal levels.
;   Status is indicated by a LED. An optional user interface
;   provides more information and some control commands via a 9600
;   baud serial connection.
;
    include "P16F1455.inc" ;include the defaults for the chip
    radix dec
; sets the config settings (oscillator type etc). Specifically,
; start assuming a high speed external clock (FOSC_ECH)
 __CONFIG _CONFIG1, _FOSC_ECH&_WDTE_OFF&_MCLRE_OFF&_CLKOUTEN_OFF&_IESO_ON
 __CONFIG _CONFIG2, _LVP_OFF&_PLLMULT_4x&_CPUDIV_NOCLKDIV
; external clock, no divisor and PLL is x4 so Fosc is 40MHz

    #define LED1 PORTA,RA4 ; port for LED
    #define PWMout PORTC,RC3 ; port for control voltage
    #define SoftRXport PORTA,RA1 ; port for NMEA data
    #define LostClk PIR2,OSFIF ; set if external clock lost
    #define MissedLimit .60 ; max contiguous missed pulses

; high and low voltages (in milliVolt) to try when calibrating.
; The desired control voltage must be between these.

    #define LowTestV 240 ; 0.24V
    #define HighTestV 3760 ; 3.76V
; adjust to byte value, 5V = 250
LowVlimit  equ LowTestV / 20 
HighVlimit equ HighTestV / 20

; configuration data location
    #define ConfigLoc	0x1FC0
; bootloader
    #define BootLoader 0x1E00
 
; some control characters for xmodem
 
    #define CharSOH 0x01
    #define CharEOT 0x04
    #define CharACK 0x06
    #define CharNAK 0x15
;
;================================
; Subroutines are in a different code page to the mainline. GoCallSubs
; sets PCLATH to the subroutine code page, so any CALL from the mainline
; is directed to the correct code page. This is the usual setting.
GoCallSubs  macro
	MOVLP	high Section10
	endm
; GoMain is used if a BRA in the mainline cannot be used because the
; jump is too far for the BRA instruction. The target destination
; will usually use GoCallSubs to reinstate the usual setting.
GoMain macro destination
     	MOVLP	high Section08
	GOTO	destination
	endm
;================================
; common area - always accesible
;================================

    udata_shr 0x70 ; always accessible
LoopC	    RES 2 ; counts TMR2 cycles (increment every 25uS)
LedBits	    RES 3 ; stores a pattern controlling LED flash
; -- flag bytes --
; Interrupts use flags in Currflags (bank 0), these are copied
; each second to Iflags
Iflags	    RES 1
    #define PeriodEnd	Iflags,0 ; set by TMR2 at end of second
				 ; cleared by mainline
    #define MissedPulse	Iflags,1 ; set if TMR1 overflows
    #define GotGSV	Iflags,2 ; a $xxGSV message decoded
    #define GotRMC	Iflags,3 ; a $xxRMC message decoded
    #define ValidFix	Iflags,4 ; $xxRMC validity = A
    #define TMR1cap	Iflags,5 ; 1ppS arrival captured
; Mainline sets and reads flags in Mflags
Mflags	    RES 1
    #define ResetLoop	Mflags,0 ; Mainline requests Loop sync
    #define CalibOK	Mflags,1 ; set if calibrated
    #define Locked	Mflags,2 ; better than 10^-9
    #define WarmedUp	Mflags,3 ; better than 10^-7
    #define DriftFlag	Mflags,4 ; set if drift adjustment made
    #define DriftDir	Mflags,5 ; direction 0 - add 1, 1 - subtract 1
; controls what is sent on the serial output
LogLev	    RES	1
    #define LogDetail	LogLev,0 ; logs every 1pps
    #define LogAdj	LogLev,1 ; logs adjustments
    #define LogNMEA	LogLev,2 ; NMEA pass through
; LastTMR1 is the value of TMR1 when last latched. The values in
; Iflags are used to determine if it is valid
LastTMR1    RES	2
GenBank	    RES	1 ; save bank for maths and prints subroutines
HisPtr	    RES 2 ; used by logging, points to adjustment data
UinBank	    RES	1 ; save bank if processing user input
; DebugSave   RES 2
; DebugCount  RES	1

;================================
; bank 0 - miscellaneous
;================================
 
bank0	udata	0x20 ;start of bank 0 general purpose registers
RXassem		RES	2 ; soft UART character assembled here
; dcount used in delays
dcount 		RES	1
SatVal		RES	1 ; number of satellites in view.
			  ; BCD From $xxGSV or 00
SatMax		RES	1 ; BCD value of best of 3 SatVal
SatLed		RES	1 ; value to display on LEDs		
; event counters - these are indexed, may not be referred to by name
; Keep size and order, unless code is changed to match
Celapsed	RES	4 ; seconds since first synch
CnoFix		RES	4 ; no fix count
CmissPulse	RES	4 ; missed pulse
Crejected	RES	4 ; pulse out of range
Csanity		RES	1 ; count of contiguous bad readings
;
; interrupts update these flags
;
CurrFlags	RES	1
; PeriodEnd is only set in the copy
    #define CuMissPulse	CurrFlags,1 ; set if TMR1 overflows
    #define CUGotGSV	CurrFlags,2 ; a $xxGSV message decoded
    #define CUGotRMC	CurrFlags,3 ; a $xxRMC message decoded
    #define CUValFix	CurrFlags,4 ; $xxRMC validity = A
    #define CUTMR1cap	CurrFlags,5 ; 1ppS arrival captured
HoldTMR1	RES	2
; drift adjustment values
DA86400		RES	3 ; constant number of seconds in a day
DAwait		RES	1 ; counts 2 full days
DAprevDate	RES	3
DAaccumPWM	RES	6
DAprevAcc	RES	6
DAincrement	RES	2
DAdriftAdj	RES	2
DAwork		RES	1
;================================
; bank 1 - table used to accumulate 1pps arrival offsets
;================================
;bank1	udata	0xA0
; accessed by linear addressing
newmods	equ	0x2050 ; array of 16 bit values
	
;================================
; bank 2 - Buffer for data sent via UART
;================================
; User interface UART send buffer 
    #define TXbufsz	D'80' ; MPASM won't resolve these unless
    #define TXbufAddr	0x120 ; specifically defined
;
bank2	udata	TXbufAddr
TXbuf		RES	TXbufsz

;================================
; bank 3 - run - UART send pointers and user input
;================================
; UART control, more
bank3	udata_ovr	0x1A0 ; start transmit/receive variables area
TXsend		RES	1   ; point to next char to send
TXnext		RES	1   ; point to next free char
CDwrloop	RES	1   ; used while writing calibration ;
CDwrCkSum	RES	1   ;   data to flash memory         ;
UsrChar		RES	1   ; keyboard input
LLsave		RES	1   ; save the log level 
UsrFlag		RES	1
    #define UsrTab	UsrFlag,0
    #define UsrCmd	UsrFlag,1
    #define Prompted	UsrFlag,2
    #define ShowHist	UsrFlag,3
Utimeout	RES	1
; logging of PWM changes
LogPWM		RES	2 ; current history entry
HistPWMptr	RES	2 ; oldest history entry
HistPWMcnt	RES	1 ; number of history entries
HistNxtPtr	RES	2 ; point to next history to display

;================================
; bank 3 - when updating the program via XMODEM
;================================
bank3	udata_ovr	0x1A0
; serial bootstrap
; Operates mainly using bank 3 as this is also the bank
; for serial ports and program memory operations
bsPass	    RES	1 ; 0 = test pass, 1 = programming pass
; Xmodem variables
bswtime	    RES 1 ; time to wait for reply (1 = approx 83ms)
bsrepeat    RES	1 ; wait this number of repeats
bsSendChr   RES	1 ; character to send (ACK or NAK)
bsEOF	    RES 1 ; end of Xmodem data
bsBlkCnt    RES	1 ; Xmodem block count
bsBlkPrev   RES	1 ; new or repeat block (0 repeat -1 new)
bsBlkSum    RES	1 ; Xmodem block checksum
bsCharPtr   RES	1 ; pointer into Xmodem block
; .hex file processing variables
bsBytesLeft RES	1 ; remaining data record characters
bsShifts    RES	1 ; used for extended address records (02, 04)
bsMemAddr   RES	3 ; program mem address
bsPrevAddr  RES	3 ; address of start of previous block
bsProgDat   RES	1 ; upper or lower part of 14 bits prog mem data 
bsRecSum    RES	1 ; Binary file (.hex) record checksum
bsTemp	    RES	1

;================================
; Other areas used by programming option
;================================
bank4	    udata   0x220
; bootstrap memory buffer
; the program memory is read, modified, and written in 32 word
; blocks. bsMemBuf holds the current block
bsMemBuf    RES	64
; The Xmodem data is processed using linear address mode, and uses
; 256 bytes starting at 0x2200. Xmodem data is transferred 128
; bytes at a time. The data is always read into the second 128
; byte block (0x2280). If, when processing a .hex record, it is
; incomplete then the partial record is moved down 128 bytes and
; the next Xmodem record is read.
    #define bsXdatHigh	0x22
;================================
; bank 4 to 6 
;================================

; bank 4 thru 6 used as linear buffer 0x2140 - 0x222F
; stores info on up to 24 adjustments of PWM
history	equ	0x2140 ; history
    #define hislength	.10
    #define hiscount	.24
lasthist equ history + ((hiscount - 1) * hislength)

;================================
; bank 7, 10, 11 - data from GPS module
;================================

; NMEA messages from the GPS module are decoded to obtain
; date, time (UTC), satellites in view, status (Active or not)
bank7	udata	0x3A0 ; start NMEA message data handler
; valid characters from software UART transferred to RXresult
RXresult	RES	1 ; valid character put here
; NMEA buffer control
NMctl		RES	1 ; determines how the next character is processed
NMbuf		RES	1 ; start of the receiving buffer data
NMptr		RES	1 ; pointer within receiving buffer for next character
NMchkSum	RES	1 ; message checksum
NMChkCnt	RES	1 ; counts the two checksum digits
NMvalidMsg	RES	1 ; pointer to a completed message
NMvalidEnd	RES	1 ; one more than last character in completed message
; Saving state while processing a character.
; Allows other interrupts to be serviced in a timely manner.
Char_Shadow	RES	8
; Saving state while processing a completed message. Allows all
; other interrupts to be serviced, including incoming characters
Msg_Shadow	RES	8
; Data extracted from NMEA messages
RXtime		RES	8 ; time from $xxRMC
RXdate		RES	8 ; date from $xxRMC
RXlimit		RES	1 ; count characters in field
MGsats		RES	2 ; satellites in view (2 characters) from $xxGSV
; used during message processing
RXcommas	RES	1 ; used to loop thru data
RXflags		RES	1 ; stop a character interrupting a character
  #define RXcharFlag	RXflags,0

; NMEA data is stored in 2 buffers of 80 characters. These buffers
; use the same high bank address for easier programming.
; Maximum received message is 82 characters including $ and checksum.
; $ and checksum are not stored so remainder fits in 80 characters.
; An unused buffer has bit 7 = 1 in the first byte (Only 7 bit ASCII
; in the message, so all message bytes have bit 7 = 0).
; The buffers are banks 10 and 11.
; NMEA message buffers. See documentation for bank 7

 #define NMEA_addr 0x520
 #define NMEA_size D'80'
bank10 udata NMEA_addr
NMEA_BUF1 RES NMEA_size
bank11 udata 0x5A0
NMEA_BUF2 RES NMEA_size

;================================
; bank 8
;================================

; values used by the mainline. The calibration data is
; used throughout. The remaining data area is overlaid,
; usage depends whether in calibration or run mode.
bank8	udata	0x420
; calibration data area
CDheader    RES	4   ; 'BGV1' Budget GPSDO Version 1
SUslope	    RES 4   ; change in PWM to get 0.25Hz change of osc
SUzeroV	    RES 3   ; calculated PWM to get exact 10MHz
SUlogLev    RES	1   ; log level from power up
SUrunLev    RES	1   ; measurement period max
SUfiller    RES 2   ; future use
SUchkSum    RES 1   ; checksum XOR of the previous 15 chars
	
    udata_ovr	0x430
; area used while storing/retrieving calibration data
CDchkCalc   RES 1   ; use while calculating checksum
CDloop	    RES	1
;
    udata_ovr	0x430
; Used during calibration
SUtime1	    RES 4 ; Timer 1 value at start of a test
SUvdiff	    RES 5 ; V difference, Hz higher and Hz lower than 10MHz
SUwork	    RES 1
SUduration  RES 2 ; how long a test ran (seconds).
SUslowV	    RES 3 ; V applied to run the oscillator slow
SUslowE	    RES 4 ; the slow frequency error 
SUfastV	    RES 3 ; V applied to run the oscillator fast
SUfastE	    RES 4 ; the fast frequency error 
SUslowfast  RES 1 ; records if a test was for slow or fast
SUediff	    RES 4 ; difference between slow and fast errors
SUavSlope   RES	4 ; the average V/Hz relationship
SUavZero    RES	4 ; the average computed V for 10MHz
SUavLoop    RES	1 ; counts the tests used for averaging
; LED control
SUledbits   RES	1 ; used to flash the tests remaining
SUledctl    RES	1 ; in binary
;    
    udata_ovr	0x430
; run time data
RTcount	    RES	3 ; count seconds since last PWM adjustment
RTwork	    RES 3 ; copy of RTcount manipulated to get levels
RTlevel	    RES	1 ; the totals level currently actioned
RTvalsum    RES	2 ; sum of the two values at the current level
RTslope	    RES	4 ; raw slope in 25ns increments
RTvalerr    RES	4 ; a calculated offset from synchronisation
; to keep arithmetic to 32 bit, slope is adjusted.
newSlope    RES	2 ; adjusted SUslope, max 12 bit + sign
newSlpAdj   RES	1 ; shifts applied on SUslope to get newSlope
newdelta    RES	4 ; measured slope as a PWM adjustment
zerodelta   RES	2 ; slope for zero cross
deltalev    RES	1 ; level at which zerodelta was stored
RTloop	    RES	1 ; used as a loop variable
RTscratch   RES	4 ; work area
RTanchor    RES	2 ; starting TMR1 value for measurements
RTflags	    RES 1 ; used for zero cross detection
 #define RTlimSign RTflags,7
 #define RTlimited RTflags,0

 ;================================
; bank 9 - arithmetic package
;================================
bank9	udata	0x4A0
;
; Maths work area all in Bank 9
;
STACK		RES	D'32'
;
; The stack can be considered as an array of 8 32-bit digits
;

; REGB is copied from the 'top' of the stack, REGA is copied from the
; 'first down'. Arithmetic is performed on REGA and REGB producing a result
; in REGA. Usually, each function copies REGA to the stack and moves the
; stack pointer down 1.
; The exception is division, where REGA has the dividend and REGB
; the remainder. Both are put back on the stack. The caller can either pop
; both results or immediately call the round function (so the divisor is
; still in REGC). The round function will round REGA then pop to get the
; rounded result at top af stack. 

REGA		RES	4
REGB		RES	4
REGC		RES	4	; used by multiply and divide

PMsave		RES	1 ; store callers BSR
MTEMP		RES	1	; work area
MCOUNT		RES	1	;  "    "
MTstackPtr	RES	1 ; Stack Pointer (low address)
PackedD		RES	5 ; result of binary to decimal conversion
AdjWork		RES     4 ; intermediate store while calculating ppb

;================================
; bank 12 - PWM data
;================================

bank12 udata 0x620 ; start of PWM control
PWM		RES	3 ; variable to set oscillator control volts
Dithr		RES	2 ; used to extend granularity of control
			  ; from PWM raw (10 bits) to 24 bits
;#################################################################
;                     E N D   D A T A
;#################################################################
; The PIC CALL and GOTO instructions have an 11 bit address, so
; are limited to a 2048 instruction boundary. The PIC16F1455 has
; memory for 8192 instructions, regarded here as 4 banks. The bank
; is determined by the register PCLATH. Instruction are located
; as follows:
; Bank 0 - starts at 0x0000
; Interrupts, startup code, fixed data such as text messages
; Bank 1 - starts at 0x0800
; Mainline processing. In general, PCLATH is set to bank 2 and
; within the mainline most branching is done by the relative
; branch BRA which does not involve PCLATH. If a branch beyond the
; limits of the BRA is required, PCLATH is temporarily set to Bank 1,
; the branch is executed by GOTO, then PCLATH set back to Bank 2.
; Bank 2 - starts at 0x1000
; Subroutines used by the mainline or by other subroutines. When
; a CALL is executed, the full address of the caller is stacked
; so a RETURN doesn't use PCLATH, and will return to the caller.
; Bank 3 - nominally starts at 0x1800
; The bootstrap loader is located at 0x1E00. 0x1800-0x1DFF currently
; not used. 0x1FC0 and above is used to store non-volatile variables
; created by the program.
; This listing not in order of ascending address. The order is:
; Mainline
; Subroutines
; Interrupts
; Startup and fixed data
; Bootstrap

	errorlevel -302 ; Disable banking error messages

	ORG 0x0000
; this is where the program starts running. Setup is in
; the default program memory. Mainline processing
; is in the bank of program memory starting at 0x0800.
    	goto	setup
;
Section08:	ORG 0x0800

;#################################################################
;                           M A I N
;#################################################################
; enter here after startup.
main:
	GoCallSubs
	CALL    PrintP ; Start Message
	data    Announce
	CALL	CRLF
;=========================================
; read calibration data from flash memory.
; Check the data is valid.
;=========================================
	BANKSEL	CDheader
; pointer into flash memory
	MOVLW	Low ConfigLoc ; Low address of calibration data
	MOVWF	FSR0L
	MOVLW	High ConfigLoc | 0x80 ; High address (program memory)
	MOVWF	FSR0H
; pointer to where data will be stored
	MOVLW	low CDheader
	MOVWF	FSR1L
	MOVLW	high CDheader
	MOVWF	FSR1H
;	
	CLRF	CDchkCalc ; clear the checksum
	MOVLW	D'16' ; move 16 characters
	MOVWF	CDloop
RBloop:
	MOVIW	FSR0++ ; get from flash
	XORWF	CDchkCalc,F ; update checksum
	MOVWI	FSR1++ ; put to ram
	DECFSZ	CDloop,F
	BRA	RBloop
	BTFSS	STATUS,Z ; still valid from XORWF
	BRA	Look4GPS ; failed checksum
; check first 4 characters are BGV1
	MOVF	CDheader,W ; Budget Gps Version 1
	XORLW	A'B'
	BTFSS	STATUS,Z
	BRA	Look4GPS ; failed header test
	MOVF	CDheader+1,W
	XORLW	A'G'
	BTFSS	STATUS,Z
	BRA	Look4GPS ; failed header test
	MOVF	CDheader+2,W
	XORLW	A'V'
	BTFSS	STATUS,Z
	BRA	Look4GPS ; failed header test
	MOVF	CDheader+3,W
	XORLW	A'1'
	BTFSS	STATUS,Z
	BRA	Look4GPS ; failed header test
;
; Calibration data passes validity test. Log it, change the
; control voltage to the one from calibration, and create
; a modified V/Hz relationship suitable for 32-bit arithmetic
;
	BSF	CalibOK
; Log the stored starting control voltage and sensitivity
	CALL	PrintP
	data	ControlV
	CALL	Push3U
	data	SUzeroV
	CALL    CV2BCD ; control volts to decimal
	MOVLW   0x67 ; 7 digits, 6 decimal places
	CALL    PrintDec ; print the control volts
	CALL	PrintP
	data	CVandSlope
	CALL	Push4
	data	SUslope ; Slope - V/Hz relationship
	CALL	PushLit
	data	D'25'
	CALL	divide
	CALL    CV2BCD ; slope to decimal
	MOVLW   0x45 ; 5 digits, 4 decimal places
	CALL    PrintDec ; print the slope
	CALL	PrintP
	data	HzPerV
; set logging level to saved level
	MOVF	SUlogLev,W
	MOVWF	LogLev
;
; replace default PWM with SUzeroV
;
	CALL	Push3U
	data	SUzeroV
	CALL	Pop3
	data	PWM
;
; create normalised signed 13 bit slope for calculation purposes
; This to account for various sensitivities
;
	CALL	Push4
	data	SUslope ; Slope - V/Hz relationship
	CALL	Pop4
	data	RTscratch
	CLRF	newSlpAdj
	BRA	normlp2
; shift right until normalised
normlp1:
	DECF	newSlpAdj,F
	ASRF	RTscratch+3,F
	RRF	RTscratch+2,F
	RRF	RTscratch+1,F
	RRF	RTscratch,F
normlp2:
; shift until RTscratch+3 is all sign bits
	MOVF	RTscratch+3,W
	BTFSS	STATUS,Z
	XORLW	0xFF
	BTFSS	STATUS,Z
	BRA	normlp1
; shift until RTscratch+2 = RTscratch+3
	MOVF	RTscratch+3,W
	XORWF	RTscratch+2,W
	BTFSS	STATUS,Z
	BRA	normlp1
; and now the high 4 bits of RTscratch+1
	MOVF	RTscratch+3,W
	XORWF	RTscratch+1,W
	ANDLW	0xF0
	BTFSS	STATUS,Z
	BRA	normlp1
; save the result
	MOVF	RTscratch,W
	MOVWF	newSlope
	MOVF	RTscratch+1,W
	MOVWF	newSlope+1
;=========================================
; Look for NMEA data from the GPS receiver.
;=========================================
; $xxRMC message should be received if the GPS module is working.
; Does not require a view of satellites.
;
; User may be running without GPS and relying on the calibration
; voltage. This is not very accurate but probably within 1Hz.
; 
; Long LED flash and wait.
Look4GPS:
	CALL	WaitAsecond
	BTFSC	GotRMC ; waiting for $xxGSV message
	BRA	GotRX ; got it
	MOVLW	0xFF
	MOVWF	LedBits ; a long flash
	MOVWF	LedBits+1 ; a long flash
	BRA	Look4GPS ; yes, flash and loop 
;=======================================================
; Got NMEA data, log No. satellites until 1pps is valid.
;=======================================================
GotRX:
	BANKSEL	SatMax
	CLRF	SatMax
	BTFSS	LogAdj
	BRA	TwelveSats  ; if not logging detail
	CALL    PrintP
	data    Receiving ; Receiving Data message
	BTFSS	LogDetail
	BRA	TwelveSats  ; if not logging detail
	CALL	PrintP
	data	SatInView ; say 'satellites in view'
; Loop waiting for GPS module indicating a valid fix.
; Log the number of satellites in view while waiting
TwelveSats:
; display 12 readings per line - whenever the counter is xxxxxx00
; it gets decremented twice, so that LED can display every 3 seconds
	MOVLW	D'16'
	MOVWF	dcount
	BTFSC	LogDetail ; CRLF every 12 lines
	CALL	CRLF
AnotherSat:
	BANKSEL	MGsats
; convert it to BCD
	SWAPF	MGsats,W
	ADDWF	MGsats+1,W
	ADDLW	0xCD ; clears off the '3's at top end of each character
	BANKSEL	SatVal
	MOVWF	SatVal
	BTFSS	LogDetail
	BRA	SkipSat
	CALL	Pspace
	CALL	Print1H ; log the value
	data	SatVal
SkipSat:
; find the most SatVal for 3 seconds 
	MOVF	SatMax,W
	SUBWF	SatVal,W ; W = SatVal - SatMax
	BTFSC	STATUS,C
	ADDWF	SatMax,F ; if SatVal=>SatMax, add diff, new SatMax
; assemble new LED pattern when dcount = xxxxxx00
	MOVLW	0x03
	ANDWF	dcount,W
	BTFSS	STATUS,Z
	BRA	SatsLed
; start of 3 second period	
	DECF	dcount,F ; flash LED every 3 seconds instead of 4
	MOVLW	0xF0 ; get high nibble of SatMax
	ANDWF	SatMax,W
	BTFSS	STATUS,Z ; assume never more than 19
	MOVLW	0xFA ; D'-6'
	ADDWF	SatMax,F
; reverse the bits, 
	BSF	STATUS,C ; flag bit, finish when SatLed = 0x01
	CLRF	SatLed
RVloop:
	RLF	SatLed,F
	RRF	SatMax,F
	MOVF	SatMax,F ; test for zero (RRF doesn't)
	BTFSS	STATUS,Z ; ? more 1 bits SatMax 
	BRA	RVloop ; yes, loop
	CLRF	SatMax ; no, make ready for next 3 readings
	BRA	SatLedF1 ; most significant bit in C already
SatsLed:
; flash at start of second
	DECF	SatLed,W ; is SatLed = 0x01 ?
	BTFSC	STATUS,Z
	BRA	NextSat ; yes, no more flash
	ASRF	SatLed,F
SatLedF1: ; entry for first flash - always; C has value
	MOVLW	0x40 ; one flash = 1
	BTFSS	STATUS,C
	MOVLW	0x90 ; two flash = 0
	MOVWF   LedBits
; flash at half second
	DECF	SatLed,W ; is SatLed = 0x01 ?
	BTFSC	STATUS,Z
	BRA	NextSat ; yes, no more flash
	ASRF	SatLed,F
	MOVLW	0x10 ; one flash = 1
	BTFSS	STATUS,C
	MOVLW	0x28 ; two flash = 0
	MOVWF   LedBits+1
;
NextSat:
; Loop until end of the next second
	CALL	WaitAsecond
	BTFSC	ValidFix ; A valid fix?
	BRA	GotFix ; yes: go to the next stage
; The GPS module may not send a $xxGSV every second.
	BTFSS	GotGSV
	BRA	NextSat 
	DECFSZ	dcount,F ; some housekeeping so there are max
	BRA	AnotherSat ; 12 satellite counts per line logged
	BRA	TwelveSats
;=======================================================
; NMEA data says 1pps is valid.
;=======================================================
GotFix:
	CALL	WaitAsecond
	BTFSS	LogAdj
	BRA	SkipFixMsg  ; if logging detail
	CALL    PrintP ; Got a fix message
	data    GPSvalid
	CALL    DateStamp ; at UTC time
; ready to start receiving 1ppS pulses
SkipFixMsg:
	BSF	ResetLoop ; TMR2 ISR doesn't run timer 1
	BANKSEL	T1CON
	BSF	T1CON,TMR1ON ; stop the timer if it is running
	BCF     PIR1,TMR1IF ; clear the interrupt flag
	BCF	PIR2,C1IF ; should not be set
; set the date for drift calculation (also in BANK 0)
	MOVLW	high RXdate
	MOVWF	FSR0H
	MOVLW	low RXdate
	MOVWF	FSR0L
	MOVF	INDF0,W
	MOVWF	DAprevDate
	MOVIW	1[FSR0]
	MOVWF	DAprevDate+1
	MOVIW	2[FSR0]
	MOVWF	DAprevDate+2
; allow comparator interrupt on high to low	
	BANKSEL	CM1CON1
	BSF	CM1CON1,C1INTN
	BANKSEL	PIE2
	BSF	PIE2,C1IE ; enable comparator interrupt
; wait until comparator resets TMR2 loop
	BTFSC	ResetLoop
	BRA	$-1
; safe to enable these, should be no pending events
	BSF     PIE1,TMR1IE ; enable TMR1 interrupt
	BSF	PIE1,TMR1GIE ; enable TMR1 gate interrupt
; decide if calibration needed
	BANKSEL	CDheader
	BTFSC	CalibOK
	BRA	StartRun
	GoMain	CalibReqd
StartRun:
	CALL	WaitAsecond ; indeterminate value for TMR1
	BRA	HardReset
;============================================
; Here if lock fails. Usually during warm up.
;============================================
RTquickAct:
    	GoCallSubs
	MOVLW	A'['
	CALL	PrintC
	CALL	Print2H
	data	RTscratch
	MOVLW	A']'
	CALL	PrintC
	MOVLW	0xFF
	MOVWF	LedBits ; a long flash
; here with a valid header
;################################################
;               R U N
;################################################
HardReset:
; Hard reset is used until the OCXO is locked to the GPS. This happens
; repeatedly while the OCXO is warming up and can't be 'captured' by
; the discipline algorithm. It should not happen once the OCXO is
; stable.
	CALL	Push3U ; reset the control voltage to the default
	data	SUzeroV
	CALL	Pop3
	data	PWM
	BCF	Locked ; not set until stability better than 10^-9
; clear the counters
	CALL	ClearCounters
; $xxRMC message should be received if the GPS module is working.
	CALL	WaitAsecond ; to get a valid TMR1 value
	MOVF	LastTMR1,W ; set the value against which all
	MOVWF	RTanchor ; subsequent TMR1 values are compared
	MOVF	LastTMR1+1,W
	MOVWF	RTanchor+1
	CLRF	RTflags
;
; Normal run loop starts here
;
NewPWM:
    	GoCallSubs
	BANKSEL	CDheader
	MOVLW	D'80'
	MOVWF	RTloop ; clear accumulators
	MOVLW	high	newmods
	MOVWF	FSR0H
	MOVLW	low	newmods
	MOVWF	FSR0L
	CLRW
	MOVWI	FSR0++
	DECFSZ	RTloop,F
	BRA	$-2
; set initial values
	CLRF	RTcount ; count of readings
	CLRF	RTcount+1
	CLRF	RTcount+2
	CLRF	deltalev
RTgetVal:
    	GoCallSubs
; drift calculations
	BANKSEL	DAaccumPWM
; has the date changed?
	MOVLW	high RXdate ; pointer to stored date
	MOVWF	FSR0H
	MOVLW	low RXdate
	MOVWF	FSR0L
	MOVF	INDF0,W
	XORWF	DAprevDate,W ; compare 3 bytes
	BTFSS	STATUS,Z
	BRA	DAnewDay
	MOVIW	1[FSR0]
	XORWF	DAprevDate+1,W
	BTFSS	STATUS,Z
	BRA	DAnewDay
	MOVIW	2[FSR0]
	XORWF	DAprevDate+2,W
	BTFSC	STATUS,Z
	BRA	DAaccum ; same date
DAnewDay:
; store the new date
	MOVF	INDF0,W
	MOVWF	DAprevDate
	MOVIW	1[FSR0]
	MOVWF	DAprevDate+1
	MOVIW	2[FSR0]
	MOVWF	DAprevDate+2
; display date change?
	BTFSC	LogAdj
	CALL	DateStamp
; need two full days before calculation
	DECFSZ	DAwait,W
	BRA	DAnotTwo
; difference between accumulated PWM
	MOVF	DAaccumPWM,W
	SUBWF	DAprevAcc,F
	MOVF	DAaccumPWM+1,W
	SUBWFB	DAprevAcc+1,F
	MOVF	DAaccumPWM+2,W
	SUBWFB	DAprevAcc+2,F
	MOVF	DAaccumPWM+3,W
	SUBWFB	DAprevAcc+3,F
	MOVF	DAaccumPWM+4,W
	SUBWFB	DAprevAcc+4,F
	MOVF	DAaccumPWM+5,W
	SUBWFB	DAprevAcc+5,F
; small enough to divide? must be 32 bit signed or less
; get the sign bit off DAprevAcc+1
	RLF	DAprevAcc+3,W
	CLRW
	ADDWFC	DAprevAcc+4,W
; if C is set, and DAprevAcc+4 is 0xFF, result is zero, C still set
; if C is clear, and DAprevAcc+4 is 0x00, result is zero, C still clear
	BTFSC	STATUS,Z
	ADDWFC	DAprevAcc+5,W
	BTFSS	STATUS,Z
	BRA	DAnext
; divide by seconds in a day
	CALL	Push4
	data	DAprevAcc
	CALL	Push3
	data	DA86400
	CALL	divide
; result is 16 bit signed or less	
	CALL	Pop4
	data	DAprevAcc+2 ; put it here, to add the zero bits
; The value returned is the difference between the two averages
; add 16 zero bits and divide by seconds in a day to get a rate
; per second to apply
	CLRF	DAprevAcc
	CLRF	DAprevAcc+1
	CALL	Push4
	data	DAprevAcc
	CALL	Push3
	data	DA86400
	CALL	divide
	CALL	Pop4
	data	DAprevAcc
; move the 2 byte value to DAincrement
	MOVF	DAprevAcc,W
	MOVWF	DAincrement
	MOVF	DAprevAcc+1,W
	MOVWF	DAincrement+1
	BRA	DAnext
DAnotTwo:
; come here if not yet two full days of data
	MOVWF	DAwait
DAnext:
; copy new to prev
	MOVF	DAaccumPWM,W
	MOVWF	DAprevAcc
	MOVF	DAaccumPWM+1,W
	MOVWF	DAprevAcc+1
	MOVF	DAaccumPWM+2,W
	MOVWF	DAprevAcc+2
	MOVF	DAaccumPWM+3,W
	MOVWF	DAprevAcc+3
	MOVF	DAaccumPWM+4,W
	MOVWF	DAprevAcc+4
	MOVF	DAaccumPWM+5,W
	MOVWF	DAprevAcc+5
; clear new
	CLRF	DAaccumPWM
	CLRF	DAaccumPWM+1
	CLRF	DAaccumPWM+2
	CLRF	DAaccumPWM+3
	CLRF	DAaccumPWM+4
	CLRF	DAaccumPWM+5
DAaccum:
; accumulate PWM for drift calculation
	MOVLW	high	PWM
	MOVWF	FSR0H
	MOVLW	low	PWM
	MOVWF	FSR0L
	MOVF	INDF0,W
	ADDWF	DAaccumPWM,F
	MOVIW	1[FSR0]
	ADDWFC	DAaccumPWM+1,F
	MOVIW	2[FSR0]
	ADDWFC	DAaccumPWM+2,F
	CLRW
	ADDWFC	DAaccumPWM+3,F
	ADDWFC	DAaccumPWM+4,F
	ADDWFC	DAaccumPWM+5,F
; implement drift
	MOVF	DAincrement,W
	ADDWF	DAdriftAdj,F
	MOVF	DAincrement+1,W
	ADDWFC	DAdriftAdj+1,F
; test C is same as sign of DAincrement. If different, change PWM
; by 1
	RRF	DAwork,F ; put C in bit 7 of DAwork
	XORWF	DAwork,W ; XOR with bit 7 of DAincrement+1 (in W)
	ANDLW	0x80
	BTFSC	STATUS,Z
	BRA	DAfinish
; add 1 or -1 to PWM. Because the last average is subtracted from the
; previous average (to save on extra instructions), the sign of the
; increment is the opposite of the compensation to be applied.
	MOVLW	0x01
	BTFSS	DAincrement+1,7
	MOVLW	0xFF
	BCF	INTCON,GIE ; don't get interrupted
	ADDWF	INDF0,F
	BTFSC	DAincrement+1,7
	CLRW
	ADDFSR	0,1
	ADDWFC	INDF0,F
	ADDFSR	0,1
	ADDWFC	INDF0,F
	BSF	INTCON,GIE ; allow interrupts
	ANDLW	0x20 ; isolate 1 bit as the sign
	IORWF	Mflags,F ; set it in mainline flags DriftDir
	BSF	DriftFlag ; and say it has been done
DAfinish:	
	BANKSEL	CDheader
; wait for the next reading.
	CALL	WaitAsecond
; something happened
; increment counters in bank 0
	MOVLW	high Celapsed ; set a pointer to the counters
	MOVWF	FSR0H
	MOVLW	low Celapsed
	MOVWF	FSR0L
	MOVLW	1 ; adds 1 to elapsed seconds
	CALL	AddCounter ; and increment FSR0
;
	MOVLW	1 ; ready to add 1 to lost fix
	BTFSC	ValidFix
	CLRW	; not lost fix, don't add 1
	CALL	AddCounter ; add 0 or 1 and increment FSR0
;
	MOVLW	1 ; ready to add 1 to missed pulse
	BTFSC	ValidFix ; if not valid fix, don't add to missed pulse
	BTFSS	MissedPulse ; Fix OK, pulse valid; don't add
	CLRW
	CALL	AddCounter ; add 0 or 1 and increment FSR0
	CLRW	; so Crejected not incremented in RTnoValue
	BTFSC	ValidFix ; no valid fix - no value
	BTFSC	MissedPulse ; skip if valid pulse
	BRA	RTnoValue
;
; the difference between readings should be small, only one
; byte is processed. The full difference is calculated in case it
; is large, causing a restart.
;
	MOVF	LastTMR1,W ; difference between anchor and current
	SUBWF	RTanchor,W
	MOVWF	RTscratch
	MOVF	LastTMR1+1,W
	SUBWFB	RTanchor+1,W
	MOVWF	RTscratch+1
; the  value should be small. If it is => 127 either ignore it or restart.
	RLF	RTscratch,W
	CLRW
	ADDWFC	RTscratch+1,W
; if C is set, and RTscratch+1 is 0xFF, result is zero
; if C is clear, and RTscratch+1 is 0x00, result is zero
	BTFSS	STATUS,Z ; -ve?
	BRA	RToutB ; out of bounds
; a valid value, reset the sanity counter
	MOVLW	MissedLimit
	MOVWI	4[FSR0]
	BRA	RTinBounds
RToutB: ; error too large
	BSF	MissedPulse ; if a pulse is missed
	MOVLW	1
	BTFSC	WarmedUp
	BRA	RTnoValue
    	GoMain	RTquickAct ; still warming up, force a restart
; add to rejected
RTnoValue:
	CALL	AddCounter
	CLRF	RTscratch ; put in a zero offset
	CLRF	RTscratch+1
	DECFSZ	INDF0,F ; test for contiguous bad readings
	BRA	RTinBounds
; too many bad readings in a row
	CALL	PrintP
	data	SigLost
	CALL	WaitAsecond
	RESET
RTinBounds:
; maybe flash the LED
	MOVLW	0x90 ; double flash until locked
	BTFSC	Locked
	MOVLW	0x80
	BTFSS	MissedPulse
	MOVWF	LedBits ; only flash if 1ppS received
	BTFSC	LogDetail
	CALL	RTlogAll ; log every 1ppS
	BCF	DriftFlag ; in case it was set
; if returning from a limit, do a zero crossing check
	BTFSS	RTlimited
	BRA	NotZcross ; not returning from a limit
	MOVF	RTscratch,W
	BTFSC	STATUS,Z
	BRA	NotZcross ; don't consider zero as a cross
	XORWF	RTflags,W ; testing against RTlimSign
	ANDLW	0x80
	BTFSC	STATUS,Z
	BRA	NotZcross
; we have a zero cross, use the last stored newdelta correction
	CLRF	RTflags
	CALL	Push2
	data	zerodelta
; right shift for saved duration
	MOVF	deltalev,W
	CALL	cvPWM
; adjustment now on stack
    	GoMain	ChangePWM
	
NotZcross:
;==========================
; Data collection algorithm
;==========================
; There are multiple data collection levels. Each level inspects
; 2 values and computes a deviation of the 1pps from ideal. The
; lowest level collects one reading per value, the next level
; collects two readings per value. level n has 2^n readings per value.
;
; Use RTcount to determine action. The count is inspected from
; the low bit, inspecting more significant bits until encountering
; a zero bit.
;
; If a zero bit is encountered, if all more significant bits are 0, then
; the value is the first so it is stored and no further action needed.
;
; Otherwise, at the current level, a stored count and the latest count
; are used to calculate a slope (deviation of frequency from target)
; and a phase error. These are compared to limits. If either limit is
; reached, a correction is applied and collection restarts. If the
; limits are not reached, the latest value is left as the stored value.
;
; When a 1 bit is encountered, the two values are added and passed
; to the next level.
;
; If the levels reach a preset maximum level, a correction is
; applied and a new collection begins.
	MOVF	RTcount,W ; RTcount copied to RTwork
	MOVWF	RTwork ; then RTwork used to test bits
	MOVF	RTcount+1,W
	MOVWF	RTwork+1
	MOVF	RTcount+2,W
	MOVWF	RTwork+2
	MOVLW	-2
	MOVWF	RTlevel
; RTlevel increments by 2 each loop. This allows left shift
; (i.e. level*4) to index the limits, no shift (i.e. level*2)
; to index the counters, right shift to get the actual level number.
RTnextLev:
	MOVLW	2
	ADDWF	RTlevel,F
	ASRF	RTlevel,W	
	MOVF	RTlevel,W ; get level * 2
; create an index into the array of values
	ADDLW	low newmods
	MOVWF	FSR0L
	MOVLW	high newmods
	MOVWF	FSR0H
; swap the old and new values
	MOVIW	++FSR0
	XORWF	RTscratch+1,W
	XORWF	INDF0,F
	XORWF	RTscratch+1,F
	MOVIW	--FSR0
	XORWF	RTscratch,W
	XORWF	INDF0,F
	XORWF	RTscratch,F
; Test to see if this is the first value for the level
	MOVF	RTwork,W
	IORWF	RTwork+1,W
	IORWF	RTwork+2,W
	BTFSC	STATUS,Z
; It is the first and the field was just filled in. Finished
    	BRA	UnderLim
TwoVal:
; add two values for later use
	MOVF	INDF0,W
	ADDWF	RTscratch,W
	MOVWF	RTvalsum
	MOVIW	1[FSR0]
	ADDWFC	RTscratch+1,W
	MOVWF	RTvalsum+1
; subtract to get slope (frequency change)
	MOVF	RTscratch,W
	SUBWF	INDF0,W
	MOVWF	RTslope
	ADDFSR	0,1
	MOVF	RTscratch+1,W
	SUBWFB	INDF0,W
	MOVWF	RTslope+1
; phase error at this time = error at mid point + slope
; calculate double to avoid rounding
	MOVWF	RTscratch+1 ; double the slope
	ASLF	RTslope,W
	RLF	RTscratch+1,F
; add in the sum of two values
	ADDWF	RTvalsum,W
	MOVWF	RTvalerr
	MOVWF	RTscratch
	MOVF	RTvalsum+1,W
	ADDWFC	RTscratch+1,W
	MOVWF	RTvalerr+1
	MOVWF	RTscratch+1
; test phase error against limit
; is it negative
	BTFSS	RTscratch+1,7
	BRA	RTphaseLim
; make it absolute
	COMF	RTscratch,F
	COMF	RTscratch+1,F
	INCFSZ	RTscratch,F
	BRA	RTphaseLim
	INCF	RTscratch+1,F
RTphaseLim:
; get index into limit array
	ASLF	RTlevel,W ; get level * 4
	ADDLW	low LimTbl
	MOVWF	FSR0L
	MOVLW	high LimTbl
	MOVWF	FSR0H
	BTFSC	STATUS,C
	INCF	FSR0H,F
; subtract the limit
	MOVF	INDF0,W
	SUBWF	RTscratch,W
	MOVIW	1[FSR0]
	SUBWFB	RTscratch+1,W
	BTFSC	STATUS,C ; skip if under limit
	BRA	correctit
; passes the phase test, now the frequency error.
; if the phase error and frequency error are opposite,
; the phase error is being corrected. If so, don't test
	MOVF	RTslope+1,W
	XORWF	RTvalerr+1,W
	ANDLW	0x80
	BTFSS	STATUS,Z ; same, do the test
	BRA	RTpasstest ; different, skip the test
	MOVF	RTslope,W
	MOVWF	RTscratch
	MOVF	RTslope+1,W
	MOVWF	RTscratch+1
; is it negative
	BTFSS	RTscratch+1,7
	BRA	RTfreqLim
; make it absolute
	COMF	RTscratch,F
	COMF	RTscratch+1,F
	INCFSZ	RTscratch,F
	BRA	RTfreqLim
	INCF	RTscratch+1,F
RTfreqLim:
; subtract the limit
	MOVIW	2[FSR0]
	SUBWF	RTscratch,W
	MOVIW	3[FSR0]
	SUBWFB	RTscratch+1,W
	BTFSC	STATUS,C ; skip if under limit
	BRA	correctit
; passed both tests. Move RTvalsum (the sum) to RTscratch
; in anticipation of going to another level
RTpasstest:
	MOVF	RTvalsum,W
	MOVWF	RTscratch
	MOVF	RTvalsum+1,W
	MOVWF	RTscratch+1
; test for next action
	LSRF	RTwork+2,F
	RRF	RTwork+1,F
	RRF	RTwork,F ; drops least significant bit into C
	BTFSC	STATUS,C
	BRA	RTnextLev ; = 1: go to process next level
; C = 0, this could be the highest level so far
; if so, replace the delta used for zero cross
	MOVF	deltalev,W
	SUBWF	RTlevel,W ; RTlevel-deltalevel
	BTFSS	STATUS,C
	BRA	UnderLim
; equal or greater - replace
	MOVF	RTlevel,W
	MOVWF	deltalev
	MOVF	RTslope,W
	MOVWF	zerodelta
	MOVF	RTslope+1,W
	MOVWF	zerodelta+1
UnderLim:
;=================================
; current values not over limit
;=================================
; if we get to level 4, assume the TMR2 loop is locked onto
; correct 1ppS and the system is warmed up enough that it is
; under control from here on. The comparator interrupt is no
; longer needed (and may be caused by a spurious spike so
; should be ignored). Also set a flag so large deviations
; are treated as 'out of bounds' and ignored.
	ASRF	RTlevel,W
	XORLW	D'4' ; is it pass 4?
	BTFSS	STATUS,Z
	BRA	LockLevel
	BSF	WarmedUp ; is pass 4, set the flag
	BANKSEL	PIE2
	BCF	PIE2,C1IE ; disable comparator interrupt
	BANKSEL	CDheader
; if we get to level 7, got 128 samples deviation less than 1 cycle,
; set the locked flag
LockLevel:
	XORLW	D'7' ^ D'4'
	BTFSC	STATUS,Z
	BSF	Locked
; if we completed the maximum time, force a correction
	ASRF	RTlevel,W
	XORWF	SUrunLev,W
	BTFSC	STATUS,Z
	BRA	correctit
; Add 1 to the count
	CLRW
	BSF	STATUS,C
	ADDWFC	RTcount,F
	ADDWFC	RTcount+1,F
	ADDWFC	RTcount+2,F
    	GoMain	RTgetVal
;	
correctit:
;=====================================================
; current deviation over limit or scheduled correction
;=====================================================
; Correct for phase error from time of pps,
; and rate of change (frequency error)
; set up a  correction to return to zero phase
	CALL	Push2
	data	RTvalerr
; right shift for current duration
	ASRF	RTlevel,W
	ADDLW	1 ; extra shift, values are * 2
	CALL	cvPWM
; adjustment now on stack, return in one second
; return over the greater of the same period as it took
; to get to limit, or 8 seconds
	MOVF	RTcount,W
	ANDLW	0xF8 ; delete lower 3 bits
	IORWF	RTcount+1,W
	IORWF	RTcount+2,W
	BTFSC	STATUS,Z
	BRA	Under8
	CALL	Push3
	data	RTcount
	BRA	GotIt
Under8:
	CALL	PushLit
	data	8
GotIt:
	CALL	divide
; add in the frequency correction
	CALL	Push2
	data	RTslope
; have to double shift	
	MOVF	RTlevel,W
	CALL	cvPWM
	CALL	addm
; set up data for zero cross 
	MOVF	RTvalerr+1,W ; pick up the error sign
	ANDLW	0x80
	MOVWF	RTflags
; no zero cross test after maximum time
	ASRF	RTlevel,W
	XORWF	SUrunLev,W ; is it maximum level?
	BTFSS	STATUS,Z 
; set flag for zero cross
	BSF	RTlimited
ChangePWM:
    	GoCallSubs
	CLRF	deltalev
	CALL	Pop4 ; save the change for logging
	data	RTscratch
	CALL	Push4
	data	RTscratch
	CALL	Push3U
	data	PWM
 	CALL	addm
	CALL	Pop4
	data	PackedD ; save the new PWM for logging
; test if the PWM is within expected limits
	BANKSEL	PackedD
	MOVF	PackedD+3,F ; should be zero
	BTFSS	STATUS,Z
	BRA	BeyondLim
	MOVLW	LowVlimit
	SUBWF	PackedD+2,W ; Is PackedD+2=>LowVlimit
	BTFSS	STATUS,C  ; if yes, within limit
	BRA	BeyondLim
	MOVLW	HighVlimit
	SUBWF	PackedD+2,W ; Is PackedD+2=>HighVlimit
	BTFSS	STATUS,C ; if yes, beyond limit
	BRA	InsideLim
BeyondLim:
    	GoMain	RTquickAct ; force a restart
InsideLim:	
	CALL	Push3U
	data	PackedD
	CALL	Pop3
	data	PWM
; flash to show PWM change
	MOVLW	0xFF
	MOVWF	LedBits
;	MOVWF	LedBits+1
; 
; Store the new information for possible logging.
; Also test if the correction is less than 1ppb
;
	BANKSEL	LogPWM
	MOVF	LogPWM+1,W
	MOVWF	FSR0H
	MOVF	LogPWM,W
	MOVWF	FSR0L
	ADDFSR	0,hislength ; increase the pointer
	XORLW	low lasthist ; is it about to overrun the buffer?
	BTFSS	STATUS,Z
	BRA	CheckHistPtr
	MOVLW	low history ; buffer overrun, set to
	MOVWF	FSR0L ; buffer start
	MOVLW	high history
	MOVWF	FSR0H	
CheckHistPtr:
; save the pointer as current and for logging
	MOVF	FSR0L,W
	MOVWF	HisPtr ; log pointer
	MOVWF	LogPWM ; current pointer
	MOVF	FSR0H,W
	MOVWF	HisPtr+1
	MOVWF	LogPWM+1
; checking
	MOVF	HistPWMcnt,W
	INCF	HistPWMcnt,F
	XORLW	hiscount ; is there already 24
	BTFSS	STATUS,Z
	BRA	StorLog
; need to move start of history as the oldest is about to be
; overwritten
	MOVF	FSR0L,W
	ADDFSR	0,hislength ; increase the pointer
	XORLW	low lasthist ; is it about to overrun the buffer?
	BTFSS	STATUS,Z
	BRA	$+5
	MOVLW	low history ; buffer overrun, set to
	MOVWF	FSR0L ; buffer start
	MOVLW	high history
	MOVWF	FSR0H	
	MOVF	FSR0L,W ; save the pointer for logging
	MOVWF	HistPWMptr
	MOVF	FSR0H,W
	MOVWF	HistPWMptr+1
	MOVLW	hiscount
	MOVWF	HistPWMcnt
StorLog:	
	MOVF	HisPtr,W
	MOVWF	FSR0L
	MOVF	HisPtr+1,W
	MOVWF	FSR0H
; move the current time to history
	BANKSEL	RXtime
	MOVF	RXtime,W
	MOVWI	0[FSR0]
	MOVF	RXtime+1,W
	MOVWI	1[FSR0]
	MOVF	RXtime+2,W
	MOVWI	2[FSR0]
; move the new PWM (saved in PackedD)
	BANKSEL	PackedD
	MOVF	PackedD,W
	MOVWI	3[FSR0]
	MOVF	PackedD+1,W
	MOVWI	4[FSR0]
	MOVF	PackedD+2,W
	MOVWI	5[FSR0]
; convert the adjustment to parts per billion (one decimal place)
	CALL	Push3
	data	RTscratch
	CALL	PushLit
	data	D'250'
	CALL	multiply
	CALL	Push4
	data	SUslope
	CALL	divide
	CALL    Pop4
	data    REGA
	CALL	Push4
	data	REGA
	CALL	Pop4
	data	AdjWork ; save for later
	CALL    BIN2BCD
	MOVF	PackedD+4,W
	ANDLW	0xF0 ; remove decimal part
	IORWF	PackedD+3,W
	IORWF	PackedD+2,W
; if not all zero, go back to not locked under 1ppb
	BTFSS	STATUS,Z ; 
; Bigger than 10^-9 so clear the Locked flag
	BCF	Locked
; answer is integer ppb. if small, increase multiplication
	MOVF	PackedD+3,W
	IORWF	PackedD+2,W
	BTFSC	STATUS,Z ; 
	BRA	Under10
	CALL	Push4
	data	AdjWork ; restore saved
; multiply by 100 - total is 25000
	CALL	PushLit
	data	D'100'
	CALL	multiply
	BRA	SaveAdj
Under10:
	CALL	Push4
	data	RTscratch
; multiply by 25000 - too big for PushLit
	MOVLW	low D'25000'
	MOVWF	REGA
	MOVLW	high D'25000'
	MOVWF	REGA+1
	CALL	Push2U
	data	REGA
	CALL	multiply
	CALL	Push4
	data	SUslope
	CALL	divide
SaveAdj:
	CALL    Pop4
	data    REGA
	MOVF	HisPtr,W
	MOVWF	FSR0L
	MOVF	HisPtr+1,W
	MOVWF	FSR0H
	MOVF	REGA,W
	MOVWI	6[FSR0]
	MOVF	REGA+1,W
	MOVWI	7[FSR0]
	MOVF	REGA+2,W
	MOVWI	8[FSR0]
	MOVF	REGA+3,W
	MOVWI	9[FSR0]
; saved the data - are we logging it
	BTFSC	LogAdj
	CALL	ShowLast
; SkipLog:
    	GoMain	NewPWM
;##################################################
;              C A L I B R A T I O N              ;	
;##################################################
CalibReqd:
    	GoCallSubs
	CALL	WaitAsecond
	CALL    PrintP ; calibration message
	data    NoCalData ; "No calibration data - starting calibration"
	CLRF	SUavSlope
	CLRF	SUavSlope+1
	CLRF	SUavSlope+2
	CLRF	SUavSlope+3
	CLRF	SUavZero
	CLRF	SUavZero+1
	CLRF	SUavZero+2
	CLRF	SUavZero+3
	MOVLW	D'16' ; loop 16 times through +-0.25Hz test
	MOVWF	SUavLoop
LLoop:
; test the frequency drift at the limits of the control voltage.
; The oscillator should run fast at one limit and slow at the
; other. The program loops here until the condition is satisfied.
	BANKSEL SUtime1
	CLRF	SUslowV+2 ; these should both get filled in
	CLRF	SUfastV+2 ; if the oscillator can be controlled
; try a low voltage (see definitions)
	BANKSEL PWM
	MOVLW	LowVlimit ; low voltage
	MOVWF   PWM+2
	BANKSEL SUtime1
	CALL	FreqErr
; try a high voltage (see definitions)
	BANKSEL PWM
	MOVLW	HighVlimit ; high voltage
	MOVWF   PWM+2
	BANKSEL SUtime1
	CALL	FreqErr
; test if there is both a fast and slow
	MOVF	SUslowV+2,F
	BTFSS	STATUS,Z
	MOVF	SUfastV+2,F
	BTFSC	STATUS,Z
	BRA	LLoop
; got our slow and fast - need a slope
TVrefine:
;
; compute a new <control volts>/<frequency change> slope
; slope = (SUfastV - SUslowV)/(SUfastE - SUslowE) expressed
; as change of 24-bit PWM value to get 25nS drift per second
;
; The E value has a one byte fraction. The V value should be adjusted
; to also have a 1 byte fraction. Initially (using the start values)
; the values are too big for 32 bit signed arithmetic. To address this,
; the V difference is not adjusted and the E fraction is removed.
; Once the V difference is small, it is adjusted to have a 1 byte fraction.
;
    	GoCallSubs
; get the error difference
	CALL    Push4
	data	SUfastE
	CALL    Push4
	data	SUslowE
	CALL    subtract    ; Efast - Eslow
	CALL	Pop4	; save this
	data	SUediff
; get the V difference
	CALL    Push3U
	data	SUfastV
	CALL    Push3U
	data	SUslowV
	CALL    subtract    ; Vfast - Vslow
	CALL	Pop4
	data	SUvdiff+1
; test if this is more than a signed 3 byte value
	RLF	SUvdiff+3,W ; sign bit for a 3 byte value into C
	CLRW
	ADDWFC	SUvdiff+4,W ; zero if C and W are all the same bits
	BTFSS	STATUS,Z ; skip if a 3 byte value
	BRA	TVbigV
; V diff is small - give it a 1 byte fraction
	CLRF	SUvdiff
	CALL	Push4 ; push adjusted V
	data	SUvdiff
	CALL	Push4
	data	SUediff
	BRA	TVslope
TVbigV:
	CALL	Push4 ; push unadjusted V
	data	SUvdiff+1
	CALL	Push3 ; remove fraction from E diff
	data	SUediff+1
TVslope:
; Vdiff and Ediff now have same number of fractional bits
; calculate slope - change of V to get 1 tick (0.25Hz) change
; in frequency.
	CALL    divide     ; get final result (slope)
	CALL    Pop4
	data	SUslope 
; now calculate a voltage for a nominal zero error
; Vfast - Efast * slope (Efast is +ve)
	CALL    Push3U
	data	SUfastV
; if error is large, multiply by slope exceeds arithmetic
; so truncate error before multiply
; a smaller error, truncate after multiply
; try truncated value first
	CALL	Push3 ; truncated error
	data	SUfastE+1
	CALL    Push4 
	data	SUslope
	CALL    multiply ; Cslow * slope
	CALL	Pop4
	data	SUvdiff
; if the sign of SUvdiff+2 is different to SUvdiff+3 then the
; truncated value is used
	ASLF	SUvdiff+2,W
	MOVLW	0
	ADDWFC	SUvdiff+3,W
	BTFSC	STATUS,Z
	BRA	TVsmallE
	CALL	Push4
	data	SUvdiff
	BRA	TVcalcZ
TVsmallE:
	CALL	Push4 ; error with fraction
	data	SUfastE
	CALL    Push4 
	data	SUslope
	CALL    multiply ; Cslow * slope
	CALL	Pop4
	data	SUvdiff
	CALL	Push3 ; truncate the result
	data	SUvdiff+1
TVcalcZ:
	CALL    subtract	; result - Vfast - Efast * slope
	CALL    Pop3	; a guess at a voltage with zero drift
	data	SUzeroV
; test to see if the difference in errors is less than 0.6Hz,in which case
; the two values for estimation will be +0.25Hz and -0.25Hz
	CALL	Push4
	data	SUfastE
	CALL	Push4
	data	SUslowE
	CALL	subtract
	CALL	Pop4
	data	SUediff
;
	MOVF    SUediff+3,W
	IORWF	SUediff+2,W
	BTFSS   STATUS,Z    ; should be zero
	BRA	TVnewV ; not zero, need to reduce the difference
	MOVLW   0xFE ; test if value is 2 (0.5Hz difference)
	ADDWF	SUediff+1,W 
	BTFSS	STATUS,C
	BRA	TVconverged ; value less than 0.5Hz
	BTFSS	STATUS,Z ; was it 2? (0.5Hz to < 0.75Hz)
	BRA	TVnewV ; value 0.75Hz or more
	MOVLW	152 ; about 0.15Hz (test 0.5Hz to 0.6Hz)
	ADDWF   SUediff,W
	BTFSS	STATUS,C ; skip if > 0.6Hz
	BRA	TVconverged
	
; new value of V with the aim to get a drift of
; +-25nS per second (+-0.25Hz)
;
; need to narrow in on the right voltage - determine size of error
TVnewV:	
	CALL	Push4
	data	SUfastE
	CALL	Push4
	data	SUslowE
	CALL	addm
; getting the larger deviation
; if the result is +ve, the fast deviation is larger
	CALL	Pop4
	data	SUediff
	BTFSS	SUediff+3,7
	BRA	PickFast
; slow deviation larger
	CALL	Push3
	data	SUslowE+1
	BRA	GotPick
PickFast:
; calculate a voltage closer to the desired voltage
; check the error is 1 or more
	CALL	Push3
	data	SUfastE+1 ; ignore the fraction part	
GotPick:
; halve the error and calculate a new voltage
	CALL	Pop3
	data	SUediff
	MOVF	SUediff,W   ; fix rounding error problem
	XORLW	0xFD	    ;
	MOVLW	0xFE	    ;
	BTFSC	STATUS,Z    ;
	MOVWF	SUediff	    ;
;	CALL	Print1H
;	data	SUediff+2 ## DEBUG - multiple of 0.25Hz
;	CALL	Print2H
;	data	SUediff
	ASRF	SUediff+2,F
	RRF	SUediff+1,F
	RRF	SUediff,F
	
	
TVcalcV:
; log the result
;	CALL	Print1H
;	data	SUediff+2 ## DEBUG - multiple of 0.25Hz
;	CALL	Print2H
;	data	SUediff
	CALL	Push3
	data	SUediff
	CALL	Push4
	data	SUslope
	CALL	multiply
	CALL	Push3U
	data	SUzeroV
	CALL	addm ; low value
	CALL	Pop3
	data	PWM
; do another test
	CALL	FreqErr
	BRA	TVrefine
TVconverged:
; now test Cfast - Cslow (held in SUtest) in the range
; 0x0001FB to 0x0000205 (i.e. within about 1%)
	MOVLW   0x05
	ADDWF   SUediff,W     ; move to range 0x200 to 20A
	BTFSC   STATUS,C
	INCF    SUediff+1,F
	SUBLW   0x0A ; test 0 to 0x0A
	BTFSS   STATUS,C
	BRA	MoreVals
	MOVF    SUediff+1,W
	SUBLW   0x02 ; must be = 2
	BTFSC   STATUS,Z
	BRA	AllGood
MoreVals:
; flip between fast and slow
	MOVF	SUslowfast,F
	BTFSS	STATUS,Z ; was slow last?
	BRA	TVnextS
	MOVLW	0x01
	CLRF	SUediff+2
	CLRF	SUediff+1
	MOVWF	SUediff
	BRA	TVcalcV
TVnextS:	
; fast was last, make it slow (-1)
	MOVLW	0xFF
	MOVWF	SUediff+2
	MOVWF	SUediff+1
	MOVWF	SUediff
	BRA	TVcalcV
; Success
AllGood:
	BTFSS	LogAdj
	BRA	AG1
	CALL	PrintP
	data	Success
AG1:
; accumulate slope
	CALL	Push4
	data	SUavSlope
	CALL	Push4
	data	SUslope
	CALL	addm
	CALL	Pop4
	data	SUavSlope
; accumulate zero volts
	CALL	Push4
	data	SUavZero
	CALL	Push3
	data	SUzeroV
	CALL	addm
	CALL	Pop4
	data	SUavZero
;
	DECFSZ	SUavLoop,F
; and repeat
	BRA	MoreVals
; accumulated 16 good values - average them and store
	CALL	Push4
	data	SUavSlope
	CALL	PushLit
	data	D'16'
	CALL	divide
	CALL	Pop4
	data	SUslope
;	
	CALL	Push4
	data	SUavZero
	CALL	PushLit
	data	D'16'
	CALL	divide
	CALL	Pop3
	data	SUzeroV
; set up the header
	MOVLW	4 ; default log level after setup
	MOVWF	SUlogLev ; is pass through NMEA data
	MOVLW	9 ; default test duration (512 seconds)
	MOVWF	SUrunLev
ChangeCV:
	MOVLW	A'B'
	MOVWF	CDheader
ClobberEnt:
	GoCallSubs
	MOVLW	A'G'
	MOVWF	CDheader+1
	MOVLW	A'V'
	MOVWF	CDheader+2
	MOVLW	A'1'
	MOVWF	CDheader+3
;
; WRITE PROGRAM MEMORY - Substantially copied from doco
; Flash row erase
; 1F80h-1FFFh durable memory - erase 1FC0-1FDF
;----- mainly copied from the manual ------
; This row erase routine assumes the following:
; 1. A valid address within the erase row is loaded in ADDRH:ADDRL
	BCF	INTCON,GIE ; Disable ints
	BANKSEL PMADRL
	MOVLW	low ConfigLoc ; Load lower 8 bits of erase address boundary
	MOVWF	PMADRL
	MOVLW	high ConfigLoc ; Load upper 6 bits of erase address boundary
	MOVWF	PMADRH
	BCF	PMCON1,CFGS ; Not configuration space
	BSF	PMCON1,FREE ; Specify an erase operation
	BSF	PMCON1,WREN ; Enable writes
	MOVLW	55h ; Start of required sequence to initiate erase
	MOVWF	PMCON2 ; Write 55h
	MOVLW	0AAh ;
	MOVWF	PMCON2 ; Write AAh
	BSF	PMCON1,WR ; Set WR bit to begin erase
	NOP ; NOP instructions are forced as processor starts
	NOP ; row erase of program memory.
; The processor stalls until the erase process is complete
; after erase processor continues with 3rd instruction
	BCF	PMCON1,WREN ; Disable writes
;	BSF	INTCON,GIE ; Enable interrupts
;----- end copied from the manual ------
; set up and write a row
;----- mainly copied from the manual ------
;	BCF	INTCON,GIE ; Disable ints
;	BANKSEL PMADRL ; already done
; the program writes 15 low bytes and a checksum to 16 locations. Only
; the low 8 bits written so they can be retrieved by indirect reads.
	MOVLW	D'15'
	MOVWF	CDwrloop
	CLRF	CDwrCkSum
	MOVLW	low ConfigLoc ; Load lower 8 bits of erase address boundary
	MOVWF	PMADRL
	MOVLW	high ConfigLoc ; Load upper 6 bits of erase address boundary
	MOVWF	PMADRH
;
	MOVLW	low CDheader
	MOVWF	FSR0L
	MOVLW	high CDheader
	MOVWF	FSR0H
	BCF	PMCON1,CFGS ; Not configuration space
	BSF	PMCON1,WREN ; Enable writes
	BSF	PMCON1,LWLO ; Only Load Write Latches
FMLOOP:
	MOVIW	FSR0++ ; Load data byte into lower
	XORWF	CDwrCkSum,F
	MOVWF	PMDATL ;
	CLRF	PMDATH ;
	MOVF	PMADRL,W ; Check if lower bits of address are '00000'
	MOVLW	55h ; Start of required write sequence:
	MOVWF	PMCON2 ; Write 55h
	MOVLW	0AAh ;
	MOVWF	PMCON2 ; Write AAh
	BSF	PMCON1,WR ; Set WR bit to begin write
	NOP	; NOP instructions are forced as processor
	NOP	; loads program memory write latches
	INCF	PMADRL,F ; Still loading latches Increment address
	DECFSZ	CDwrloop,F
	BRA	FMLOOP ; Write next latches
; START_WRITE:
	MOVF	CDwrCkSum,W
	MOVWF	PMDATL ;
	CLRF	PMDATH ;
	BCF	PMCON1,LWLO ; No more loading latches
	; - Actually start Flash program memory write
	MOVLW	55h ; Start of required write sequence:
	MOVWF	PMCON2 ; Write 55h
	MOVLW	0AAh ;
	MOVWF	PMCON2 ; Write AAh
	BSF	PMCON1,WR ; Set WR bit to begin write
; NOP instructions are forced as processor writes all the program memory
; write latches simultaneously to program memory.
	NOP
	NOP
; After NOPs, the processor stalls until the self-write process is complete
; after write processor continues with 3rd instruction
	BCF PMCON1,WREN ; Disable writes
	BSF INTCON,GIE ; Enable interrupts
;----- end copied from the manual ------
	BANKSEL	CDheader
	MOVF	CDheader,W
	XORLW	A'X'
	BTFSC	STATUS,Z
	RESET
	CALL	WaitAsecond
;
;	CALL    PrintP
;	data	CalFinish
	CALL	WaitAsecond
	RESET

;#################################################################
;                      S U B R O U T I N E S
;#################################################################

Section10: ORG 0x1000

cvPWM:
;	
; Calculations are done in 25ns increments. cvPWM converts
; increments to adjustments of the control voltage (i.e. the
; Pulse Width Modulator or PWM). The conversion involves the
; known (control voltage change:Hz change) relationship, and
; the period over which the 25ns increments ocurred.
;
; to get PWM adjustment, multiply by newSlope to convert to
; a PWM change and adjust for measurement period.
	MOVWF	RTloop
	CALL	Push2
	data	newSlope
	CALL	multiply
	CALL	Pop4
	data	RTscratch
; the change has to be shifted for
; 1. The newSlope may have been scaled, newSlpAdj has left shifts
; 2. The result right shifted for the measurement period
; 3. Other shifts for arithmetic adjustment
; Enter with W = right shifts (2. and 3.)
	MOVF	newSlpAdj,W ; left shifts
	ADDWF	RTloop,F
	BTFSC	STATUS,Z
	BRA	cvpush	; no shifts required
	BTFSC	RTloop,7
	BRA	newadjl ; left shifts required
newadjr:
; too big, adjust right
	ASRF	RTscratch+3,F
	RRF	RTscratch+2,F
	RRF	RTscratch+1,F
	RRF	RTscratch,F
	DECFSZ	RTloop,F
	BRA	newadjr
	BRA	cvpush
newadjl:
; too small, adjust left
	ASLF	RTscratch,F
	RLF	RTscratch+1,F
	RLF	RTscratch+2,F
	RLF	RTscratch+3,F
	INCFSZ	RTloop,F
	BRA	newadjl
cvpush:
	CALL	Push4
	data	RTscratch
	RETURN
;-----------------------------------------------------------------
ResetDly:
; -- Synchronises LoopC to 1ppS --
; Also a delay to allow the control voltage to settle if it was
; changed. Forces the comparator to synchronise the LoopC values
; so the the TMR2 IRC starts TMR1 about 1mS before a 1ppS.
; A 1ppS should then arrive about half way between TMR1 starting
; and TMR1 overflowing if a 1ppS is not detected.
; -- 2 seconds to settle --
	CALL	WaitAsecond
	CALL	WaitAsecond
; If the flag ResetLoop is clear, the TMR2 IRC will start TMR1
; at the end of its second (which may not be synchronised with
; 1ppS). Set ResetLoop and clear any running state of TMR1 so
; TMR1 stays stopped
	BSF	ResetLoop ; TMR2 doesn't run timer 1
	MOVF	BSR,W ; save the caller's bank
	BANKSEL	T1CON
	BSF	T1CON,TMR1ON ; stop the timer if it is running
	BCF     PIR1,TMR1IF ; clear the interrupt flag
	BCF	PIR2,C1IF ; precaution, should not be set
; Allow comparator interrupt on high to low. The next 1ppS causes
; the comparator IRC to run, it resets LoopC so a TMR2 second is
; just before the next 1ppS. The comparator IRC also disables
; itself and clears ResetLoop.
	BANKSEL	CM1CON1
	BSF	CM1CON1,C1INTN
	MOVWF	BSR ; restore the caller's bank
; wait until comparator resets TMR2 loop
	BTFSC	ResetLoop
	BRA	$-1
; The first TMR2 second after a reset will deliver an indeterminate
; value for TMR1. Ignore it and wait another second.
	CALL	WaitAsecond
	BTFSS	ValidFix ; don't proceed unless data was valid
	BRA	ResetDly
; the second TMR2 second should capture a valid TMR1 
;	GOTO	WaitAsecond - drop through to 
;------------------------------------------------------------------------
WaitAsecond:
;
; When mainline processing is complete and waiting for the next second,
; this subroutine is called. At this point, there is time to acknowledge
; and act on user input. The subroutine then loops until the TMR2
; interrupt handler signals end of a second.
;
    	MOVF	BSR,W
	MOVWF	UinBank
; test for user input processing
	BANKSEL	Utimeout
	MOVF	UsrFlag,F
	BTFSC	STATUS,Z
	GOTO	WaitExit ; fall through
;
; User can interact via serial terminal. Input is initiated by
; pressing TAB.
; User Commands:
; 0-4 set logging level
; 5-9 set maximum measurement period
; C - initiate calibration
; H - show up to the last 24 PWM adjustments
; R - reset
; T - timestamp (includes date)
;
; should we prompt?
	BTFSS	UsrTab ; has TAB been received
	GOTO	URtimeOut ; not yet
	BTFSC	Prompted ; has the '>' prompt been sent
	GOTO	URskipPrompt ; yes, skip sending it again
URprompt:
	CALL	PrintP
	data	Prompt
	BSF	Prompted
URskipPrompt:
	BTFSS	UsrCmd ; any command yet?
	GOTO	URtimeOut ; no
	MOVF	UsrChar,W ; display it
	CALL	PrintC
	MOVF	UsrChar,W ; letter or numeric
	ADDLW	0xC0 ; will overflow on a letter
	BTFSC	STATUS,C
	GOTO	UsrLetter
	ADDLW	0x06 ; test number 0 to 9
	BTFSC	STATUS,C
	GOTO	UsrNoCmd ; not in range
	ADDLW	0x05 ; overflow on a number 5 to 9
	BTFSC	STATUS,C
	BRA	UsrLevelSet
	ADDLW	0x05 ; overflow on a number 0 to 4
	BTFSS	STATUS,C
	GOTO	UsrNoCmd
	BTFSS	STATUS,Z ; leave 0 as 0
	ADDLW	1 ; 10 - only changes, 11 - both, 100 - NMEA
	XORLW	0x05 ; convert 101 to 111 - full log
	BTFSC	STATUS,Z
	IORLW	0x02
	XORLW	0x05
	BANKSEL	LLsave
	MOVWF	LLsave ; set the log level
	BANKSEL	SUlogLev
	MOVWF	SUlogLev ; save as default
	GOTO	UsrResume
UsrLevelSet:
	ADDLW	8
	BANKSEL	SUrunLev
	MOVWF	SUrunLev
	GOTO	UsrResume
UsrLetter:
	ANDLW	0xDF ; lower to upper
	XORLW	A'T' & 0x1F 
	BTFSC	STATUS,Z
	GOTO	URdaytime ; T - timestamp
	XORLW	(A'T' ^ A'H') & 0x1F
	BTFSC	STATUS,Z
	GOTO	UsrHistory ; H - history
	XORLW	(A'H' ^ A'R') & 0x1F
	BTFSC	STATUS,Z
	RESET		; R - reset
	XORLW	(A'R' ^ A'C') & 0x1F
	BTFSC	STATUS,Z
	GOTO	ClobberCalib ; C - calibrate
	XORLW	(A'C' ^ A'S') & 0x1F
	BTFSC	STATUS,Z
	GOTO	ShowStats ; S - show stats
	XORLW	(A'S' ^ A'Z') & 0x1F
	BTFSC	STATUS,Z
	GOTO	ClearStats ; Z - clear stats
	XORLW	(A'Z' ^ A'V') & 0x1F
	BTFSC	STATUS,Z
	GOTO	NewDefCV ; V - new default control voltage
	XORLW	(A'V' ^ A'W') & 0x1F
	BTFSC	STATUS,Z
	GOTO	WriteFlash ; W - write settings to flash
	XORLW	(A'W' ^ A'I') & 0x1F
	BTFSC	STATUS,Z
	GOTO	ShowInfo ; I - show startup message
	XORLW	(A'I' ^ A'U') & 0x1F
	BTFSS	STATUS,Z
	GOTO	UsrNoCmd
; user requests update
	CALL	PrintP
	data	ProgReq
	BANKSEL	TXSTA
UsrProgWait:
	BTFSS	TXSTA,TRMT   ; wait transmit buffer empty
	GOTO	UsrProgWait
	BCF	INTCON,GIE ; Disable interrupts
	LGOTO	bootstrap
UsrNoCmd:
	CALL	PrintP
	data	BadAT
	BCF	UsrCmd
	GOTO	URprompt
UsrHistory:
;	BANKSEL	HistPWMcnt
	MOVF	HistPWMcnt,F ; is there history
	BTFSC	STATUS,Z
	GOTO	NoUsrHist
	MOVF	HistPWMptr,W
	MOVWF	HistNxtPtr
	MOVF	HistPWMptr+1,W
	MOVWF	HistNxtPtr+1
	MOVLW	0x08 ; clear input, showing history
	MOVWF	UsrFlag
	GOTO	URtimeOut
NoUsrHist:
	CALL	PrintP
	data	NoHistory
	GOTO	UsrResume
URdaytime:
	CALL	DateStamp
	GOTO	UsrResume
ShowInfo:
	CALL	PrintP
	data	Announce
	CALL	PrintP
	data	Mperiod
	BANKSEL	REGA
	CLRF	REGA
	CLRF	REGA+1
	CLRF	REGA+2
	CLRF	REGA+3
	INCF	REGA,F
	MOVLW	Low ConfigLoc ; Low address of calibration data
	MOVWF	FSR0L
	MOVLW	High ConfigLoc | 0x80 ; High address (program memory)
	MOVWF	FSR0H
	MOVIW	12[FSR0]
	MOVWF	MCOUNT
	BCF	STATUS,C
SIloop:
	CALL	sla
	DECFSZ	MCOUNT,F
	BRA	SIloop
	CALL	PrinNum
	CALL	PrintP
	data	Seconds
	GOTO	UsrResume
ClobberCalib:
	BANKSEL	CDheader
	MOVLW	A'X'
	MOVWF	CDheader
	GoMain	ClobberEnt
NewDefCV:
	CALL	Push3U
	data	PWM
	CALL	Pop3
	data	SUzeroV
	CALL	PrintP
	data	VoltSave
	GOTO	UsrResume
WriteFlash:
    	GoMain	ChangeCV

ShowStats:
; log a message showing the statistics e.g.
; Up 2483 secs. Lost Fix:0 No 1ppS:0 Rejected:0
	CALL	PrintP ; Up time
	data	Uptime ; "\r\nUp \x00"
	CALL	Push4
	data	Celapsed
	CALL	PrinStat
	CALL	PrintP ; No Fix
	data	NoFix ; " secs. Lost Fix:\x00"
	CALL	Push4
	data	CnoFix
	CALL	PrinStat
	CALL	PrintP ; missing 1pps
	data	NoPulse ; " No 1ppS:\x00"
	CALL	Push4
	data	CmissPulse
	CALL	PrinStat
	CALL	PrintP ; 1pps rejected (too early or late)
	data	RejPulse ; " Rejected:\x00"
	CALL	Push4
	data	Crejected
	CALL	PrinStat
	GOTO	UsrResume
PrinStat:
; display 1 counter
	CALL	Pop4
	data	REGA
PrinNum:
	CALL	BIN2BCD
	MOVLW	0x08 ; print 8 digits
	GOTO	PrintDec

ClearStats:
	CALL	ClearCounters
	GOTO	UsrResume

URtimeOut:
; Come here if waiting for user input OR logging history
; history is logged on line a second, sending it all at
; once would overload the output buffer
	BTFSS	ShowHist ; are we showing history
	GOTO	CountDown ; no, go to timeout
	MOVF	HistNxtPtr,W
	MOVWF	HisPtr
	XORWF	LogPWM,W ; this the last one
	BTFSC	STATUS,Z
	BCF	ShowHist ; finished
	MOVF	HistNxtPtr+1,W
	MOVWF	HisPtr+1
; test buffer wraparound
	MOVF	HistNxtPtr,W
	XORLW	low lasthist
	BTFSS	STATUS,Z
	GOTO	IncHistPtr
; wrap
	MOVLW	low history
	MOVWF	HistNxtPtr
	MOVLW	high history
	MOVWF	HistNxtPtr+1
	GOTO	ShowHistEnt
IncHistPtr:
	MOVLW	hislength ; 10
	ADDWF	HistNxtPtr,F
	BTFSC	STATUS,C
	INCF	HistNxtPtr+1,F
ShowHistEnt:
	CALL	ShowHistory
	BANKSEL	Utimeout
	BTFSC	ShowHist
	GOTO	WaitExit
	GOTO	UsrRnoCRLF
CountDown:
; when waiting for user input, count down the timer.
; if it reaches zero, cancel the input
; Also dealing with history one line at a time
	DECFSZ	Utimeout,F
	GOTO	WaitExit
UsrResume:
    	CALL	CRLF
UsrRnoCRLF:
	BANKSEL	LLsave
	MOVF	LLsave,W ; reinstate previous logging
	MOVWF	LogLev
	CLRF	UsrFlag
WaitExit:
	BTFSS	PeriodEnd
	GOTO	$-1
	BCF	PeriodEnd
	MOVF	UinBank,W
	MOVWF	BSR
	RETURN
;--------------------------------------------------------------
; Keep counters of 1pps status. Cleared at start or by request 
ClearCounters:
	MOVLW	high Celapsed
	MOVWF	FSR0H
	MOVLW	low Celapsed
	MOVWF	FSR0L
	MOVLW	.16 ; clear 16 bytes
	MOVWF	GenBank ; temporary use as counter
	CLRW
	MOVWI	FSR0++
	DECFSZ	GenBank,F
	GOTO	$-2
	MOVLW	MissedLimit
	MOVWF	INDF0
	RETURN
	
AddCounter:
; Add counter called several times to update
; add W to a 4 byte counter pointed by FSR0, increment FSR0 by 4
	ADDWF	INDF0,W
	MOVWI	FSR0++
	CLRW
	ADDWFC	INDF0,F
	ADDFSR	0, 1
	ADDWFC	INDF0,F
	ADDFSR	0, 1
	ADDWFC	INDF0,F
	ADDFSR	0, 1
	RETURN
;------------------------------------------------------------------
; various logging routines
;
;
; RTlogAll logs each 1ppS deviation
;
RTlogAll:
	MOVF	RTcount,W
	ANDLW	0x0F ; multiple of 16?
	BTFSS	STATUS,Z
	BRA	RTlogNext
	CALL	Push3
	data	RTcount
	CALL	Pop4
	data	REGA
        CALL    BIN2BCD
	MOVLW	0x85
	CALL    PrintDec ; print the count since correction
RTlogNext:
; if a drift correction happened, show it
	MOVLW	' '
	BTFSS	DriftFlag
	BRA	RTlogOffset
	MOVLW	'-'
	BTFSS	DriftDir
	MOVLW	'+'
RTlogOffset:
; Log raw error
	CALL	PrintC
	MOVLW	'f'
	BTFSS	ValidFix ; if no valid fix, assume zero
	BRA	Missed
	MOVLW	'm'
	BTFSC	MissedPulse ; if no valid fix, assume zero
	BRA	Missed	
	CALL	Print1H
	data	RTscratch
	BRA	LogEOL
Missed:
	CALL	PrintC
	MOVLW	A'-'
	CALL	PrintC
LogEOL:
	MOVF	RTcount,W
	ANDLW	0x0F ; last in the line?
	XORLW	0x0F
	BTFSC	STATUS,Z
	CALL	CRLF
	RETURN
	
; -- control volts to BCD --
; Converts the 24 bit control volt value to microvolt
; It is assumed that PWM 0x000000 = 0 Volt PWM 0xFA0000 = 5 Volt
; 0xFA0000 = 16,384,000.
; The fraction 112/367 is within 8uV at full scale, the integers
; used are limited by the 32 bit signed arithmetic routines.
; The value is not intended to be accurate, the source of the
; control voltage is unlikely to be perfect.
; Input is already pushed onto the stack
; Output is delivered to PackedD
CV2BCD:
    CALL    PushLit
    data    D'112'
    CALL    multiply
    CALL    PushLit
    data    D'367'
    CALL    divide
TI2BCD: ; entry used to convert stack to BCD
    CALL    Pop4
    data    REGA
    CALL    BIN2BCD
    RETURN

; -- frequency error to bcd --
; the error is in 25nS deviations (i.e. 0.25Hz) with a 1 byte
; fraction. Taken as a binary number (ignoring the fraction)
; this is equivalent to Hz/1024. This is adjusted to Hz/1000
; by subtracting 3/128ths
FE2BCD:
    CALL    Push4
    data    SUediff
    CALL    Push4
    data    SUediff
    CALL    PushLit
    data    D'3'
    CALL    multiply
    CALL    PushLit
    data    D'128'
    CALL    divide
    CALL    subtract
    CALL    Pop4
    data    REGA
    CALL    BIN2BCD
    RETURN
    
  
FreqErr:
; Calculates a frequency change for a set value of PWM. The test is run
; for up to 256 seconds, but will exit early if the difference is large.
;
; The frequency error is returned as processor ticks (25nS difference
; from 10MHz) per seconds at the set PWM. This can be a fraction so it is
; calculated as 3 (2 significant) bytes whole number + 1 byte fraction.
;
; The length of test is 2^n seconds. Initially, the test is run for 1
; second. If the ticks have changed by less than 256 (a frequency
; error of 64Hz) then the length of test is extended by doubling it.
; The test ends if the duration of test is 256 seconds or the ticks
; change by more than 256 (which over 256 seconds is an error of 0.25Hz)
;
; Results of the test are stored in one of two locations, depending on
; if the oscillator is running fast (>10MHz) or slow (<10MHz). If
; detail logging is enabled, the result is logged
	CALL	ResetDly
;    BANKSEL SUtime1
        CALL    Push2U ; fetch TMR1 at start
        data    LastTMR1
        CALL    Pop4 ; and save it in SUtime1
        data    SUtime1
        MOVLW   1
	MOVWF   SUwork
	MOVWF   SUduration
	CLRF    SUduration+1
	CLRF    SUledctl
SUloop:
; The LED flashes the current test number in binary every 4 seconds.
; The test number counts down from 16 to 1. SUledbits has the test
; number, SUledctl has a bit set for every significant bit in
; SUledbits. The two values are shifted 2 bits to the right each
; second. If SUledctl is zero, then the values are reloaded 
;	BANKSEL	SUwork ; ??
	MOVF    SUledctl,F ; is there a value?
	BTFSS	STATUS,Z
	BRA	SUflash
; SUledctl is zero, reload values
	MOVF    SUavLoop,W ; get the test number
	MOVWF   SUledbits ; look for significant bits
SUsigled:
	BSF	STATUS,C ; at least one significant digit
	RLF	SUledctl,F ; left
	LSRF    SUledbits,F ; right
	BTFSS	STATUS,Z ; no more significant bits?
	BRA	SUsigled ; yes, shift in another 1 bit to ctl
	MOVWF   SUledbits ; no, put back the test number
SUflash:
	MOVLW	0x40 ; one flash = 1
	LSLF	SUledbits,F
	BTFSS	STATUS,C
	MOVLW	0x90 ; two flash = 0
	LSLF	SUledctl,F
	BTFSC	STATUS,C ; skip if not a significant bit
	MOVWF   LedBits
	MOVF    SUavLoop,W
	MOVLW	0x10 ; one flash = 1
	LSLF	SUledbits,F
	BTFSS	STATUS,C
	MOVLW	0x24 ; two flash = 0
	LSLF	SUledctl,F
	BTFSC	STATUS,C ; skip if not a significant bit
	MOVWF   LedBits+1
; done setting up LED
	CALL	WaitAsecond
	DECFSZ  SUwork,F ; end of test period?
        BRA	SUloop
; calculate ticks difference
	CALL    Push2U ; the current TMR1
        data    LastTMR1
	CALL    Push4 ; the first TMR1
        data    SUtime1
        CALL    subtract ; difference <TMR1 now> - <TMR1 start>
        CALL    Pop4
	data    SUediff ; could be signed
; check for maximum time limit (256 ticks)
	MOVF    SUduration,W
	BTFSC   STATUS,Z ; is duration 256?
	BRA	SUnoAdj ; yes, leave
	MOVWF   SUwork ; no, set up to go again
; test for between 0xFFFFFF00 and 0x000000FF. Both SUdiff+1
; and SUdiff+2 should be all sign bits (0 if +ve, 1 if -ve)
	MOVF    SUediff+1,W
	XORWF	SUediff+2,W
	BTFSS   STATUS,Z
	BRA	SUadjust
; double the test length	
	MOVF    SUwork,W
	ADDWF   SUduration,F ; and double the duration
	BRA	SUloop
; The error returned should be ticks/second with 1 fraction
; byte. The difference value is adjusted.
SUadjust:
	LSLF    SUediff,F
	RLF	SUediff+1,F
	RLF	SUediff+2,F
	RLF	SUediff+3,F
	LSLF    SUwork,F
	BTFSC   STATUS,Z
	BRA	ValSave
	BRA	SUadjust
SUnoAdj:
; happens if duration is 256, the difference does not need adjusting
	INCF    SUduration+1,F
ValSave:
; save the data in either frequency higher or frequency lower store
	CALL    Push3U
	data    PWM
	CALL    Push4
	data    SUediff
	BTFSS   SUediff+3,7 ; sign
	BRA	SaveFast
	CALL    Pop4 ; if now - start -ve, going slow
	data    SUslowE
	CALL    Pop3
	data    SUslowV
	CLRF	SUslowfast
	BRA	CalMess
SaveFast: ; if now - start +ve, going fast
	CALL    Pop4
	data    SUfastE
	CALL    Pop3
	data    SUfastV
	INCF	SUslowfast,F
	
CalMess:
	BTFSS	LogAdj ; logging details?
	RETURN
; log the change
	CALL    PrintP ; 
	data    ControlV ; "Control \x00"
	CALL    Push3U
	data    PWM
	CALL    CV2BCD ; control volts to decimal
	MOVLW   0x67 ; 7 digits, 6 decimal places
	CALL    PrintDec ; print the control volts
	CALL    PrintP
	data    CVandErr ; "V Error=\x00"
	CALL    FE2BCD ; frequency error to decimal
	MOVLW   0x36 ; print 6 digits, 3 dec places
	CALL    PrintDec ; print the error    
	CALL    PrintP
	data    HzAndOver ; "Hz measured over "
	CALL    Push2 ; the test duration
	data    SUduration
	CALL    TI2BCD
	MOVLW   0x03 ; print 3 digits, no dec places
	CALL    PrintDec    
	CALL    PrintP
	data    Seconds ; " seconds.\x00"
	CALL	CRLF
	RETURN

;
;
DateStamp:
; print date and time retrieved from last $xxRMC
	CALL    PrintP
	data    TSdate
	MOVLW   high RXtime
	MOVWF   FSR0H
	MOVLW   low RXdate
	CALL    PrintDT
;
	CALL    PrintP
	data    TStime  ; " Time \x00"
	MOVLW   high RXtime
	MOVWF   FSR0H
        MOVLW   low	RXtime
	CALL    PrintDT
	CALL    PrintP
	data    TSutc   ; " UTC.\x00"
	GOTO    CRLF
;
ShowLast:
; log the last control voltage change, test if a CRLF is needed first
	BTFSS	LogDetail
	BRA	ShowHistory
	BANKSEL	RTcount
	MOVF	RTcount,W
	ANDLW	0x0F ; last in the line of 16?
	XORLW	0x0F
	BTFSS	STATUS,Z ; need a CRLF if not last
	CALL	CRLF
    
; ShowHistory is used to log control voltage changes from the history
; store. It may log as the data is put in the store (i.e. at the time
; the change ocurred) or if requested by the user.
ShowHistory:
; Retrieves information about a control voltage change and sends
; it to logging. The data is pointed to by HisPtr. The stored format
; is: HisPtr[0-2] UTC time stored as packed decimal DDMMYY
;     HisPtr[3-5] Control voltage as the 24-bit PWM value
;     HisPtr[6-9] Control voltage change (signed) as change in PWM value
; time
        CALL    PrintP
	data    TStime  ; " Time \x00"
	MOVF	HisPtr+1,W ; set up a pointer
	MOVWF	FSR0H
	MOVF	HisPtr,W
	CALL    PrintDT
	CALL    PrintP
	data    TSutc   ; " UTC.\x00"
; control voltage
	CALL	PrintP
	data	ControlV ; " Ctrl \x00"
	BANKSEL	PackedD
	MOVF	HisPtr,W
	MOVWF	FSR0L
	MOVF	HisPtr+1,W
	MOVWF	FSR0H ; move the voltage to the work area
	MOVIW	3[FSR0] ; PackedD use as temporary work area
	MOVWF	PackedD
	MOVIW	4[FSR0]
	MOVWF	PackedD+1
	MOVIW	5[FSR0]
	MOVWF	PackedD+2
	CLRF	PackedD+3
        CALL    Push4
	data    PackedD
	CALL    CV2BCD ; control volts to decimal
	MOVLW   0x67 ; 7 digits, 6 decimal places
	CALL    PrintDec ; print the control volts
; change in control voltage
	CALL    Pspace
	MOVF	HisPtr,W
	MOVWF	FSR0L
	MOVF	HisPtr+1,W
	MOVWF	FSR0H
	MOVIW	6[FSR0]
	MOVWF	REGA
	MOVIW	7[FSR0]
	MOVWF	REGA+1
	MOVIW	8[FSR0]
	MOVWF	REGA+2
	MOVIW	9[FSR0]
	MOVWF	REGA+3
        CALL    BIN2BCD
	MOVLW	0x38
	CALL    PrintDec ; print the frequency correctiom
	CALL	PrintP
	data	PPbillion
	CALL	CRLF
	RETURN
;-------------------------------------------------------------
PMgetPointr:
; generic routine where a CALL is followed by data (usually a pointer).
; The data is moved to FSR1 and the return address on the stack is
; incremented to skip the data.
; Used by Maths routines, Print routines
; switch to the stack pointer bank
	BANKSEL	TOSL
; set FSR1 to the address of the registers used to access program memory.
	MOVLW	low PMADRL
	MOVWF   FSR1L
	MOVLW   high PMADRL
	MOVWF   FSR1H
	BCF	INTCON,GIE ; no interrupts while modifying STKPTR
	DECF	STKPTR,F ; the caller of the caller
	DECF	STKPTR,F ; the caller of the caller
; move the top of stack pointer (the address of the caller)
; to the registers used to access program memory
	MOVF	TOSL,W
	MOVWF	INDF1
	MOVF	TOSH,W
	MOVWI	1[FSR1]
; increment the return address to skip the pointer
	INCFSZ	TOSL,F
	BRA	$+2
	INCF	TOSH,F
	INCF	STKPTR,F ; STKPTR returned to previous value
	INCF	STKPTR,F ; STKPTR returned to previous value
	BSF	INTCON,GIE ; and allow interrupts
; retrieve the data pointer from program memory
; this from the reference
	BANKSEL PMADR
	BCF	PMCON1,CFGS ; Do not select Configuration Space
	BSF	PMCON1,RD ; Initiate read
	NOP ; Ignored (Figure 11-2)
	NOP ; Ignored (in the reference)
; the word after the call now in PMDATL, PMDATH
; move it to FSR1 (it's usually a pointer)
	MOVF	PMDATL,W
	MOVWF	FSR1L
	MOVF	PMDATH,W
	MOVWF	FSR1H
	RETURN
;
;##################### start print routines ##################
; Data to be 'printed' is put in the TX buffer to be sent out
; the serial port by the UART.
; PrintP - prints a string stored in program memory
;	string is stored as 2 7-bit chars per 14 bit address
; PrintS - prints a string from data memory
; Print1H, Print2H - print 1 or 2 bytes as 2 or 4 hex characters
; PrintC - prints the character in W
; Pspace - uses PrintC to print a space
; CRLF - print a carriage return, line feed (end of line)
; PrintDec - Prints numbers with some formatting
;-------------------------------------------------------------

PrinSetup:
; used when the CALL to a print routine is followed by a pointer.
; This routine retrieves the pointer (which is in program memory)
	MOVF	BSR,W
	MOVWF	GenBank
; set FSR1 to the adress of the source string
	CALL	PMgetPointr ; fetch source pointer into FSR1
	RETURN

PrinCleanup:
	MOVF	GenBank,W
	MOVWF	BSR
	RETURN
    
PrintP:
; A CALL to PrintP is followed by a pointer to a message stored in
; program memory. e.g.
;    CALL    PrintP ; print literal from program memory
;    data    World
; be nice and save caller's bank
	CALL	PrinSetup
; PMDATH, PMDATL is a pointer to data in the program area.
; move it to the address registers
	MOVF    PMDATL,W
	MOVWF   PMADRL
	MOVF    PMDATH,W
	MOVWF   PMADRH
; now copy the data from program memory to the transmit buffer
PGloop:
	BCF	PMCON1,CFGS ; Do not select Configuration Space
	BSF	PMCON1,RD ; Initiate read
	NOP ; Ignored (Figure 11-2)
	NOP ; Ignored (in the reference)
	; convert back to two ASCII characters
	RLF	PMDATL,F ; Get MSb into C
	RLF	PMDATH,W ; then into the first character
	LSRF	PMDATL,F ; fixup second character
TXpMtest:
	IORLW	0x00 ; test for zero (end message)
	BTFSC   STATUS,Z
	GOTO	PrinCleanup
	CALL	PrintC
	BTFSC	PMDATL,7 ; retrieve PMDATL once
	BRA	TXpMore
	MOVF	PMDATL,W
	BSF	PMDATL,7 ; so it isn't read twice
	BRA	TXpMtest
TXpMore:
	INCFSZ	PMADRL,F
	BRA	PGloop
	INCF	PMADRH,F
	BRA	PGloop
;
PrintS:
; A CALL to PrintS is followed by a pointer to a message stored in
; data memory. e.g.
;    CALL    PrintS ; print literal from data memory
;    data    Date
; Save caller's bank, set up FSR0, FSR1
	CALL	PrinSetup 
; now copy the data to the transmit buffer
PSloop:
	MOVIW	FSR1++
	IORLW	0x00 ; test for zero (end message)
	BTFSC   STATUS,Z
	GOTO	PrinCleanup
	CALL	PrintC
	BRA	PSloop
;
Print2H:
; A CALL to Print2H is followed by a pointer to a 2 byte binary
; e.g.
;    CALL    Print2H ; print 2 byte variable as hex
;    data    LastTimer
	CALL	PrinSetup ; returns to PrintS2
	ADDFSR	FSR1,1
	SWAPF	INDF1,W
	CALL	TXcvhex
	MOVF	INDF1,W
	CALL	TXcvhex
	ADDFSR	FSR1,-1
	BRA	P1Hentry
Print1H:
; A CALL to Print1H is followed by a pointer to a byte
; e.g.
;    CALL    Print1H ; print 1 byte variable as hex
;    data    LastTimer
	CALL	PrinSetup
P1Hentry:
 	SWAPF	INDF1,W
	CALL	TXcvhex
	MOVF	INDF1,W
	CALL	TXcvhex
	GOTO	PrinCleanup
;
;
Pspace:
; print a space
	MOVLW	A' '
        GOTO    PrintC

CRLF:
; 'print' an end of line
	MOVLW   0x0D
	CALL    PrintC
	MOVLW   0x0A
	GOTO    PrintC
	
PrintDT:
; Print date or time (stored as 6 digit packed decimal)
; FSR0H should point to the right page, W to point
; to the value in the page to be 'printed'.
	MOVWF	FSR0L
	MOVF	BSR,W
	BANKSEL	STACK
	MOVWF	PMsave
	CLRF    PackedD
	CLRF    PackedD+1
	MOVIW   0[FSR0]
	MOVWF   PackedD+2
	MOVIW   1[FSR0]
	MOVWF   PackedD+3
	MOVIW   2[FSR0]
	MOVWF   PackedD+4
	MOVLW   0x86 ; print 6 digits no decimal places
	BRA	DTnext
; fall through to print 6 digits

PrintDec:
; print a decimal number from the BCD in PackedD. Takes a format
; from W
;    Bits 7 6 5 4 3 2 1 0
;         | --|-- -------- digits to print
;         |   +----------- decimal places
;         +--------------- 0 = zero suppress 1 = no suppression
	MOVWF	FSR1L ; temporary store
	MOVF	BSR,W
	BANKSEL	STACK
	MOVWF	PMsave
	MOVF	FSR1L,W
DTnext: ; entry for date/time	
	MOVWF	MTEMP
	ANDLW	0x80 ; isolate zero suppress
	MOVWF	REGC ; use for zero suppress 
	MOVLW	D'10' ; digits in the store
	MOVWF	MCOUNT
	MOVLW	high PackedD
	MOVWF	FSR1H
	MOVLW	low PackedD
	MOVWF	FSR1L
; test and clear the sign bit
	MOVLW	A'-'
	LSLF	PackedD,F ; sign bit in C
	BTFSC	STATUS,C ; leading minus?
	CALL	PrintC ; yes
	LSRF	PackedD,F ; sign gone
PDloop:
	INCF	MTEMP,W ; add 1 to number of digits to print
	ANDLW	0x0F ; how many to print?
	SUBWF	MCOUNT,W ; set C if not printing
	SWAPF	INDF1,W ; get the high nibble first
	BTFSC	MCOUNT,0 ; odd or even?
	MOVIW	FSR1++ ; odd, get low nibble and advance pointer
	ANDLW	0x0F ; isolate the digit
	IORWF	REGC,F ; will be zero if all leading zeros
	BTFSC	STATUS,C ; do we want to print this?
	BRA	PDendL ; no
	BTFSC	STATUS,Z ; skip if not leading zero
	BRA	PDdecPt
;
	IORLW	0x30 ; make it ASCII
	CALL	PrintC
PDdecPt:
	SWAPF	MTEMP,W
	ANDLW	0x0F ; decimal places
	ADDLW	0x01
	XORWF	MCOUNT,W ; are we one before decimal point required?
	ANDLW	0x07 ; decimal places
	BTFSS	STATUS,Z
	BRA	PDendL
	MOVLW	A'0'
	MOVF	REGC,F ; have all zeros been suppressed?
	BTFSC	STATUS,Z ; skip if non zero has been sent
	CALL	PrintC ; all suppressed, send a zero
	INCF	REGC,F ; ensure all printed after this
	DECF	MCOUNT,W ; set Z if this is last character
	MOVLW	A'.'
	BTFSS	STATUS,Z ; if this is the last character, no point
	CALL	PrintC ; not last, send the decimal point
PDendL:
	DECFSZ	MCOUNT,F
	BRA	PDloop
	MOVF	PMsave,W
	MOVWF	BSR
	RETURN
;
TXcvhex:
; oonvert lower 4 bits of W into a hex character
	ANDLW   0x0F	; remove the other nibble
	ADDLW   0x06    	; DC overflow if > 9
	BTFSC   STATUS,DC	
	ADDLW   0x07	; this is the gap between characters 9 and A
	ADDLW   0x2A	; convert to an ASCII hex character 0-9,A-F
; drop through to save the character
PrintC:
; saves the character in W in the transmit buffer and updates
; the pointer.
; This may be called from background or interrupt, so only
; uses registers that are saved and restored by interrupts.
; Interrupts are disabled while changing TXnext
; use FSR0 as temporary store
	MOVWF	FSR0H ; save the character
	MOVF	BSR,W
	MOVWF	FSR0L ; save the bank
	BANKSEL	TXsend ; to get TXnext	
	BCF	INTCON,GIE ; no interrupts while using TXnext
; swap TXnext in W with caller's bank in FSR0L
	MOVF    TXnext,W
	XORWF	FSR0L,W ; W = difference mask
	XORWF	FSR0L,F ; flips F from char to TXnext value
	XORWF	FSR0L,W ; flips W from mask to bank
; reinstate caller's bank (temporarily)
	MOVWF   BSR
	MOVLW   high TXbufAddr
; swap the literal with the character
	XORWF	FSR0H,W ; W = difference mask
	XORWF	FSR0H,F ; flips F from char to high TXbufAddr
	XORWF	FSR0H,W ; flips W from mask to char
; now FSR0 points to the buffer, W has character, save
	MOVWI	FSR0++ ; into the buffer
; save caller's bank again to update TXnext
	MOVF	BSR,W
	MOVWF	FSR0H ; save the bank
	BANKSEL	TXsend ; to get TXnext	
	MOVF	FSR0L,W ; test the updated pointer
	XORLW	low TXbufAddr+TXbufsz	; test end of buffer
	BTFSC   STATUS,Z
	MOVLW	(low TXbufAddr+TXbufsz) ^ low TXbufAddr
	XORLW	low TXbufAddr+TXbufsz	    ; circular buffer
	MOVWF	TXnext
	BANKSEL	PIE1
	BSF	PIE1,TXIE   ; set the TX interrupt enable flag
	BSF	INTCON,GIE ; finished with TXnext
	MOVF	FSR0H,W ; retrieve the bank
	MOVWF	BSR
	RETURN
;
;##################### start arithmetic routines ##################
; The arithmetic functions operate on a stack. There are two sets
; of subroutines:
; 1a. Push subroutines move variables onto the stack
; 1b. Pop subroutines move variables off the stack
; Push and Pop routines have the generic form
;	CALL	pushX/popX
;	variable ; address of variable
;		 ; OR in the case of PushLit, a value
; All push/pop routines use PushPopSetup to retrieve the 
; variable address/value.
; 2. the arithmetic functions that retrieve stack variables and
;    return the result to the stack
;-------------------------------------------------------------

	

	errorlevel -302 ; Turn off banking message
; known tested (good) code
; PushPopSetup used when the CALL to a routine is followed by data (a pointer
; to a variable except for PushLit, when it is a value).
PushPopSetup:
; used by Push, Pop to set up FSR0 to top of stack and
; and FSR1 to the source/destination variable
	MOVF	BSR,W
	MOVWF	GenBank
; set FSR1 to the source/destination variable
	CALL	PMgetPointr
	BANKSEL	STACK
; set up FSR0 and MCOUNT expecting the caller will move a value to/from
; the math stack
	MOVLW	4
	MOVWF	MCOUNT
	MOVLW	high STACK
	MOVWF	FSR0H
	MOVF	MTstackPtr,W
	MOVWF	FSR0L
	RETURN
;------------------
; Push entry points for various size variables.
; Push 1 to 4 expect signed variables
;
PushLit:
; entry to push an unsigned constant
	CALL	PushPopSetup
; the data now in PMDATL,PMDATH. Just set a pointer to it
; and treat it as any other 2 byte memory push
	MOVLW	low PMDATL ;
	MOVWF   FSR1L
	MOVLW   high PMDATL
	MOVWF   FSR1H
	BRA	PushEnt2

Push1U:
	CALL	PushPopSetup
	BRA	PushEnt1
	
Push2U:
	CALL	PushPopSetup
	BRA	PushEnt2

Push2:
	CALL	PushPopSetup
	BSF	MCOUNT,7 ; signed push
	BRA	PushEnt2

Push3U:
	CALL	PushPopSetup
	BRA	PushEnt3
Push3:
	CALL	PushPopSetup
	BSF	MCOUNT,7 ; signed push
	BRA	PushEnt3

Push4:
	CALL	PushPopSetup
; Push 4 does not need sign propagation
	MOVIW	FSR1++
	MOVWI	FSR0++
	DECF	MCOUNT,F ; 
PushEnt3:
	MOVIW	FSR1++
	MOVWI	FSR0++
	DECF	MCOUNT,F
PushEnt2:
	MOVIW	FSR1++
	MOVWI	FSR0++
	DECF	MCOUNT,F ; 
PushEnt1:
	MOVIW	FSR1++
	MOVWI	FSR0++
	DECF	MCOUNT,F
	BTFSC	STATUS,Z
	BRA	PushExit
; W has MSB of number. If signed, bit 7 is the sign
	ANDWF	MCOUNT,W ; bit 7 set if -ve and MCOUNT,7 set 
	BCF	MCOUNT,7 ; jst leave the count
	ANDLW	0x80 ; Z set +ve Z clear -ve
	BTFSS	STATUS,Z
	MOVLW	0xFF
Fillx:
	MOVWI	FSR0++
	DECFSZ	MCOUNT,F
	BRA	Fillx
PushExit:
	MOVF	FSR0L,W
	MOVWF	MTstackPtr
	MOVF	GenBank,W
	MOVWF	BSR
	RETURN
;--------------------------
; Pop entry points for 3 or 4 byte variables
; 3 byte values are truncated with no testing
;
Pop4:
	CALL	PushPopSetup
	ADDFSR	0,-4
	MOVF	FSR0L,W
	MOVWF	MTstackPtr
	MOVIW	FSR0++
	MOVWI	FSR1++
	BRA	Put3
Pop3:
	CALL	PushPopSetup
	ADDFSR	0,-4
	MOVF	FSR0L,W
	MOVWF	MTstackPtr
Put3:
	MOVIW	FSR0++
	MOVWI	FSR1++
	MOVIW	FSR0++
	MOVWI	FSR1++
	MOVIW	FSR0++
	MOVWI	FSR1++
	MOVF	GenBank,W
	MOVWF	BSR
	RETURN
;---------------------------
; The following functions are based on:
;
;*** SIGNED 32-BIT INTEGER MATHS ROUTINES FOR PIC16 SERIES BY PETER HEMSLEY ***
;
;Functions:
;	add
;	subtract
;	multiply
;	divide
;	round
;
; These were NOT implemented: sqrt, bin2dec, dec2bin
;
; The original routines mostly used lower case for instructions. Additions or
; changes are mostly in upper case. Almost all the changes are to the division
; routine where the role of REGB and REGC are reversed so the reminder is
; left in REGB (if changing the register name was the only modification the
; instruction is still in lower case). The round routine was mostly rewritten
; to save duplicating existing code. Apart from divide, return is via
; PopStk. PopStk does not affect the state of the C flag.

GetAB:
; first step in any function, fetch the A and B registers off the stack
; the B register is top of stack, A is next.
	MOVF	BSR,W	; save the caller's bank
	BANKSEL	STACK
	MOVWF	PMsave
; get the top of stack pointer	
	MOVLW	high STACK
	MOVWF	FSR0H
	MOVF	MTstackPtr,W
	MOVWF	FSR0L
; fetch top of stack into register B
	MOVIW	--FSR0
	MOVWF	REGB+3
	MOVIW	--FSR0
	MOVWF	REGB+2
	MOVIW	--FSR0
	MOVWF	REGB+1
	MOVIW	--FSR0
	MOVWF	REGB
; fetch register A
	MOVIW	--FSR0
	MOVWF	REGA+3
	MOVIW	--FSR0
	MOVWF	REGA+2
	MOVIW	--FSR0
	MOVWF	REGA+1
	MOVIW	--FSR0
	MOVWF	REGA
; FSR0 is left at the current position to allow A to be put back
	RETURN

PutA:
; this is called after a function completes. Any function error
; results in C being set. Save the value of C to report back to
; the caller
        RLF	PMsave,F ; add value of C to save
; put register A on the stack
	MOVF	REGA,W
	MOVWI	FSR0++
	MOVF	REGA+1,W
	MOVWI	FSR0++
	MOVF	REGA+2,W
	MOVWI	FSR0++
	MOVF	REGA+3,W
	MOVWI	FSR0++
	MOVF	FSR0L,W
	MOVWF	MTstackPtr
; get back the C and reinstate caller's bank
	LSRF	PMsave,W
	MOVWF	BSR
	RETURN
;
;*** 32 BIT SIGNED SUBTRACT ***
;REGA - REGB -> REGA
;Return carry set if overflow

subtract
	CALL	GetAB
	call	negateb		;Negate REGB
	skpnc
	GOTO	PutA		;Overflow
	BRA	Add2
;*** 32 BIT SIGNED ADD ***
;REGA + REGB -> REGA
;Return carry set if overflow

addm
	CALL	GetAB
Add2:
	movf	REGA+3,w		;Compare signs
	xorwf	REGB+3,w
	movwf	MTEMP

	call	addba		;Add REGB to REGA

	clrc			;Check signs
	movf	REGB+3,w		;If signs are same
	xorwf	REGA+3,w		;so must result sign
	btfss	MTEMP,7		;else overflow
	addlw	0x80
	GOTO	PutA

;*** 32 BIT SIGNED MULTIPLY ***
;REGA * REGB -> REGA
;Return carry set if overflow

multiply
	CALL	GetAB
	clrf	MTEMP		;Reset sign flag
	call	absa		;Make REGA positive
	skpc	; if it was -2,147,483,648
	call	absb		;Make REGB positive
	skpnc
	GOTO	PutA	;Overflow

;Move REGA to REGC
;Used by multiply

	movf	REGA,w	; code variation: this was in a subroutine,
	movwf	REGC		; but was moved inline
	movf	REGA+1,w
	movwf	REGC+1
	movf	REGA+2,w
	movwf	REGC+2
	movf	REGA+3,w
	movwf	REGC+3

;Clear REGA
;Used by multiply

	clrf	REGA		;Clear product
	clrf	REGA+1		; code variation: this was in a subroutine,
	clrf	REGA+2		; but was moved inline
	clrf	REGA+3
	movlw	D'31'		;Loop counter
	movwf	MCOUNT

muloop
	call	sla		;Shift left product and multiplicand
	rlf	REGC,f	; code variation: this was in a subroutine,
	rlf	REGC+1,f	; but was moved inline
	rlf	REGC+2,f
	rlf	REGC+3,f

	rlf	REGC+3,w		;Test MSB of multiplicand
	skpnc			;If multiplicand bit is a 1 then
	call	addba		;add multiplier to product

	skpc			;Check for overflow
	rlf	REGA+3,w
	skpnc
	GOTO	PutA

	decfsz	MCOUNT,f	;Next
	goto	muloop

	btfsc	MTEMP,0		;Check result sign
	call	negatea		;Negative
	GOTO	PutA


;*** 32 BIT SIGNED DIVIDE ***
;REGA / REGB -> REGA
;Remainder in REGB
;Return carry set if overflow or division by zero
divmore
	CALL	GetAB
	movlw	D'47'	;Loop counter
	BRA	divmain
divide
	CALL	GetAB
	movlw	D'31'	;Loop counter
divmain	
	movwf	MCOUNT
	clrf	MTEMP		;Reset sign flag
	call	absb		;Make divisor (REGB) positive
	skpnc
	GOTO	dvend		;Overflow
;
; modification - so the remainder ends up on the stack, REGB is moved to
; REGC. The use of REGB and REGC is the opposite of the original code but
; the logic remains the same
;
	MOVF	REGB,w		; Move REGB (divisor) to REGC at the
	MOVWF	REGC	; same time test for zero divisor
	MOVF	REGB+1,w
	MOVWF	REGC+1
	IORWF	REGB,f
	MOVF	REGB+2,w
	MOVWF	REGC+2
	IORWF	REGB,f
	MOVF	REGB+3,w
	MOVWF	REGC+3
	IORWF	REGB,w
;
	sublw	0	; if all zero, will set C
	skpc
	call	absa		;Make dividend (REGA) positive
	skpnc
	GOTO	dvend			;Overflow
; clear REGB to take the remainder
	clrf	REGB		;Clear remainder
	clrf	REGB+1
	clrf	REGB+2
	clrf	REGB+3
	call	sla		;Purge sign bit


dvloop
	call	sla		;Shift dividend (REGA) msb into remainder (REGB)
	CALL	SlbTst		; shifts and tests remainder > divisor
	skpc			;Carry set if remainder >= divisor
	goto	dremlt

	movf	REGC,w		;Subtract divisor (REGC) from remainder (REGB)
	subwf	REGB,f
	movf	REGC+1,w
	skpc
	incfsz	REGC+1,w
	subwf	REGB+1,f
	movf	REGC+2,w
	skpc
	incfsz	REGC+2,w
	subwf	REGB+2,f
	movf	REGC+3,w
	skpc
	incfsz	REGC+3,w
	subwf	REGB+3,f
	clrc
	bsf	REGA,0		;Set quotient bit

dremlt
	decfsz	MCOUNT,f	;Next
	goto	dvloop

	btfsc	MTEMP,0		;Check result sign
	call	negatea		;Negative
;	CALL	PutA
;	GOTO	PutB

;*** ROUND RESULT OF DIVISION TO NEAREST INTEGER ***

; round - in this program all divides round
; modified from original. Some code duplication was noticed so some was
; put in subroutine SlbTst and the IncA entry to negatea was added.
; No error testing, should not be capable of creating an error
;	CALL	GetAB
	clrf	MTEMP		;Reset sign flag
	call	absa		;Make positive
	clrc
	CALL	SlbTst  	; shifts and tests remainder > divisor
	CLRW			; prevent IncA from returning an error
	BTFSC	STATUS,C	; Carry set if remainder >= divisor
	CALL	IncA		; Increment REGA
	btfsc	MTEMP,0		;Restore sign
	call	negatea
dvend:
	GOTO	PutA

;UTILITY ROUTINES


;Add REGB to REGA (Unsigned)
;Used by add, multiply,

addba	movf	REGB,w		;Add lo byte
	addwf	REGA,f

	movf	REGB+1,w		;Add mid-lo byte
	addwfc	REGA+1,f		;Add and propagate carry_out

	movf	REGB+2,w		;Add mid-hi byte
	addwfc	REGA+2,f

	movf	REGB+3,w		;Add hi byte
	addwfc	REGA+3,f
	return


;Check sign of REGA and convert negative to positive
;Used by multiply, divide, round

absa	rlf	REGA+3,w ; is sign bit = 1 (-ve)
	skpc
	return			;Positive

;Negate REGA
;Used by absa, multiply, divide, round

negatea	movf	REGA+3,w		;Save sign in w
	andlw	0x80

	comf	REGA,f		;1's complement
	comf	REGA+1,f
	comf	REGA+2,f
	comf	REGA+3,f
	incf	MTEMP,f		;flip sign flag
IncA ; new entry point from round routine
	incfsz	REGA,f	; 1's complement to 2's complement
	goto	negafin
	incfsz	REGA+1,f
	goto	negafin
	incfsz	REGA+2,f
	goto	negafin
	incf	REGA+3,f
negafin
	addwf	REGA+3,w		;Return carry set if -2147483648
	return


;Check sign of REGB and convert negative to positive
;Used by multiply, divide

absb	rlf	REGB+3,w
	skpc
	return			;Positive

;Negate REGB
;Used by subtract, absb (multiply, divide)

negateb	movf	REGB+3,w		;Save sign in w
	andlw	0x80

	comf	REGB,f		;2's complement
	comf	REGB+1,f
	comf	REGB+2,f
	comf	REGB+3,f
	incf	MTEMP,f		;flip sign flag
	incfsz	REGB,f
	goto	negbfin
	incfsz	REGB+1,f
	goto	negbfin
	incfsz	REGB+2,f
	goto	negbfin
	incf	REGB+3,f
negbfin
	addwf	REGB+3,w		;Return carry set if -2147483648
	return

SlbTst:
;
; code modification: moved from divide, used by divide, round
; shifts remainder - when dividing, shifts in a bit from REGA;
; if rounding, a zero bit . Then tests remainder => divisor
;
	rlf	REGB,f ; shift
	rlf	REGB+1,f
	rlf	REGB+2,f
	rlf	REGB+3,f
	movf	REGC+3,w	; Test
	subwf	REGB+3,w
	skpz
	RETURN
	movf	REGC+2,w
	subwf	REGB+2,w
	skpz
	RETURN
	movf	REGC+1,w
	subwf	REGB+1,w
	skpz
	RETURN
	movf	REGC,w
	subwf	REGB,w
	RETURN


;Shift left REGA
;Used by multiply, divide, round

sla	rlf	REGA,f
	rlf	REGA+1,f
	rlf	REGA+2,f
	rlf	REGA+3,f
	return

; untested code
; *** END SIGNED 32-BIT INTEGER MATHS ROUTINES ***
;
; BIN2BCD - converts a 32-bit signed integer in REGA to a
; 10 digit packed decimal number in PackedD. If negative,
; bit 7 of PackedD is set (the largest number has a 2 in
; the most significant digit so there is no conflict)
BIN2BCD:
	MOVF	BSR,W
	BANKSEL	STACK
	MOVWF	PMsave
	CLRF	PackedD
	CLRF	PackedD+1
	CLRF	PackedD+2
	CLRF	PackedD+3
	CLRF	PackedD+4
	CLRF	MTEMP
	MOVLW	D'32'
	MOVWF	MCOUNT
; test for -ve
	BTFSC	REGA+3,7 ; skip if +ve
	CALL	negatea
B2Bloop:
; the double dabble algorithm
	MOVLW	0x33 
	ADDWF	PackedD,F
	ADDWF	PackedD+1,F
	ADDWF	PackedD+2,F
	ADDWF	PackedD+3,F
	ADDWF	PackedD+4,F
	MOVLW	0xFD
	BTFSS	PackedD,3
	ADDWF	PackedD,F
	BTFSS	PackedD+1,3
	ADDWF	PackedD+1,F
	BTFSS	PackedD+2,3
	ADDWF	PackedD+2,F
	BTFSS	PackedD+3,3
	ADDWF	PackedD+3,F
	BTFSS	PackedD+4,3
	ADDWF	PackedD+4,F
	MOVLW	0xD0
;	BTFSS	PackedD,7
	ADDWF	PackedD,F ; always - max value is 2
	BTFSS	PackedD+1,7
	ADDWF	PackedD+1,F
	BTFSS	PackedD+2,7
	ADDWF	PackedD+2,F
	BTFSS	PackedD+3,7
	ADDWF	PackedD+3,F
	BTFSS	PackedD+4,7
	ADDWF	PackedD+4,F
;	RLF	REGA+3,W ; do this so REGA same at the end
	RLF	REGA,f
	RLF	REGA+1,f
	RLF	REGA+2,f
	RLF	REGA+3,f    ; Shift msb into carry
	RLF	PackedD+4,F   ; then into lsb of decimal register
	RLF	PackedD+3,F
	RLF	PackedD+2,F
	RLF	PackedD+1,F
	RLF	PackedD,F
	DECFSZ	MCOUNT,F
	BRA	B2Bloop
	BTFSC	MTEMP,0 ; gets set if negated
	BSF	PackedD,7
	MOVF	PMsave,W
	MOVWF	BSR
	RETURN

;#################################################################
;                    I N T E R R U P T S
;#################################################################
;
; all interrupts redirect the program to here
;
	ORG 0x0004
;
	CLRF	PCLATH
	BANKSEL PIR1 ; most interrupt flags in this bank
	BTFSS   PIR1,TMR2IF ; skip if TMR2 interrupt
	GOTO	INTrx
;
; ----------- start TMR2 interrupt process ---------
;
; This occurs every 25uS.
; On every entry:
;   1. Dither the PWM duty cycle
;   2. Update the count of TMR2 interrupts
;   3. After 40,000 interrupts (one second), the count is reset
;      and TMR1 is started to detect the next 1ppS
	BCF     PIR1,TMR2IF ; clear the interrupt flag

; Timer 2 determines the length of each PWM output. The duty cycle of
; each pulse is dithered each interrupt. In software, the PWM value is
; held as 24 bits. The most significant 10 bits are loaded into the PWM
; duty cycle register. The least significant 14 bits of the 3 byte PWM
; value are repeatedly added to the value Dithr. If Dithr overflows to
; the 15th bit then the duty cycle is increased by 1 bit for the next
; PWM pulse.
	
	BANKSEL PWM
	MOVF    PWM,W   ; Add the least significant
	ADDWF   Dithr,F   ; 14 bits of PWM to Dithr
	MOVF    PWM+1,W
	ANDLW   0x3F ; truncate to top 6 of 14 bits
	ADDWFC	Dithr+1,W
	MOVWF	Dithr+1
	ANDLW	0x40 ; isolate possible carry out
	XORWF	Dithr+1,F ; remove it from Dithr
	ADDWF	PWM+1,W ; now the most significant 10 bits
	MOVWF   PWM2DCL ; bits 0-5 ignored
	CLRW
	ADDWFC	PWM+2,W
	MOVWF   PWM2DCH ; and store it as top 8 bits of next pulse width
;
; update the loop counters
;
	INCFSZ  LoopC,F
	RETFIE
	INCFSZ  LoopC+1,F
	GOTO	LedUpdate
;
; end of 1 second
;
; start another second - 157 outer loops, the first of 64 TMR2
; interrupts and the rest of 256 = 256 * 156 + 64 = 40,000 loops.
; The counter counts up to zero
;
	MOVLW   -D'157'
	MOVWF   LoopC+1
	MOVLW   -D'64'
	MOVWF   LoopC
; copy the state for use by the mainline
	BANKSEL	TMR2 ; TMR1 and TMR2 in the same bank
	MOVF	HoldTMR1,W
	MOVWF	LastTMR1
	MOVF	HoldTMR1+1,W
	MOVWF	LastTMR1+1
	MOVF	CurrFlags,W
	MOVWF	Iflags
	CLRF	CurrFlags ; ready for the next second
    	BSF	PeriodEnd
; Mainline requests comparator to resync Loop. The comparator
; interrupt is disabled only when TMR1 is running (see below).
; Leaving here means TMR1 doesn't run so comparator will resync.
	BTFSC	ResetLoop
	RETFIE
;
; start TMR1
;
; There is a chance this interrupt is delayed by another. It is
; necessary that TMR1 value be consistent, so it is derived from
; TMR2. TMR1 is set to TMR2*4, because it is clocked 4 times faster.
; TMR1 runs until either it is stopped by a 1ppS pulse via the
; comparator or by the TMR1 overflow interrupt routine if there
; is no detected 1ppS.
;
	CLRF	TMR1H
	LSLF	TMR2,W ; multipy by 2
	MOVWF	TMR1L
	RLF	TMR1H,F
	LSLF	TMR1L,F ; and again (TMR1 = TMR2*4)
	RLF	TMR1H,F
	BSF	T1CON,TMR1ON ; start the timer
; inhibit -ve going comparator interrupts. Any 1ppS should be picked
; up by the TMR1 gate while the timer is running.
	BANKSEL CM1CON1 
	BCF	CM1CON1,C1INTN
	RETFIE
LedUpdate:
; LED is controlled by a bit pattern in LedBits. When high LoopC
; is B'xxxx x101' LedBits is shifted, the 0 or 1 that gets shifted
; out determines if LED is on (bit shifted out was 1) or off. The
; bit time is approx 50mS.
	MOVF	LoopC+1,W
	ANDLW	0x07
	XORLW	0x05
	BTFSS	STATUS,Z
	RETFIE
	BANKSEL	PORTA
	LSLF	LedBits+2,F
	RLF	LedBits+1,F
	RLF	LedBits,F
	BTFSS	STATUS,C
	BSF	LED1
	BTFSC	STATUS,C
	BCF	LED1
	RETFIE
;
; ------------ end TMR2 interrupt process ----------
;
INTrx:
; test for interrupt on change - software UART at 9600 baud
	BTFSS	INTCON,IOCIF; skip if this is IOC interrupt
	GOTO	INTtmr0
;
; ----------- start interrupt on change (IOC) - soft UART ---------
;
; The software UART uses both IOC and TMR0. Initially, the first
; byte of the character assembly area (RXassem) is filled with 1 bits
; and TMR0 interrupts disabled. A 1 to 0 transition on the input pin
; is accepted as start of character. A 0 bit is shifted in, and TMR0
; enabled to interrupt in 1.5 bit times. An IOC before TMR0 expires
; shifts in a new bit value, and TMR0 restarted to interrupt in 1.5
; bit times. If TMR0 expires before IOC, a repeat of the last bit value
; is shifted in and TMR0 is set to 1 more bit times. Either TMR0 or
; IOC can detect a properly framed character - because initially there
; were all 1 bits, the overflow bit (bit 7 of RXassem+1) only becomes
; 0 when the start bit is shifted out of RXassem.
	BANKSEL	IOCAF
; there's only one IOC source, no need to follow documentation
	BCF	IOCAF,IOCAF1
	BCF	STATUS,C ; assume a value 0 to shift in
	BANKSEL	PORTA
	BTFSS	SoftRXport ; new value = 1? could be stop bit
	GOTO	RXCshift ; no, it's a zero
	BTFSS	INTCON,TMR0IE ; yes: currently assembling a character?
	RETFIE	 ; no: shouldn't happen (may be logically impossible?)
	BTFSS	RXassem+1,7 ; overflow bit = 0?
; RXvalidChar handler is after all interrupt handlers
	GOTO	RXvalidChar ; yes, a legal frame
	BSF	STATUS,C ; still assembling: value 1 to shift in
RXCshift:
	RRF	RXassem,F ; collect the bit from C
	RRF	RXassem+1,F ; collect overflow
	MOVLW	-D'190' ; TMR0 overflow in 1.5 bit time
	MOVWF	TMR0
	BCF	INTCON,TMR0IF ; may need clearing
	BSF	INTCON,TMR0IE ; may need setting
; finished
	RETFIE
;
; ----------- end interrupt on change (soft UART) ---------
;
INTtmr0:
	BTFSS	INTCON,TMR0IF
	GOTO	INTtx
	BCF	INTCON,TMR0IF
; since TMR0 is always running, TMR0IF can get set but not
; be the source of an interrupt. Was an interrupt expected?
	BTFSS	INTCON,TMR0IE
	GOTO	INTtx
;
; ----------- start TMR0 interrupt handler ---------
;
; TMR0 interrupt occurs as part of the software UART when assembling
; a character. If a bit transition occurs 1 bit time after a previous
; transition, it is caught by the interrupt on change (IOC). The IOC
; sets TMR0 to interrupt 1-1/2 bit times after a transition so a longer
; delay between transitions causes a TMR0 interrupt and the previous
; bit value is repeated. The TMR0 interrupt also resets TMR0 to
; interrupt after one bit time later (an integer+1/2 bit times after
; the last transition).
;
; test for end of character
	BTFSC	RXassem+1,7 ; could be start bit,
	GOTO	RXTnotend ; not a start bit
	BTFSC	SoftRXport ; could be stop bit, skip if not
; RXvalidChar handler is after all interrupt handlers
	GOTO	RXvalidChar ; got a start and a stop
RXTnotend:
	ASRF	RXassem,F ; propagate last bit
	RRF	RXassem+1,F ; collect overflow
	MOVLW	-D'130' ; come back in 1 bit time
; After rolling over and causing an interrupt, TMR0 has incremented by
; an unknown amount. This routine can be delayed by other interrupts.
; 'Adding' -130 actually subtracts from the unknown amount so the next
; interrupt is one bit time after the previous.
	ADDWF	TMR0,F
	RETFIE
;
; ------------ end TMR0 interrupt process ----------
;	
INTtx:
; The software only sets TXIE if it has something to send.
	BANKSEL	PIE1
	BTFSS	PIE1,TXIE   ; test the interrupt enable flag
	GOTO	IntGateR    ; no: test for TMR1 gate interrupt
; Test if it is a UART TX interrupt
	BANKSEL	PIR1
	BTFSS	PIR1,TXIF ; skip if this is UART TX interrupt
	GOTO	IntGate	    ; no: test for TMR1 gate interrupt
; TXIF set by hardware if the transmitter can accept a character.
;
; ----------- start TX interrupt process ---------
;
; TXsend points to the next character to transmit.
; If TXsend = TXnext then there is no more so disable the interrupt.
; Otherwise, send the character and update the pointer.
;
	BANKSEL	TXsend
	MOVLW	high TXbufAddr ; point to the character to send
	MOVWF	FSR0H
	MOVF	TXsend,W
	MOVWF	FSR0L
	MOVF	INDF0,W ; retrieve the character
	MOVWF   TXREG	; and send it
	INCF	TXsend,W ; update the pointer
	XORLW	low TXbufAddr+TXbufsz ; test end of buffer
	BTFSC   STATUS,Z ; skip if not end
	; a value that resolves to low TXbuf after the following XOR  
	MOVLW	(low TXbufAddr+TXbufsz) ^ low TXbufAddr
	XORLW	low TXbufAddr+TXbufsz
	MOVWF	TXsend ; update the pointer
	XORWF	TXnext,W ; is there more to send?
	BTFSS	STATUS,Z ; skip if no
	RETFIE	; More to send. Leave interrupt enabled
; no more to send, disable interrupt
	BANKSEL	PIE1
	BCF	PIE1,TXIE   ; clear the interrupt enable flag
	RETFIE
;
; ----------- end TX interrupt process ---------
;
IntGateR:
	BANKSEL	PIR1
IntGate:    
	BTFSS	PIR1,TMR1GIF
	GOTO	IntTM1
;
; ------------ Start TMR1 gate interrupt process ----------
;
; The TMR1 gate interrupt indicates a 1ppS pulse is received
; and TMR1 is latched. Stop the timer and indicate via OK2process
; that the mainline can retrieve it. The comparator negative
; interrupt is enabled to detect rogue pulses.
;
	BCF	PIR1,TMR1GIF
	BCF	T1CON,TMR1ON ; stop the timer
	MOVF	TMR1L,W
 	MOVWF	HoldTMR1
	MOVF	TMR1H,W  ; indicates invalid data
	MOVWF	HoldTMR1+1
	BSF	CUTMR1cap
; enable the comparator interrupt
	BANKSEL CM1CON1 
	BSF	CM1CON1,C1INTN
	RETFIE
;
; ------------ End TMR1 gate interrupt process ----------
IntTM1:
	BTFSS   PIR1,TMR1IF ; skip if TMR1 overflow interrupt
	GOTO	IntCP1 ; no: test comparator interrupt
;
; ----------- start TMR1 overflow interrupt process ---------
;
; This occurs if TMR1 overflows. TMR1 is supposed to stop
; when gated by the comparator on arrival of a 1ppS pulse.
; Assume the pulse is missed. Stop the counter and flag
; a missed pulse.
;
	BCF	T1CON,TMR1ON ; stop the timer
	BCF     PIR1,TMR1IF ; clear the interrupt flag
	BSF	CuMissPulse
; if the pulse arrives after this, let the comparator
; interrupt pick it up
	BANKSEL CM1CON1 
	BSF	CM1CON1,C1INTN
	RETFIE
;
; ----------- end TMR1 overflow interrupt process ---------
IntCP1:
	BTFSS	PIR2,C1IF ; skip if comparator interrupt
	GOTO	IntURX ; no: test character from user
;
;  -------- Change of comparator state ------------
;
; The 1ppS is detected by the comparator. The comparator is supposed to
; gate TMR1 while TMR1 is running. However, TMR1 is clocked at 40MHz
; so overflows in about 1.6mS. TMR1 starts running when LoopC is zero,
; once a second. While TMR1 is running, this interrupt is inhibited.
; If this interrupt occurs and it is a High to Low transition (start
; of a 1ppS), LoopC is set up so it reaches zero 999.2mS later, and
; the next 1ppS should be detected by the TMR1 Gate interrupt.
; Only high to low interrupts are enabled.
; NOTE: comparator inputs are +ve DAC (1.9V) and -ve 1ppS.
; The output is therefore an inverted 1ppS
;
	BCF	PIR2,C1IF
; high to low is start of 1ppS pulse detected 'out of bounds'.
; Reset LoopC so the next 1ppS arrives while TMR1 is running.
	MOVLW   -D'157'
	MOVWF   LoopC+1
	MOVLW   -D'32'
	MOVWF   LoopC
	BCF	ResetLoop
	BANKSEL	CM1CON1 ; only need one interrupt
	BCF	CM1CON1,C1INTN ; so inhibit
        RETFIE
;
;  -------- End change of comparator state ------------
IntURX:
	BTFSS	PIR1,RCIF ; skip if user input
	GOTO	LostOCXO
;
;  -------- User input via serial port ------------
;
; User input is a TAB followed by a single character. This routine
; accepts and notes the TAB, accepts and stores the character. These
; are echoed and actioned by the background process when it has time.
; The current state is held as bits in UsrFlag.
;
	BCF	PIR1,RCIF
	BANKSEL	RCREG
	MOVLW	D'6' ; 6 seconds to input the next data
	MOVWF	Utimeout
	MOVF	RCREG,W ; get the character
	BTFSC	UsrTab ; has TAB been received?
	GOTO	URXcommand ; yes, this is a command
	XORLW	0x09 ; no TAB yet. Is this an TAB?
	BTFSS	STATUS,Z 
	RETFIE	; not TAB, ignore it
; Received TAB, save and clear logging flags so logging stops.
; The '>' prompt is output by the background process, not here
	MOVF	LogLev,W ; save the current log level
	MOVWF	LLsave
	CLRF	LogLev	; stop output so background can prompt
	MOVLW	0x01 ; set UsrTab, clear any other action
	MOVWF	UsrFlag
        RETFIE
URXcommand:
	MOVWF	UsrChar ; save for the background
	BSF	UsrCmd ; and flag it is received
	RETFIE
	
;  -------- End User Input ------------
LostOCXO:
	BTFSC	LostClk ; not a lost clock, go to spin
	RESET ; otherwise, restart
Spin:
; unresolved interrupt, shouldn't happen. This locks the system up
	GOTO    Spin
;
;#################################################################
;                     N M E A   M E S S A G E S
;#################################################################
;
; An assumed valid character has been assembled.
; Try to assemble a valid NMEA message.
;
RXvalidChar:
	BCF	INTCON,TMR0IE ; stop TMR0 interrupts
	MOVF	RXassem,W
	CLRF	RXassem ; set assembly to all 1 bits
	COMF	RXassem,F
	BANKSEL RXresult
; Can't allow this character to start processing while processing
; a previous one. Since there are approx 10,000 instructions between
; characters received (at 9600 baud) this is highly unlikely.
; But better to drop a character rather than crash the system
	BTFSC	RXcharFlag
	RETFIE	; drop the character
	BSF	RXcharFlag
; OK to process, store the character
	MOVWF	RXresult
; Save interrupted process status before processing the character.
; Processing a character could delay other interrupts, so this
; routine saves state and allows interrupts.
; set up pointer to shadow data (status saved by interrupt)
	MOVLW	high STATUS_SHAD
	MOVWF	FSR1H
	MOVLW	low STATUS_SHAD
	MOVWF	FSR1L
; and copy status of caller before enabling interrupts
	MOVF	INDF1,W
	MOVWF	Char_Shadow
	MOVIW	1[FSR1]
	MOVWF	Char_Shadow+1
	MOVIW	2[FSR1]
	MOVWF	Char_Shadow+2
	MOVIW	3[FSR1]
	MOVWF	Char_Shadow+3
	MOVIW	4[FSR1]
	MOVWF	Char_Shadow+4
	MOVIW	5[FSR1]
	MOVWF	Char_Shadow+5
	MOVIW	6[FSR1]
	MOVWF	Char_Shadow+6
	MOVIW	7[FSR1]
	MOVWF	Char_Shadow+7
; allow this thread to be interrupted
	BSF	INTCON,GIE
; has the user requested NMEA data be logged?
	BTFSS	LogNMEA
	GOTO	RXafter
	MOVF	RXresult,W
	BTFSC	LogDetail
	IORLW	0x80 ; set high bit to distinguish stream
	LCALL	PrintC
	CLRF	PCLATH
	
RXafter:
	
; characters are dealt with according to the value of NMctl
	MOVF	RXresult,W
 	MOVF	NMctl,W
	BRW
	GOTO	NMdollar ; NMctl = 0 - looking for a message start
	GOTO	NMprocChr ; NMctl = 1 - storing a message
; if NMctl = 2, process the checksum characters
; convert ascii hex to binary and remove it from the checksum
	MOVF	RXresult,W
	ADDLW   0xBF    ; 'A'-'F' becomes 0x00 - 0x05 with overflow
	BTFSS   STATUS,C
	ADDLW   0x07    ; '0'-'9' becomes 0xF6 - 0xFF
	ADDLW   0x0A    ; 0xF6 - 0x05 becomes 0x00 - 0x0F
	SWAPF   NMchkSum,F
	XORWF   NMchkSum,F
	DECFSZ	NMChkCnt,F
	GOTO    RXdone ; after first checksum character
; now processed both checksum characters. The checksum fails if it
; is not zero.
	BTFSC	STATUS,Z ; still valid from XOR
	GOTO	NMhandler ; valid message in buffer
NMreset:
; Arrive here if any error occurs during message assembly.
; One reason for being here is an unexpected $.
; drop through to NMdollar to cater for that case.
	CLRF	NMctl
NMdollar:
	MOVF	RXresult,W
	XORLW	A'$'
	BTFSS	STATUS,Z
	GOTO	RXdone ; not a $, ignore
	MOVF	NMbuf,W ; the available buffer address
	MOVWF	NMptr ; set the pointer to start of buffer
	CLRF	NMchkSum
	INCF	NMctl,F ; point to storing characters in a buffer
	GOTO	RXdone
NMstar:
	INCF	NMctl,F ; point to processing the checksum
	MOVLW	D'2' ; two checksum characters
	MOVWF	NMChkCnt
	GOTO	RXdone
NMprocChr:
	MOVF	RXresult,W
; test if it's a $ - maybe message was corrupt and the '*' was missed
	XORLW	A'$'
	BTFSC	STATUS,Z
	GOTO	NMreset
	XORLW	A'$' ^ A'*' ; test for an asterisk
	BTFSC	STATUS,Z
	GOTO	NMstar
 ; not a $ or *, store it
	MOVLW	high NMEA_BUF1 ; same as high NMEA_BUF2
	MOVWF	FSR0H
	MOVF	NMptr,W
	MOVWF	FSR0L
	ANDLW	0x7F ; test pointer for overflow
	SUBLW	(low NMEA_addr+NMEA_size) - 1
	BTFSS	STATUS,C
	GOTO	NMreset ; overflow, quit
	INCF	NMptr,F ; where to save a subsequent character
	MOVF	RXresult,W ; retrieve this character
	XORWF	NMchkSum,F ; update the checksum
	MOVWF	INDF0 ; store the character and leave
RXdone:
; reinstate status 
	BCF	INTCON, GIE ; Disable Global Interrupt
; some PIC processors needed this
	BTFSC	INTCON, GIE ; Global Interrupt Disabled?
	GOTO	RXdone ; NO, try again
; YES, continue with program flow
; FSR1 was not changed after saving the state
	MOVF	Char_Shadow,W
	MOVWF	INDF1
	MOVF	Char_Shadow+1,W
	MOVWI	1[FSR1]
	MOVF	Char_Shadow+2,W
	MOVWI	2[FSR1]
	MOVF	Char_Shadow+3,W
	MOVWI	3[FSR1]
	MOVF	Char_Shadow+4,W
	MOVWI	4[FSR1]
	MOVF	Char_Shadow+5,W
	MOVWI	5[FSR1]
	MOVF	Char_Shadow+6,W
	MOVWI	6[FSR1]
	MOVF	Char_Shadow+7,W
	MOVWI	7[FSR1]
	BCF	RXcharFlag
	RETFIE
;
; enter NMhandler with a valid NMEA message in the buffer
;
NMhandler:
	MOVLW	high NMEA_BUF1 ; same as high NMEA_BUF2
	MOVWF	FSR0H
	MOVF	NMptr,W
	MOVWF	NMvalidEnd
	MOVF	NMbuf,W
	MOVWF	NMvalidMsg
; switch receiving buffers, but first test the other buffer is free.
; the buffer should be free. When a buffer has a valid message it
; is not free until the message is processed. This should always
; complete before the buffer is required again. If, for reasons unknown,
; the message is still being processed this thread cannot start
; processing another message, so the current message is discarded
; and the next message received into the current buffer
	XORLW	0x80 ; address difference between 2 buffers
	MOVWF	FSR0L
	BTFSS	INDF0,7 ; test the other buffer available
	GOTO	NMreset
	MOVWF	NMbuf ; set up to receive next message
	CLRF	NMctl
; It is necessary to take another copy of the state of the
; interrupted mainline. The first copy was to allow any interrupt
; to be serviced while processing a character. Now another copy
; is made to allow character assembly of the next message be
; serviced while processing the current message.
NMstartMsg:
; No need to disable interrupts, only character processing affects
; the state copy, and it can't proceed until RXcharFlag is cleared.
	MOVF	Char_Shadow,W
	MOVWF	Msg_Shadow
	MOVF	Char_Shadow+1,W
	MOVWF	Msg_Shadow+1
	MOVF	Char_Shadow+2,W
	MOVWF	Msg_Shadow+2
	MOVF	Char_Shadow+3,W
	MOVWF	Msg_Shadow+3
	MOVF	Char_Shadow+4,W
	MOVWF	Msg_Shadow+4
	MOVF	Char_Shadow+5,W
	MOVWF	Msg_Shadow+5
	MOVF	Char_Shadow+6,W
	MOVWF	Msg_Shadow+6
	MOVF	Char_Shadow+7,W
	MOVWF	Msg_Shadow+7
	BCF	RXcharFlag ; allow character processing to continue
;
; Interested in xxRMC and xxGSV,x,1,
; RMC gives validity and date/time if active
;
; FSR0H already set up
	MOVF	NMvalidMsg,W
	MOVWF	FSR0L
	MOVIW	2[FSR0]
	XORLW	A'R' ; is it R
	BTFSC	STATUS,Z
	GOTO	NMrmc ; yes: test for xxRMC
	XORLW	A'R' ^ A'G' ; not R, test for G
	BTFSS	STATUS,Z
	GOTO	NMendMsg
; got a G, test for GSV	
	MOVIW	3[FSR0]
	XORLW	A'S'
	BTFSS	STATUS,Z
	GOTO	NMendMsg
	MOVIW	4[FSR0]
	XORLW	A'V'
	BTFSS	STATUS,Z
	GOTO	NMendMsg
; got GSV, is it the first?
	MOVIW	8[FSR0]
	XORLW	A'1'
	BTFSS	STATUS,Z
	GOTO	NMendMsg
; first one, note satellites in view
	MOVIW	D'10'[FSR0]
	MOVWF	MGsats
	MOVIW	D'11'[FSR0]
	MOVWF	MGsats+1
	MOVF	BSR,W
	BANKSEL	CurrFlags
	BSF	CUGotGSV
	MOVWF	BSR
	GOTO	NMendMsg
;
NMrmc:
; got an R, test for RMC	
	MOVIW	3[FSR0]
	XORLW	A'M'
	BTFSS	STATUS,Z
	GOTO	NMendMsg
	MOVIW	4[FSR0]
	XORLW	A'C'
	BTFSS	STATUS,Z
	GOTO	NMendMsg
	MOVF	BSR,W
	BANKSEL	CurrFlags
	BSF	CUGotRMC
	MOVWF	BSR
; set up the data destination pointer for the time
	MOVLW	high RXtime
	MOVWF	FSR1H
	MOVLW	low RXtime
	MOVWF	FSR1L
	MOVLW	6
	MOVWF	RXlimit
; got RMC, have to work through looking at commas
	ADDFSR	0,5 ; should be at the first comma
	CLRF	RXcommas
NMrmc1:
	MOVF	RXcommas,W
	BRW
	GOTO	NMcomChk ; the comma after xxRMC
	GOTO	NMfield ; saving the time
	GOTO	NMvORa ; check Void or Active
	GOTO	NMcomChk ; skip latitiude
	GOTO	NMcomChk ; lat N or S
	GOTO	NMcomChk ; skip longitude
	GOTO	NMcomChk ; lat E or W
	GOTO	NMcomChk ; skip speed
	GOTO	NMcomChk ; skip track angle
	GOTO	NMfield ; saving the date
	GOTO	NMrmcXit ; that's it, folks
NMvORa:
; set up the data destination pointer for the date
	MOVLW	low RXdate ; for next jump to NMfield
	MOVWF	FSR1L
	MOVLW	6
	MOVWF	RXlimit
; checking for A (active)
	MOVIW	FSR0++
	XORLW	A'A' ; is it an A?
	BTFSS	STATUS,Z
	BRA	NMcomChk
	MOVF	BSR,W
	BANKSEL	CurrFlags
	BSF	CUValFix
	MOVWF	BSR
; next character should be comma
	BRA	NMcomChk
;
NMfield:
; copying data up to next comma
	MOVIW	FSR0++
 ; test for a comma - cautious, according to specification
 ; there shouldn't be one in the length specified by RXlimit
	XORLW	A','
	BTFSS	STATUS,Z 
	XORLW	A',' ; restore if not comma, leave as 0x00 if comma
	MOVWI	FSR1++ ; does not affect status
	BTFSC	STATUS,Z
	BRA	NMendFld
	DECFSZ	RXlimit,F
	BRA	NMnextChar
; Through to NMnextChar if under limit
NMcomChk:
; looking for a comma ignoring data	
	MOVIW	FSR0++
	XORLW	A','
	BTFSC	STATUS,Z
NMendFld:
	INCF	RXcommas,F
	CLRF	INDF1 ; beware - not skipped
NMnextChar:
	MOVF	NMvalidEnd,W ; check we are still in the message
	SUBWF	FSR0L,W ; W = FSR0L - end, should be negative
	BTFSS	STATUS,C ; set if > or = 0
	GOTO	NMrmc1
; drop through if pointer goes beyond message
NMrmcXit:
; convert time and date to Packed Decimal
	MOVLW	low RXtime
	CALL	NMpack
	MOVLW	low RXdate
	CALL	NMpack
	
;	MOVLW
;NMpack1:
;	SWAPF	RXtime,W
;	ADDWF	RXtime+1,W
;	ADDLW	0xCD
;	MOVWF	RXtime
;	SWAPF	RXtime+2,W
;	ADDWF	RXtime+3,W
;	ADDLW	0xCD
;	MOVWF	RXtime+1
;	SWAPF	RXtime+4,W
;	ADDWF	RXtime+5,W
;	ADDLW	0xCD
;	MOVWF	RXtime+2
NMendMsg:
	BANKSEL Char_Shadow
	MOVLW	high STATUS_SHAD
	MOVWF	FSR1H
	MOVLW	low STATUS_SHAD
	MOVWF	FSR1L
	BCF	INTCON, GIE ; Disable Global Interrupt
; some PIC processors needed this
	BTFSC	INTCON, GIE ; Global Interrupt Disabled?
	GOTO	NMendMsg ; NO, try again
; YES, continue with program flow
	MOVF	Msg_Shadow,W
	MOVWF	INDF1
	MOVF	Msg_Shadow+1,W
	MOVWI	1[FSR1]
	MOVF	Msg_Shadow+2,W
	MOVWI	2[FSR1]
	MOVF	Msg_Shadow+3,W
	MOVWI	3[FSR1]
	MOVF	Msg_Shadow+4,W
	MOVWI	4[FSR1]
	MOVF	Msg_Shadow+5,W
	MOVWI	5[FSR1]
	MOVF	Msg_Shadow+6,W
	MOVWI	6[FSR1]
	MOVF	Msg_Shadow+7,W
	MOVWI	7[FSR1]
; mainline state restored, mark the current message buffer as available
	MOVLW	high NMEA_BUF1 ; same as high NMEA_BUF2
	MOVWF	FSR0H
	MOVF	NMvalidMsg,W
	MOVWF	FSR0L
	BSF	INDF0,7 ; set the buffer as available
	RETFIE
;
NMpack:
	MOVWF	FSR0L ; both pointers at beginning of field
	MOVWF	FSR1L
	MOVF	FSR1H,W
	MOVWF	FSR0H
	MOVLW	3	; pack 6 characters into 3
	MOVWF	RXlimit
NMpkLoop:
	SWAPF	INDF0,W
	INCF	FSR0L,F
	ADDWF	INDF0,W
	INCF	FSR0L,F
	ADDLW	0xCD
	MOVWI	FSR1++
	DECFSZ	RXlimit,F
	BRA	NMpkLoop
	RETURN
;#################################################################
;                         S E T U P
;#################################################################
; Two stage setup.
; 1. Set up some functionality that works on the internal clock
; 2. Switch to external clock and set up the rest
;
setup:
; Diagnostic 1: switch on the LED. If power is applied and the LED doesn't
; come on then there's a problem
; Diagnostic 2: Set up the control voltage output as 2.5V - in case the
; the external oscillator needs a control voltage to run
; Diagnostic 3: If the external oscillator isn't running, the program
; will loop between setup and RESET and the LED will flash quickly
	BANKSEL TRISA
	BCF	LED1   ; RA4 set to output to the LED ;
	BCF	PWMout ; RC3 PWM voltage to 10MHz OCXO ;
	BANKSEL PORTA
	BCF	LED1 ; turn on the LED
; set up TMR2 for the PWM period of 250 instructions
; Once PWM is started, the period is not changed, just the duty cycle
	MOVLW   0xF9 ; d'249' - TMR2 reset every 250 instructions,
	MOVWF   PR2         ; = 1000 Osc cycles = 40KHz
	BSF     T2CON,TMR2ON ; start timer
;
	BANKSEL PWM
	CLRF	PWM
	CLRF	PWM+1
 	MOVLW   0x7D ; d'125' - PWM Duty Cycle to start, 50%
 	MOVWF   PWM+2 ; working value once TMR2 interrupt enabled
	MOVWF   PWM2DCH ; set up 50% duty cycle before interrupts
	CLRF	PWM2DCL
	MOVLW	B'11010000' ; enable PWM2 inverted output
;	MOVLW	B'11000000' ; enable PWM2 normal output
; 1 1 0 0 0 0 0 0
; | | | | ---|--- 
; | | | |    +---  x = Unimplemented: Read as ?0?
; | | | +--------  1 = PWM output is active low ### INVERTED ###
; | | | +--------  0 = PWM output is active high ### not inverted ###
; | | +----------  0 = PWM Module Output Value bit
; | +------------  1 = Output to PWMx pin is enabled
; +--------------  1 = PWM module is enabled
	MOVWF	PWM2CON
;
; Could be running on internal or external oscillator. Switch to the
; internal oscillator for certainty.
;
	BANKSEL OSCCON
; SCS<1:0>: System Clock Select bits
	BSF	OSCCON,1 ; SCS<1:0> = 1x, switch to INTOSC
	BANKSEL PORTA
;
	CLRF	LoopC ; set up a delay loop
	MOVLW	0x20
	MOVWF	LoopC+1
loop01:
	DECFSZ  LoopC,F ; delay for a while
	BRA    loop01
	DECFSZ  LoopC+1,F
	BRA    loop01
	BCF	LostClk ; in case it was set
	BSF	LED1 ;turn off the LED
	BANKSEL OSCCON
; try to switch to external clock	
	MOVLW	0xFC ; mask
; SCS<1:0>: System Clock Select bits
	ANDWF	OSCCON,F ; SCS<1:0> = 00
	BANKSEL PORTA
	MOVLW	0x20
	MOVWF	LoopC+1
loop02:
	DECFSZ  LoopC,F ; if now on external clock, short delay
	BRA    loop02  ; if internal clock then off time = on time
	DECFSZ  LoopC+1,F ; and the bit test sends control back to setup
	BRA    loop02
	BTFSC	LostClk ; continue if we are now on the external clock
	RESET ; otherwise, back to setup
;
; End stage 1 setup.
; Should now be running on external clock
;
; ------ Set up to capture 1ppS pulses ----------
; uses the DAC, comparator and TMR1
;
; --- DAC ---
; Analog input on RC2 - all RCx inputs are analog at reset, only
; RC2 is used as input so no need to change ANSEL. Get the comparator
; +ve input from the DAC - set to approx 1.9V
; V = (DACCON0/32)*5[Vdd]
	BANKSEL	DACCON0
	MOVLW	D'12' ; set the voltage
	MOVWF	DACCON1
	MOVLW	B'10000000' ; enable DAC, source Vdd
; 1 x 0 0 0 0 x x
; |   | | -+-
; |   | |  +----- 00 = DAC Positive Source Vdd
; |   | +--------  0 = DAC not connected to the DACOUT2 pin
; |   +----------  0 = DAC not connected to the DACOUT1 pin
; +--------------  1 = DAC is enabled
	MOVWF	DACCON0
;
; --- Comparator ---
	MOVLW	B'00010010'
; 0 0 0 1 x 0 1 0
; | | -|-   --+-- 010 = CxVN connects to CxIN2- pin
; | |  +---------  01 = CxVP connects to DAC Voltage Reference
; | +------------   0 = No interrupt on a negative going edge 
; +--------------   0 = No interrupt on a positive going edge
	MOVWF	CM1CON1
	MOVLW	B'10000101'
; 1 x 0 0 x 1 0 1
; |   | |   | | + 1 = Comparator output to Timer1 is synchronous
; |   | |   | +-- 0 = Comparator hysteresis disabled
; |   | |   +---- 1 = Comparator operates in Normal-Power mode
; |   | +-------- 0 = Comparator output is not inverted
; |   +---------- 0 = CxOUT is internal only
; +-------------- 1 = Comparator is enabled
	MOVWF	CM1CON0
;
; --- TMR1 and gate control ---
; TMR1 counts the 40MHz clock, gated by the comparator
	BANKSEL	PORTA ; PORTA also bank for TMR1
	MOVLW	B'01000000'
; 0 1 0 0 0 0 x 0
; -+- -+- | |   + 0  = Stops Timer1 and clears Timer1 gate flip-flop
;  |   |  | +---- 0  = Synchronize with system clock (FOSC)
;  |   |  +------ 0  = Dedicated Timer1 oscillator circuit disabled
;  |   +--------- 00 = 1:1 Prescale value
;  +------------- 01 = Timer1 clock source is system clock (FOSC)
	MOVWF	T1CON
	MOVLW	B'11000010'
; 1 1 0 0 0 x 1 0
; | | | | |   -+- 10 = Source is Comparator 1
; | | | | +------  0 = Timer1 gate single-pulse has not been started
; | | | +--------  0 = Timer1 Gate Single-Pulse mode is disabled
; | | +----------  0 = Timer1 Gate Toggle mode is disabled
; | +------------  1 = Timer1 counts when gate is high
; +--------------  1 = Timer1 is controlled by the gate function
	MOVWF	T1GCON
;
; ----- Operation can be monitored on an RS232 terminal ----
; set up the UART for transmitting and receiving
;
	BANKSEL	BAUDCON
	MOVLW   0x08    ; use 16 bit generator (and high speed)
	MOVWF   BAUDCON
	MOVLW   0x04 ; 1041 = 9600 baud = 0x0411
	MOVWF	SPBRGH
	MOVLW   0x11
	MOVWF	SPBRGL
;
; ## DEBUG - 9600+ - There appears to be a problem with duplicated
; characters if the output is 9600, so increase by 0.5%
; 
;	MOVLW   0x04
;	MOVWF	SPBRGH
;	MOVLW   0x0C
;	MOVWF	SPBRGL
; ## DEBUG
	MOVLW   0x24
; x 0 1 0 x 1 x x
;   | | |   +- BRGH - use high speed clock
;   | | +----- SYNC - select asynchronous
;   | +------- TXEN - enable transmission
;   +--------- TX9  - select 8-bit transmission
	MOVWF   TXSTA
	BSF     RCSTA,CREN  ; Serial Port receive enable
	BSF     RCSTA,SPEN  ; Serial Port Enable bit
; set up the transmit buffer
	BANKSEL TXsend
	MOVLW   low TXbufAddr ; initialise the send and free pointers
	MOVWF   TXsend
	MOVWF	TXnext
; Pointer to store PWM change logging
	MOVLW	low lasthist
	MOVWF	LogPWM
	MOVLW	high lasthist
	MOVWF	LogPWM+1
	MOVLW	low history
	MOVWF	HistPWMptr
	MOVLW	high history
	MOVWF	HistPWMptr+1
	CLRF	HistPWMcnt ; start with no history
; set to log program events. If calibration data is valid, this
; is replaced by user's preference.
	MOVLW	0x03 ; programmed default, details and adjustments
	MOVWF	LogLev
	CLRF	UsrFlag ; no user input yet
;
; [BANK 0 for access to PORTA and TMR0]
	BANKSEL	RXassem
	CLRF	CurrFlags ; a bit of housekeeping
;
; ------ set up the software UART RX ------
;
; initialise the soft UART bit assembly area
; set to all 1 bits so
	MOVLW	0xFF ; they need to be shifted out
	MOVWF	RXassem ; before seeing a start bit
; drift calculation
; set up the value 86400 (seconds in a day) - 0x015180
	MOVLW	0x01
	MOVWF	DA86400+2
	MOVLW	0x51
	MOVWF	DA86400+1
	MOVLW	0x80
	MOVWF	DA86400
; need to wait 2 full days before applying drift correction,
; start calculation on 3rd date change
; part day <change> full day <change> full day <change+claculate>
	MOVLW	3
	MOVWF	DAwait
;
	CLRF	DAincrement
	CLRF	DAincrement+1	
;
; set up interrupt on change for RA1 [BANK 7]
;
	BANKSEL	IOCAP
	BSF	IOCAP,IOCAP1
	BSF	IOCAN,IOCAN1
;
; set up NMEA message pointers and buffers. The buffers are
; BANK 10 and 11, they share the same high address.
;
	MOVLW	low NMEA_BUF1 ; initially, data stored in first buffer
	MOVWF	NMbuf ; pointer to start of buffer
	CLRF	NMctl ; start looking for a $
	BCF	RXcharFlag ; required to allow assembly of a character
	BCF	IOCAF,IOCAF1 ; clear any pending IOC
;
	BANKSEL	NMEA_BUF2 ; mark the second buffer as not in use
	BSF	NMEA_BUF2,7
;
	BANKSEL	MGsats	; until $xxGSV arrives set
	MOVLW	A'0'	; number of satellites to zero
	MOVWF	MGsats
	MOVWF	MGsats+1
;	
	BANKSEL PIE1
	BSF	PIE1,RCIE ; enable serial receive
;
; TMR0 used to time the soft UART characters. Baud rate set by the
; prescaler. 1:8 - 9600 1:16 - 4800. One bit time at 9600 baud is 104.2uS.
; With a 10MHz clock, and prescale of 8, is about 130 counts of TMR0.
; The values 130 and 190 (1-1/2 bit time minus interrupt latency) are
; hard coded into the interrupt handlers.
;	
	MOVLW	B'11000010'
; 1 1 0 x 0 0 1 0
; | | | | | --+-- PS<2:0> - Prescaler Rate Select bits (010 = 1:8)
; | | | | +------- PSA    - Prescaler assigned to Timer0
; | | | +--------- TMR0SE - Timer0 Source Edge Select bit
; | | +----------- TMR0CS - Timer0 clock source (FOSC/4)
; | +------------- INTEDG - Interrupt on rising edge of INT (not used)
; +--------------- WPUEN  - weak pull-ups are disabled
	MOVWF	OPTION_REG	
;
; ------ end set up the software UART RX ------
;
	BSF     PIE2,OSFIE ; allow lost oscillator interrupt
	BSF     PIE1,TMR2IE ; enable TMR2 interrupt
; some general initialisation
	CLRF	Iflags ; IRC/Mainline communication
	CLRF	Mflags
	CLRF	LedBits+2 ; Mainline/TMR2
	CLRF	LedBits+1
	CLRF	LedBits
	BANKSEL	MTstackPtr ; Maths stack pointer
	MOVLW	Low STACK
	MOVWF	MTstackPtr
; enable all interrupts
	BANKSEL	PORTA
 	MOVLW   0xC8
; 1 1 0 0 1 0 0 0
; | | |   +------- IOCIE  - all IOC interrupts
; | | +----------- TMR0IE - TMR0 interrupt inhibited for now
; | +------------- PEIE   - allows peripheral interrupts
; +--------------- GIE    - allow any enabled interrupt
	MOVWF   INTCON
;
;----------- All setup complete, interrupts enabled ------------
;
	GoMain	main

;#################################################################
;               F I X E D   D A T A   I N   F L A S H
;#################################################################
Announce:   da	"\r\nGPSDO Created 2023-01-08\x00"
BadAT:	    da	" ?unknown\x00"
CalFinish:  da	"Calibration saved successfully\x00"
ControlV:   da	" Ctrl \x00"
CVandErr:   da	"V Err=\x00"
CVandSlope: da	"V Sensitivity \x00"
GPSvalid:   da	"\r\nGPS data is now valid.\x00"
HzAndOver:  da	"Hz over \x00"
HzPerV:	    da	" Volt/Hz\r\n\x00"
NoCalData:  da	"No calibration data - starting calibration\r\n\x00"
NoFix:	    da	" secs. Lost Fix:\x00"
NoHistory:  da	" no history yet!\x00"
NoPulse:    da  " No 1ppS:\x00"
Mperiod:    da	" Max. period: \x00"
PPbillion:  da	" ppb\x00"
ProgReq:    da	"\r\nUpdate via Xmodem\r\n\x00"
Prompt:	    da	"\r\n>\x00"
Receiving:  da	"Receiving serial data from GPS.\r\n\x00"
RejPulse:   da	" Rejected:\x00"
SatInView:  da	"Satellites in view:\x00"
Seconds:    da	" secs.\x00"
SigLost:    da	"\r\nToo many missing or bad pulses. Resetting\x00"
Success:    da	"Success!\r\n\x00"
TMR1Start:  da	"TMR1 value at start \x00"
TSdate:	    da	"\r\nTime Stamp: Date \x00"
TStime:	    da	" Time \x00"
TSutc:	    da	" UTC.\x00"
Uptime:	    da	"\r\nUp \x00"
VoltSave:   da	"\r\nVoltage copied: W to save\x00"
;
; Limit table for errors.
; For each level, there are two 16 bit limits
; The first is a phase limit - how far the average phase error
; can drift.
; The second is a frequency error limit
; The limits are compared to unajusted values in terms of 25us
; increments
;
LimTbl:
; Level 0 - over 2 seconds
	    data    low(32) ; 400ns
	    data    high(32)
	    data    low(9) ; 2.25Hz error
	    data    high(9)
; Level 1 - over 4 seconds
	    data    low(40) ; 250ns
	    data    high(40)
	    data    low(32) ; 1Hz error
	    data    high(32)
; Level 2 - over 8 seconds
	    data    low(56) ; 175ns
	    data    high(56) ; over 48 seconds - Level 4
	    data    low(30) ; 0.47Hz error
	    data    high(30)
; Level 3 - over 16 seconds
	    data    low(112) ; 175ns
	    data    high(112)
	    data    low(56) ; 0.22Hz error
	    data    high(56)
; Level 4 - over 32 seconds
	    data    low(192)  ; 150ns
	    data    high(192)
	    data    low(102) ; 0.1Hz error
	    data    high(102)
; Level 5 - over 64 seconds
	    data    low(320)  ; 125ns
	    data    high(320)
	    data    low(192) ; 0.047Hz error
	    data    high(192)
; Level 6 - over 128 seconds
	    data    low(512) ; 100ns
	    data    high(512)
	    data    low(360) ; 0.022Hz error
	    data    high(360)
; Level 7 - over 256 seconds
	    data    low(1024) ; 100ns
	    data    high(1024)
	    data    low(655) ; 0.01Hz error
	    data    high(655)
; Level 8 - over 512 seconds
	    data    low(1536) ; 75ns
	    data    high(1536)
	    data    low(1468) ; 0.0056Hz error
	    data    high(1468)
; Level 9 - over 1024 seconds
	    data    low(2048) ; 50ns
	    data    high(2048)
	    data    low(3355) ; 0.0032Hz error
	    data    high(3355)
; Level 10 - over 2048 seconds
	    data    low(4096) ; 50ns
	    data    high(4096)
	    data    low(7130) ; 0.0017Hz error
	    data    high(7130)
; Level 11 - over 2048 seconds
	    data    low(8192) ; 50ns
	    data    high(8192)
	    data    low(16777) ; 0.001Hz error
	    data    high(1677)
;--------------------------------------------------------------
;	   values below here not worked out
;--------------------------------------------------------------
; Level 12 - over 4096 seconds
	    data    low(0)
	    data    high(0) ; Hz error
	    data    low(0)
	    data    high(0) ; Hz error
;
;#################################################################
;                 B O O T S T R A P   L O A D E R
;#################################################################
Section18:  ORG	    BootLoader
;
bootstrap:
; switching to internal oscillator at 16MHz
	BANKSEL OSCCON
	MOVLW	B'00111111'
; x x 1 1 1 1 1 1
; | | ---+--- -+-
; | |    |     +--- SCS<1:0>: 1x = switch to INTOSC
; | |    +--------- IRCF<3:0>: 1111 = 16 MHz or 48 MHz HF
; | +-------------- SPLLMULT ignored when SCS = 1x
; +---------------- SPLLEN: ignored when SCS = 1x
	MOVWF	OSCCON
bsReady:
	MOVF	OSCSTAT,W
	ANDLW	0x91 ; testing PLL ready, oscillator steady
	XORLW	0x91
	BTFSS	STATUS,Z
	BRA	bsReady
;
; set up the UART for transmitting and receiving
;
	BANKSEL	BAUDCON
	CLRF	TXSTA
	CLRF	RCSTA
	MOVLW   0x08    ; use 16 bit generator (and high speed)
	MOVWF   BAUDCON ; 416 = 9600 baud = 0x01A0 for 16MHz
	MOVLW   0x01
	MOVWF	SPBRGH
	MOVLW   0xA0
	MOVWF	SPBRGL
	MOVLW   0x24
; x 0 1 0 x 1 x x
;   | | |   +- BRGH - use high speed clock
;   | | +----- SYNC - select asynchronous
;   | +------- TXEN - enable transmission
;   +--------- TX9  - select 8-bit transmission
	MOVWF   TXSTA
	BSF     RCSTA,CREN  ; Serial Port receive enable
	BSF     RCSTA,SPEN  ; Serial Port Enable bit
	CLRF	bsPass ; test pass = 0
;
bsStartPass:
	MOVF	RCREG,W ; in case there was a pending character
	CLRF	bsEOF ; EOF flag
	MOVLW	0xFF
	MOVWF	bsPrevAddr+2 ; no previous record
	MOVLW	bsXdatHigh ; Xmodem block linear address high
	MOVWF	FSR0H 	; do not change FSR0H after this
	CLRF	bsMemAddr+2 ; bsMemAddr, bsMemAddr+1 filled in
	CLRF	bsBlkCnt ; block counter
; Set up as if there is a checksum of zero on end of a previous
; (not real) record followed by some end of record character
	CLRF	bsRecSum
	CLRF	bsBytesLeft
	MOVLW	0xFC
	MOVWF	bsCharPtr ; start point for reading hex
	MOVWF	FSR0L
	MOVLW	A'0'
	MOVWF	INDF0
	INCFSZ	FSR0L,F
	BRA	$-2
; set up the Xmodem waiting protocol - send a NAK every three
; seconds until something is received. Time out in a minute.
	MOVLW	CharNAK
	MOVWF	bsSendChr
	MOVLW	36 ; send a NAK every 3 seconds
	MOVWF	bswtime
	MOVLW	20 ; we will wait a minute
	MOVWF	bsrepeat
; This is the main loop - fetch a word into bsProgWd and write
; it to bsMemAddr
bsNextPloc:
	MOVF	bsBytesLeft,F
	BTFSC	STATUS,Z
	BRA	bsRecChk ; end of record, check the checksum
	INCFSZ	bsMemAddr,F ; increment the address in program memory
	BRA	bsData
	INCFSZ	bsMemAddr+1,F
	BRA	bsData
	INCF	bsMemAddr+2,F
bsData:
; get the data to be placed at address bsMemAddr
	CALL	bsFetchHex 
	MOVWF	bsProgDat
; test if this address is to be excluded. The address is converted
; from a byte address to word address for testing (>>1)
	LSRF	bsMemAddr+2,W ; exclude any address > bootloader
	BTFSS	STATUS,Z
	BRA	bsNextPloc
	RRF	bsMemAddr+1,W
	ADDLW	-(high BootLoader)
	BTFSC	STATUS,C
	BRA	bsNextPloc
bsWritePrev:
; set FSR1H now for use later
	MOVLW	high bsMemBuf
	MOVWF	FSR1H
; test if there is a previous block
	INCF	bsPrevAddr+2,W
	BTFSC	STATUS,Z
	BRA	bsReadNext ; no previous, start a new one
; Is new address in same block as previous address?
; If not, write the block
	MOVF	bsPrevAddr,W
	XORWF	bsMemAddr,W
	ANDLW	0xC0 ; is address in the same 64 addresses
	MOVWF	bsTemp
	MOVF	bsPrevAddr+1,W
	XORWF	bsMemAddr+1,W
	IORWF	bsTemp,F
	MOVF	bsPrevAddr+2,W
	XORWF	bsMemAddr+2,W
	IORWF	bsTemp,W
	BTFSC	STATUS,Z ; BRA if same block
	BRA	bsOvrwrite
; yes there's a previous block
	BTFSC	bsPass,0 ; don't write on first pass
	CALL	bsBlkWrite
bsReadNext:
; read a block of 32 program memory locations into bsMemBuf
	INCF	bsMemAddr+2,W ; test if end, no block to read
	BTFSC	STATUS,Z
	BRA	bsEndIt ; no more blocks, wind up
; copy the data from program memory to the memory buffer
; have to divide address by 2 to get word address
	MOVF    bsMemAddr+2,W
	MOVWF	bsPrevAddr+2
	MOVF    bsMemAddr+1,W
	MOVWF	bsPrevAddr+1
	LSRF	bsPrevAddr+1,W
	MOVWF   PMADRH
	MOVF    bsMemAddr,W
; start address of 32 program locations (64 bytes)
	ANDLW	0xC0
	MOVWF	bsPrevAddr
	RRF	bsPrevAddr,W
	MOVWF   PMADRL
	MOVLW	low bsMemBuf
	MOVWF	FSR1L
bsReadLoop:
	BCF	PMCON1,CFGS ; Do not select Configuration Space
	BSF	PMCON1,RD ; Initiate read
	NOP ; Ignored (Figure 11-2)
	NOP ; Ignored (in the reference)
	MOVF	PMDATL,W
	MOVWI	FSR1++
	MOVF	PMDATH,W
	MOVWI	FSR1++
	INCF	PMADRL,F
	MOVF	PMADRL,W
	ANDLW	0x1F
	BTFSS	STATUS,Z
	BRA	bsReadLoop
bsOvrwrite:
; the address of new data is now an address in the block
; currently in bsMemBuf
	MOVF	bsMemAddr,W
	ANDLW	0x3F ; just use low 6 bits of address
	ADDLW	low bsMemBuf
	MOVWF	FSR1L
	MOVF	bsProgDat,W
	BTFSC	bsMemAddr,0
	ANDLW	0x3F ; high 6 bits of 14
	MOVWF	INDF1
	BRA	bsNextPloc
bsEndIt:
; bsBlkWait will deal with any remaining Xmodem blocks
; and eventually run out of data returning a time out
	CALL	bsBlkWait
	IORLW	0 ; waiting for a timeout
	BTFSC	STATUS,Z
	GOTO	bsEndIt
	BTFSC	bsPass,0 ; 
	RESET
	MOVLW	A'O'
	MOVWF	TXREG
	MOVLW	A'K'
	MOVWF	TXREG
	BSF	bsPass,0 
	BRA	bsStartPass
; End of hex record. Read the checksum, should give zero 
bsRecChk:
	CALL	bsFetchHex
	BTFSS	STATUS,Z
	RESET	; failed checksum
; looking for the next hex record in Xmodem data block
	MOVF	bsCharPtr,W
	ADDLW	12 ; minimum record size is 11 bytes+end of record
	BTFSS	STATUS,C ; if C set, run off end of block
	BRA	bsFindColon
	CALL	bsMoreRec ; get another block
	IORLW	0 ; test for OK return
	BTFSS	STATUS,Z
	RESET
;
bsFindColon:
; hex record starts with ':' - scan until one is found
	MOVF	bsCharPtr,W
	MOVWF	FSR0L
	MOVF	INDF0,W
	XORLW	A':'
	BTFSC	STATUS,Z
	BRA	bsGetLen
	INCFSZ	bsCharPtr,F
	BRA	bsFindColon
; if bsCharPtr hits zero, it has run off the end of the Xmodem
; block. bsRecChk should ensure there's enough data so this doesn't
; happen
	RESET	;
bsGetLen:
; record length is two hex characters
	MOVF	bsCharPtr,W
	ADDLW	3 ; length should be in the current block
	BTFSC	STATUS,C ; if C set, run off end of block
	RESET
	CALL	bsFetchHex ; number of data bytes
	ADDLW	3 ; 2 address bytes, type byte
	MOVWF	bsBytesLeft
	ASLF	bsBytesLeft,W ; each byte is 2 chars
	ADDLW	4 ; checksum, end rec, one advance
	ADDWF	bsCharPtr,W
	BTFSS	STATUS,C ; if C set, could run off end of
	; the Xmodem block, so fetch another
	BRA	bsInBlock
	CALL	bsMoreRec
	IORLW	0 ; test for OK return
	BTFSS	STATUS,Z
	RESET
; now the address (read big endian, stored little endian)
bsInBlock:
	CALL	bsFetchHex
	MOVWF	bsMemAddr+1
	CALL	bsFetchHex
	MOVWF	bsMemAddr
; and the type
	CALL	bsFetchHex
	BRW	; jump table for actions
	BRA	bsData ; 00 - program data
	BRA	bsEndData ; 01 - end of file
	MOVLW	1 ; 02 - extended address
	NOP	; 03 - shouldn't be any
bsExtAddr: ; 04 - extended address (02 here also as 01)
	MOVWF	bsShifts
	ASLF	bsShifts,F ; 1 becomes 2, 4 becomes 8
	ASLF	bsShifts,F ; 2 becomes 4, 8 becomes 16
; rest of data should be 2 bytes address
	CALL	bsFetchHex
	MOVWF	bsMemAddr+1
	CALL	bsFetchHex
	MOVWF	bsMemAddr
	CLRF	bsMemAddr+2
bsMulAddr:
	ASLF	bsMemAddr,F
	RLF	bsMemAddr+1,F
	RLF	bsMemAddr+2,F
	DECFSZ	bsShifts,F
	BRA	bsMulAddr
; extended address set up. Do checksum then fetch next record
	BRA	bsRecChk
bsEndData:
	MOVLW	0xFF
	MOVWF	bsMemAddr+2 ; indicate no more data
	BRA	bsWritePrev ; go to write any outstanding data
;
bsFetchHex:
; collapses 2 hex characters (0-9,A-F) into one byte returned in W
; 1st hex char
	INCF	bsCharPtr,W
	MOVWF	FSR0L
	ADDLW	1
	MOVWF	bsCharPtr ; pointer advanced by 2 characters
	SWAPF	INDF0,W
	ADDLW	0x55
	BTFSS	INDF0,6 ; test if letter
	ADDLW	0x71
; 2nd hex char
	INCF	FSR0L,F
	ADDWF	INDF0,W
	BTFSS	INDF0,6 ; test if letter
	ADDLW	0x07
	DECF	bsBytesLeft,F
	ADDWF	bsRecSum,F ; add to the checksum
	RETURN

bsBlkWrite:
; WRITE PROGRAM MEMORY - Substantially copied from doco
; Flash row erase
; 1F80h-1FFFh durable memory - erase 1FA0-1FBF
;----- mainly copied from the manual ------
; This row erase routine assumes the following:
; 1. A valid address within the erase row is loaded in ADDRH:ADDRL
	LSRF	bsPrevAddr+1,W
	MOVWF   PMADRH
	RRF	bsPrevAddr,W
	MOVWF   PMADRL
	BCF	PMCON1,CFGS ; Not configuration space
	BSF	PMCON1,FREE ; Specify an erase operation
	BSF	PMCON1,WREN ; Enable writes
	MOVLW	55h ; Start of required sequence to initiate erase
	MOVWF	PMCON2 ; Write 55h
	MOVLW	0AAh ;
	MOVWF	PMCON2 ; Write AAh
	BSF	PMCON1,WR ; Set WR bit to begin erase
	NOP ; NOP instructions are forced as processor starts
	NOP ; row erase of program memory.
; The processor stalls until the erase process is complete
; after erase processor continues with 3rd instruction
	BCF	PMCON1,WREN ; Disable writes
;----- end copied from the manual ------
; set up and write a row
;----- mainly copied from the manual ------
; the program writes 15 low bytes and a checksum to 16 locations. Only
; the low 8 bits written so they can be retrieved by indirect reads.
	MOVLW	D'31'
	MOVWF	bsTemp
; these should be set already	
;	MOVLW	low ConfigLoc ; Load lower 8 bits of erase address boundary
;	MOVWF	PMADRL
;	MOVLW	high ConfigLoc ; Load upper 6 bits of erase address boundary
;	MOVWF	PMADRH
;
	MOVLW	low bsMemBuf
	MOVWF	FSR1L
	BCF	PMCON1,CFGS ; Not configuration space
	BSF	PMCON1,WREN ; Enable writes
	BSF	PMCON1,LWLO ; Only Load Write Latches
bsLatchLd:
	MOVIW	FSR1++ ; Load data byte into lower
	MOVWF	PMDATL ;
	MOVIW	FSR1++ ; Load data byte into lower
	MOVWF	PMDATH ;
	MOVLW	55h ; Start of required write sequence:
	MOVWF	PMCON2 ; Write 55h
	MOVLW	0AAh ;
	MOVWF	PMCON2 ; Write AAh
	BSF	PMCON1,WR ; Set WR bit to begin write
	NOP	; NOP instructions are forced as processor
	NOP	; loads program memory write latches
	INCF	PMADRL,F ; Still loading latches Increment address
	DECFSZ	bsTemp,F
	BRA	bsLatchLd
; Write next latches
; START_WRITE:
	MOVIW	FSR1++ ; Load data byte into lower
	MOVWF	PMDATL ;
	MOVIW	FSR1++ ; Load data byte into lower
	MOVWF	PMDATH ;
	BCF	PMCON1,LWLO ; No more loading latches
	; - Actually start Flash program memory write
	MOVLW	55h ; Start of required write sequence:
	MOVWF	PMCON2 ; Write 55h
	MOVLW	0AAh ;
	MOVWF	PMCON2 ; Write AAh
	BSF	PMCON1,WR ; Set WR bit to begin write
; NOP instructions are forced as processor writes all the program memory
; write latches simultaneously to program memory.
	NOP
	NOP
; After NOPs, the processor stalls until the self-write process is complete
; after write processor continues with 3rd instruction
	BCF PMCON1,WREN ; Disable writes
	RETURN
;----- end copied from the manual ------
;####################################
;## retrieves 128 character blocks ##
;## using Xmodem protocol          ##
;####################################
bsMoreRec:
; shuffle remaining bytes of last block down 128 byte
    	MOVF	FSR0H,W
	MOVWF	FSR1H
	MOVF	bsCharPtr,W
	MOVWF	FSR0L
	ANDLW	0x7F ; move down 128 (0x80) bytes
	MOVWF	FSR1L
	MOVWF	bsCharPtr
bsShuffle:
; use INCFSZ rather than MOVIW FSR0++ so FSR0H does not change
	MOVF	INDF0,W
	MOVWI	FSR1++
	INCFSZ	FSR0L,F ; gone over limit yet?
	BRA	bsShuffle
; send ACK or NAK here either waiting for first
; block or in response to last received block
bsBlkWait:
	MOVF	bsSendChr,W
	BTFSS	TXSTA,TRMT
	BRA	$-1
	MOVWF	TXREG
	CLRF	LoopC ; set up the delay loop
	CLRF	LoopC+1
	MOVF	bswtime,W
	MOVWF	LoopC+2
	BANKSEL	PIR1
bsRXloop:
	BTFSC	PIR1,RCIF
	BRA	bsGetBlk
	DECFSZ	LoopC,F
	BRA	bsRXloop
	DECFSZ	LoopC+1,F
	BRA	bsRXloop
	DECFSZ	LoopC+2,F
	BRA	bsRXloop
	BANKSEL	TXSTA
	DECFSZ	bsrepeat,F
	BRA	bsBlkWait
	RETLW	1 ; timed out
; process next block, first character already waiting
bsGetBlk:
	BANKSEL	RCREG
	MOVLW	12
	MOVWF	bswtime  ; send a reply at 1 second intervals 
	MOVLW	3
	MOVWF	bsrepeat ; a maximum of 3 times
	MOVF	RCREG,W	; get the first character
	MOVWF	bsBlkSum ; initialise the checksum
	XORLW	CharEOT ; is this the last record?
	BTFSS	STATUS,Z
	BRA	bsChar2 ; no, another record
; end of file
	INCF	bsEOF,F 
	DECFSZ	bsEOF,W ; first EOF?
	BRA	bsEOF2
; first time through send a NAK
	MOVLW	CharNAK
	MOVWF	bsSendChr
	RETLW	0 ; return
bsEOF2:
; send one ACK, delay minimum. Should time out, the
; caller accepts a time out when all data has been processed
	MOVLW	1
	MOVWF	bswtime  ; wait about 300ms
	MOVWF	bsrepeat ; don't repeat
	MOVLW	CharACK
	MOVWF	bsSendChr
	BRA	bsBlkWait ; send ACK
	
bsChar2:
; test for SOH
	XORLW	CharEOT ^ CharSOH
	BTFSS	STATUS,Z
	BRA	bsBadBlk ; not SOH or EOT
; read record No.
	CALL	bsGetChar
	BTFSC	STATUS,C
	BRA	bsShortBlk ; bad block
	SUBWF	bsBlkCnt,W
	MOVWF	bsBlkPrev
; result should be 0 (previous block) or -1 (expected block)
	BTFSS	STATUS,Z
	ADDLW	1
	BTFSS	STATUS,Z
	BRA	bsBadBlk
; read 1s complement record No.
	CALL	bsGetChar
	BTFSC	STATUS,C
	BRA	bsShortBlk ; bad block
; result of add to checksum should be zero
	BTFSS	STATUS,Z
	BRA	bsBadBlk
; OK so far, get 128 bytes of data
; store data in linear memory starting at 0x2280
	MOVLW	0x80
	MOVWF	FSR0L
bsbk128:
	CALL	bsGetChar ; 128 chars
	BTFSC	STATUS,C
	BRA	bsShortBlk ; bad block
	MOVWF	INDF0
	INCFSZ	FSR0L,F
	BRA	bsbk128
; now check the checksum
	MOVF	bsBlkSum,W ; invert the checksum
	SUBLW	0 ; inverts W
	MOVWF	bsBlkSum ; an add should yield zero
	CALL	bsGetChar ; checksum
	BTFSC	STATUS,Z ; if checksum bad, skip
	BTFSC	STATUS,C ; checksum OK, skip if no timeout
	BRA	bsShortBlk ; should be no more characters
	MOVLW	CharACK ; good block
	MOVWF	bsSendChr
	INCFSZ	bsBlkPrev,F ; test if repeat of previous record
	BRA	bsBlkWait
; completed read of new record
	INCF	bsBlkCnt,F
	INCFSZ	bsPrevAddr+2,W ; maybe the first block
	RETLW	0 ; not first
; if this is the first block, keep reading input until there are no
; more characters. This caters for the case where the sender has buffered
; some initilisation NAKS and continues sending record 1
bsClrRec1:
	CALL	bsGetChar
	BTFSC	STATUS,C ; is there a character
	RETLW	0
	BRA	bsClrRec1 ; get another

bsBadBlk:
; get characters until timeout
	CALL	bsGetChar
bsShortBlk:
	BTFSS	STATUS,C ; is there a character
	BRA	bsBadBlk ; get another
	MOVLW	CharNAK
	MOVWF	bsSendChr
	BRA	bsBlkWait

;########################################################
	
bsGetChar:
	CLRF	LoopC ; next character to arrive within 300ms
	CLRF	LoopC+1
bsGChar1:
	BANKSEL	PIR1
	BTFSC	PIR1,RCIF
	BRA	bsGChar2
	DECFSZ	LoopC,F
	BRA	bsGChar1
	DECFSZ	LoopC+1,F
	BRA	bsGChar1
; timed out waiting for char in record
	BANKSEL	RCREG
	BSF	STATUS,C
	RETURN
bsGChar2:
	BANKSEL	RCREG
	MOVF	RCREG,W
	ADDWF	bsBlkSum,F
	BCF	STATUS,C
	RETURN
;
	end