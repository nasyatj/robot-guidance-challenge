; -------------------------------------------------------------------------------------------------
;   Nasya James  -  501014179
;   Alaa Yafaoui -  501052383
; -------------------------------------------------------------------------------------------------
              XDEF Entry, _Startup ; export �Entry� symbol
              ABSENTRY Entry ; for absolute assembly: mark
              INCLUDE "derivative.inc"

;EQUATES
; -------------------------------------------------------------------------------------------------

; Liquid Crystal Display Equates
;-------------------------------
CLEAR_HOME    EQU   $01                   ; Clear the display and home the cursor
INTERFACE     EQU   $38                   ; 8 bit interface, two line display
CURSOR_OFF    EQU   $0C                   ; Display on, cursor off
SHIFT_OFF     EQU   $06                   ; Address increments, no character shift
LCD_SEC_LINE  EQU   64                    ; Starting addr. of 2nd line of LCD (note decimal value!)

; LCD Addresses
; -------------
LCD_CNTR      EQU   PTJ                   ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT       EQU   PORTB                 ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E         EQU   $80                   ; LCD E-signal pin
LCD_RS        EQU   $40                   ; LCD RS-signal pin

; Other codes
; -----------
NULL          EQU   00                    ; The string �null terminator�
CR            EQU   $0D                   ; �Carriage Return� character
SPACE         EQU   ' '                   ; The �space� character


START         EQU   0
FWD           EQU   1
ALL_STOP      EQU   2
LEFT_TURN     EQU   3
RIGHT_TURN    EQU   4
REV_TURN      EQU   5                     ; LEFT_ALIGN & RIGHT_ALIGN are the states that allow 
LEFT_ALIGN    EQU   6                     ; the robot to align and correctly straighten out 
RIGHT_ALIGN   EQU   7                     

; Turning Timers
;---------------
T_LEFT        EQU   7
T_RIGHT       EQU   7

; variable/data section
; ---------------------
              ORG   $3800
              
              
; Storage Registers (9S12C32 RAM space: $3800 ... $3FFF)
; ------------------------------------------------------
SENSOR_LINE   FCB   $01                     ; Storage for guider sensor readings
SENSOR_BOW    FCB   $23                     ; Initialized to test values
SENSOR_PORT   FCB   $45
SENSOR_MID    FCB   $67
SENSOR_STBD   FCB   $89
SENSOR_NUM    RMB   1                       ; The currently selected sensor

; Initial values based on the initial readings & variance
; -------------------------------------------------------
BASE_LINE     FCB   $9D
BASE_BOW      FCB   $CA
BASE_MID      FCB   $CA                                                                                                        
BASE_PORT     FCB   $CA
BASE_STBD     FCB   $CA                                                                             
                                            
VAR_LINE      FCB   $18                     ; Utilizing variance for each sesnor so to ensure            
VAR_BOW       FCB   $30                     ; that the robot correctly adjust itself whenever                                   
VAR_PORT      FCB   $20                     ; it turns, straighten out, or performs a 180 turn
VAR_MID       FCB   $20
VAR_STBD      FCB   $15

TOP_LINE      RMB   20                      ; Top line of display
              FCB   NULL                    ; terminated by null
              
BOT_LINE      RMB   20                      ; Bottom line of display
              FCB   NULL                    ; terminated by null
              
CLEAR_LINE    FCC   '                  '    ; Clear the line of display
              FCB   NULL                    ; terminated by null
              
TEMP          RMB   1                       ; Temporary location


; variable section
; -------------------------------------------------------------------------------------------------
              ORG   $3850                   ; Where our TOF counter register lives
TOF_COUNTER   dc.b  0                       ; The timer, incremented at 23Hz
CRNT_STATE    dc.b  2                       ; Current state register
T_TURN        ds.b  1                       ; time to stop turning
TEN_THOUS     ds.b  1                       ; 10,000 digit
THOUSANDS     ds.b  1                       ; 1,000 digit
HUNDREDS      ds.b  1                       ; 100 digit
TENS          ds.b  1                       ; 10 digit
UNITS         ds.b  1                       ; 1 digit
NO_BLANK      ds.b  1                       ; Used in �leading zero� blanking by BCD2ASC
HEX_TABLE       FCC   '0123456789ABCDEF'        ; Table for converting values
BCD_SPARE     RMB   2

; code section
; -------------------------------------------------------------------------------------------------
              ORG   $4000
Entry:                                                                       
_Startup: 

;Initialization
              LDS   #$4000                 ; Initialize the stack pointer
              CLI                          ; Enable interrupts
              JSR   INIT                   ; Initialize ports
              JSR   openADC                ; Initialize the ATD
              JSR   initLCD                ; Initialize the LCD
              JSR   CLR_LCD_BUF            ; Write �space� characters to the LCD buffer                     I
              BSET  DDRA,%00000011         ; STAR_DIR, PORT_DIR                        
              BSET  DDRT,%00110000         ; STAR_SPEED, PORT_SPEED                    
                                                                                
              JSR   initAD                 ; Initialize ATD converter                  
                                                                                  
              JSR   initLCD                ; Initialize the LCD                        
              JSR   clrLCD                 ; Clear LCD & home cursor 
              
              JSR   ENABLE_TOF             ; Jump to TOF initialization
            
; -------------------------------------------------------------------------------------------------
; MAIN routine
;   update: display, state, sensors, motors(dispatcher)
; -------------------------------------------------------------------------------------------------
MAIN            JSR UPDT_DISPL
                LDAA CRNT_STATE      ;**
                JSR DISPATCHER       ;**
                JSR G_LEDS_ON        ; Enable the guider LEDs
                JSR READ_SENSORS     ; Read the 5 guider sensors
                JSR G_LEDS_OFF       ; Disable the guider LEDs
                JSR DISPLAY_SENSORS  ; and write them to the LCD
                BRA MAIN             ; Loop forever
              
; data section
; -------------------------------------------------------------------------------------------------
tab           dc.b  "START  ",0
              dc.b  "FWD    ",0
              dc.b  "ALL_STP",0
              dc.b  "LTURN  ",0
              dc.b  "RTURN  ",0
              dc.b  "REVTRN ",0
              dc.b  "LADJUST",0     
              dc.b  "RADJUST",0  
                 
; subroutine section
; -------------------------------------------------------------------------------------------------
;|                               Dispatcher                                                                  
; -------------------------------------------------------------------------------------------------
DISPATCHER        CMPA  #START                                ;  if the robot is in the 
                  BNE   NOT_START                             ; START state, else proceed to 
                  JSR   START_ST                              ;   
                  BRA DISP_EXIT                                         ; FORWARD state

NOT_START         CMPA  #FWD                                  ;  if the robot is in the 
                  BNE   NOT_FWD                               ; FORWARD state, else proceed to 
                  JSR   FWD_ST                                ;  
                  BRA DISP_EXIT                               ; ALL_STOP state

NOT_FWD           CMPA  #ALL_STOP                             ;  if the robot is in the
                  BNE   NOT_ALL_STP                           ; ALL_STOP state, else proceed to  
                  JSR   ALL_STOP_ST                           ;   
                  BRA DISP_EXIT                               ; LEFT_TURN state

NOT_ALL_STP       CMPA  #LEFT_TURN                            ;  if the robot is in the 
                  BNE   NOT_LEFT_TURN                         ; LEFT_TURN state, else proceed to
                  JSR   FULL_LEFT                             ;  
                  BRA DISP_EXIT                               ; RIGHT_TURN state

NOT_LEFT_TURN     CMPA  #RIGHT_TURN                           ;  if the robot is in the 
                  BNE   NOT_RIGHT_TURN                        ; RIGHT_TURN state, else proceed to 
                  JSR   FULL_RIGHT                            ;  
                  BRA DISP_EXIT                               ; REV_TURN state
                  
NOT_RIGHT_TURN    CMPA  #REV_TURN                             ;  if the robot is in the 
                  BNE   NOT_REV_TURN                          ; REV_TURN state, else proceed to 
                  JSR   REV_TURN_ST                           ;  
                  BRA DISP_EXIT                               ; LEFT_ALIGN state
                  
NOT_REV_TURN      CMPA  #LEFT_ALIGN                           ;  if the robot is in the 
                  BNE   NOT_LEFT_ALIGN                        ; LEFT_ALIGN state, else proceed to 
                  JSR   CONTINUE_FORWARD                      ; 
                  BRA DISP_EXIT                               ; RIGHT_ALIGN state

NOT_LEFT_ALIGN    CMPA  #RIGHT_ALIGN                          ;  if the robot is in the 
                  BNE   NOT_RIGHT_ALIGN                       ; RIGHT_ALIGN state, else proceed to
                  JSR   CONTINUE_FORWARD                      ;  
                  BRA DISP_EXIT                               ; INVALID state
                      
NOT_RIGHT_ALIGN   SWI                                         ; Break
DISP_EXIT         RTS                                         ; Exit from the state dispatcher
; -------------------------------------------------------------------------------------------------

; -------------------------------------------------------------------------------------------------
; Start when front bumper is pressed                                                                      
; -------------------------------------------------------------------------------------------------
START_ST          BRCLR   PORTAD0, %00000100, NO_START_RELEASE                                    ;|
                  JSR     INIT_FWD                                                                ;|
                  MOVB    #FWD, CRNT_STATE                                                        ;|
NO_START_RELEASE  RTS                                                                             ;|
; -------------------------------------------------------------------------------------------------


; -------------------------------------------------------------------------------------------------
; Forward  State
;   - check bumpers
;   - make adjustments using EF                                       
; -------------------------------------------------------------------------------------------------               
FWD_ST            BRSET   PORTAD0, %00000100, NO_FWD_BUMP       ;If FWD_BUMP                       |                          
                  MOVB    #REV_TURN, CRNT_STATE                                                   ;|                        
                                                                                                  ;|               
                  JSR     UPDT_DISPL                                                              ;|                     
                                                                                                  ;|
                  JSR     INIT_REV                                                                ;|
                  LDY     #6000                                                                   ;|
                  JSR     del_50us                                                                ;|
                                                                                                  ;|
                  JSR     INIT_RIGHT                                                              ;|
                  LDY     #6000                                                                   ;|
                  JSR     del_50us                                                                ;|
                  LBRA    EXIT                                                                    ;|
                                                                                                  ;|
NO_FWD_BUMP       BRSET   PORTAD0, %00001000, NO_FWD_REAR_BUMP                                    ;| 
                  MOVB    #ALL_STOP, CRNT_STATE                                                   ;|                    
                  JSR     INIT_STOP                                                               ;|
                  LBRA    EXIT                                                                    ;|                                                               |
                                                                                                  ;|
NO_FWD_REAR_BUMP  LDAA    SENSOR_BOW                                                              ;|
                  ADDA    VAR_BOW                                                                 ;|
                  CMPA    BASE_BOW                                                                ;|
                  BPL     NO_ALIGN                                                                ;|
                                                                                                  ;|
                  LDAA    SENSOR_MID                                                              ;|
                  ADDA    VAR_MID                                                                 ;|
                  CMPA    BASE_MID                                                                ;|
                  BPL     NO_ALIGN                                                                ;|
                                                                                                  ;|
                  LDAA    SENSOR_LINE                                                             ;|
                  ADDA    VAR_LINE                                                                ;|
                  CMPA    BASE_LINE                                                               ;|
                  BPL     GO_RIGHT_ALIGN                                                          ;|
                                                                                                  ;|
                  LDAA    SENSOR_LINE                                                             ;|
                  SUBA    VAR_LINE                                                                ;|
                  CMPA    BASE_LINE                                                               ;|
                  BMI     GO_LEFT_ALIGN                                                           ;|
                                                                                                  ;|
NO_ALIGN          LDAA    SENSOR_PORT                                                             ;|
                  ADDA    VAR_PORT                                                                ;|
                  CMPA    BASE_PORT                                                               ;|
                  BPL     PART_LEFT_TURN                                                          ;|
                  BMI     NO_PORT_DET                                                             ;|
                                                                                                  ;|
NO_PORT_DET       LDAA    SENSOR_BOW                                                              ;|
                  ADDA    VAR_BOW                                                                 ;|
                  CMPA    BASE_BOW                                                                ;|
                  BPL     EXIT                                                                    ;|
                  BMI     NO_BOW_DET                                                              ;|
                                                                                                  ;|
NO_BOW_DET                                                                                        ;|
                  LDAA    SENSOR_STBD                                                             ;|
                  ADDA    VAR_STBD                                                                ;|
                  CMPA    BASE_STBD                                                               ;|
                  BPL     PART_RIGHT_TURN                                                         ;|
                  BMI     EXIT                                                                    ;|
                                                                                                  ;|
PART_LEFT_TURN    LDY     #6000                                                                   ;|
                  jsr     del_50us                                                                ;|
                  JSR     INIT_LEFT                                                               ;|
                  MOVB    #LEFT_TURN, CRNT_STATE                                                  ;|
                  LDY     #6000                                                                   ;|
                  JSR     del_50us                                                                ;|
                  BRA     EXIT                                                                    ;|
                                                                                                  ;|
PART_RIGHT_TURN   LDY     #6000                                                                   ;|
                  jsr     del_50us                                                                ;|
                  JSR     INIT_RIGHT                                                              ;|
                  MOVB    #RIGHT_TURN, CRNT_STATE                                                 ;|
                  LDY     #6000                                                                   ;|
                  JSR     del_50us                                                                ;|
                  BRA     EXIT                                                                    ;|
                                                                                                  ;|
GO_LEFT_ALIGN     JSR     INIT_LEFT                                                               ;|
                  MOVB    #LEFT_ALIGN, CRNT_STATE                                                 ;|
                  BRA     EXIT                                                                    ;|
                                                                                                  ;|
GO_RIGHT_ALIGN    JSR     INIT_RIGHT                                                              ;|
                  MOVB    #RIGHT_ALIGN, CRNT_STATE                                                ;|
                  BRA     EXIT                                                                    ;|
                                                                                                  ;|
                                                                                                  ;|
EXIT              RTS                                                                             ;|
; ------------------------------------------------------------------------------------------------- 


; -------------------------------------------------------------------------------------------------
;  turn left if there is a path                                                                         
; -------------------------------------------------------------------------------------------------                  
FULL_LEFT         LDAA    SENSOR_BOW                                                              ;|
                  ADDA    VAR_BOW                                                                 ;|
                  CMPA    BASE_BOW                                                                ;|
                  BPL     CONTINUE_FORWARD                                                        ;|
                  BMI     EXIT  

; -------------------------------------------------------------------------------------------------
;  continue moving forward after turning                                                                         
; -------------------------------------------------------------------------------------------------                                                                                    ;|                                                                                    
CONTINUE_FORWARD  MOVB    #FWD, CRNT_STATE                                                        ;|
                  JSR     INIT_FWD                                                                ;|
                  BRA     EXIT                                                                    ;|
; -------------------------------------------------------------------------------------------------


; -------------------------------------------------------------------------------------------------
;  turn right if there is a path                                                                         
; ------------------------------------------------------------------------------------------------- 
FULL_RIGHT        LDAA    SENSOR_BOW                                                              ;|
                  ADDA    VAR_BOW                                                                 ;|
                  CMPA    BASE_BOW                                                                ;|
                  BPL     CONTINUE_FORWARD                                                        ;|
                  BMI     EXIT                                                                      ;|
; -------------------------------------------------------------------------------------------------


; -------------------------------------------------------------------------------------------------
; turns around and repositions on black line, then continues moving forward
; -------------------------------------------------------------------------------------------------
                                                                                                  ;|
REV_TURN_ST       LDAA    SENSOR_BOW                                                              ;|
                  ADDA    VAR_BOW                                                                 ;|
                  CMPA    BASE_BOW                                                                ;|
                  BMI     EXIT                                                                    ;|
                                                                                                  ;|
                  JSR     INIT_LEFT                                                               ;|
                  MOVB    #FWD, CRNT_STATE                                                        ;|
                  JSR     INIT_FWD                                                                ;|
                  BRA     EXIT                                                                    ;|
                                                                                                  ;|
; -------------------------------------------------------------------------------------------------


; -------------------------------------------------------------------------------------------------
; Stop: turns off both motors
; -------------------------------------------------------------------------------------------------                         
ALL_STOP_ST       BRSET   PORTAD0, %00000100, NO_START_BUMP                                       ;|
                  MOVB    #START, CRNT_STATE                                                      ;|
NO_START_BUMP     RTS                                                                             ;|
; -------------------------------------------------------------------------------------------------      


; Initialization Subroutines
;---------------------------------------------------------------------------
;right turn
INIT_RIGHT        BSET    PORTA,%00000010           ; Set REV dir. for right motor
                  BCLR    PORTA,%00000001           ; Set FWD dir. for left motor
                  LDAA    TOF_COUNTER               ; Mark the fwd_turn time Tfwdturn
                  ADDA    #T_RIGHT
                  STAA    T_TURN
                  RTS

;left turn                  
INIT_LEFT         BSET    PORTA,%00000001           ; Set left motor to reverse
                  BCLR    PORTA,%00000010           ; Set right motor to fwd
                  LDAA    TOF_COUNTER               ; Mark the current TOF time
                  ADDA    #T_LEFT                   ; Add left turn time to that
                  STAA    T_TURN                    ; store in T_TURN to read later on
                  RTS

;forward       
INIT_FWD          BCLR    PORTA, %00000011          ; Set FWD dir. for both motors
                  BSET    PTT, %00110000            ; Turn on the drive motors
                  RTS 
                  
;backwards  
INIT_REV          BSET PORTA,%00000011              ; Set REV direction for both motors
                  BSET PTT,%00110000                ; Turn on the drive motors
                  LDAA    TOF_COUNTER               ; Mark the current TOF time
                  ADDA    #T_LEFT                   ; Add left turn time to that
                  STAA    T_TURN                    ; store in T_TURN to read later on
                  RTS

; Turns off both motors                  
INIT_STOP         BCLR    PTT, %00110000            ; Turn off the drive motors
                  RTS


;---------------------------------------------------------------------------
;               Initialize ports

INIT              BCLR   DDRAD,$FF ; Make PORTAD an input (DDRAD @ $0272)
                  BSET   DDRA,$FF ; Make PORTA an output (DDRA @ $0002)
                  BSET   DDRB,$FF ; Make PORTB an output (DDRB @ $0003)
                  BSET   DDRJ,$C0 ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A)
                  RTS


;---------------------------------------------------------------------------
;        Initialize ADC              
openADC           MOVB   #$80,ATDCTL2 ; Turn on ADC (ATDCTL2 @ $0082)
                  LDY    #1 ; Wait for 50 us for ADC to be ready
                  JSR    del_50us ; - " -
                  MOVB   #$20,ATDCTL3 ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
                  MOVB   #$97,ATDCTL4 ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
                  RTS

;---------------------------------------------------------------------------
; Clear LCD Buffer
; This routine writes ’space’ characters (ascii 20) into the LCD display
;  buffer in order to prepare it for the building of a new display buffer.
; This needs only to be done once at the start of the program. Thereafter the
;  display routine should maintain the buffer properly.
CLR_LCD_BUF       LDX   #CLEAR_LINE
                  LDY   #TOP_LINE
                  JSR   STRCPY
              
CLB_SECOND        LDX   #CLEAR_LINE
                  LDY   #BOT_LINE
                  JSR   STRCPY
              
CLB_EXIT          RTS

; -------------------------------------------------------------------------------------------------      
;               String Copy

; Copies a null-terminated string (including the null) from one location to
; another

; Passed: X contains starting address of null-terminated string
;  Y contains first address of destination
STRCPY            PSHX            ; Protect the registers used
                  PSHY
                  PSHA
STRCPY_LOOP       LDAA 0,X        ; Get a source character
                  STAA 0,Y        ; Copy it to the destination
                  BEQ STRCPY_EXIT ; If it was the null, then exit
                  INX             ; Else increment the pointers
                  INY
                  BRA STRCPY_LOOP ; and do it again
STRCPY_EXIT       PULA            ; Restore the registers
                  PULY
                  PULX
                  RTS  
                      
; -------------------------------------------------------------------------------------------------      
;               Guider LEDs ON
; This routine enables the guider LEDs so that readings of the sensor
;  correspond to the ’illuminated’ situation.

; Passed: Nothing
; Returns: Nothing
; Side: PORTA bit 5 is changed                                                                   |
; -------------------------------------------------------------------------------------------------      
G_LEDS_ON         BSET PORTA,%00100000 ; Set bit 5                                                 |
                  RTS                                                                             ;|
; -------------------------------------------------------------------------------------------------      


; -------------------------------------------------------------------------------------------------      
;               Guider LEDs OFF

; This routine disables the guider LEDs. Readings of the sensor
;  correspond to the ’ambient lighting’ situation.

; Passed: Nothing
; Returns: Nothing
; Side: PORTA bit 5 is changed                                                                   |
; -------------------------------------------------------------------------------------------------                                                                                                       
G_LEDS_OFF        BCLR PORTA,%00100000 ; Clear bit 5                                               |
                  RTS                                                                             ;|
; -------------------------------------------------------------------------------------------------      

              
;---------------------------------------------------------------------------
;               Read Sensors
;
; This routine reads the eebot guider sensors and puts the results in RAM
;  registers.
;
; Note: Do not confuse the analog multiplexer on the Guider board with the
;  multiplexer in the HCS12. The guider board mux must be set to the
;  appropriate channel using the SELECT_SENSOR routine. The HCS12 always
;  reads the selected sensor on the HCS12 A/D channel AN1.
; 
; The A/D conversion mode used in this routine is to read the A/D channel
;  AN1 four times into HCS12 data registers ATDDR0,1,2,3. The only result
;  used in this routine is the value from AN1, read from ATDDR0. However,
;  other routines may wish to use the results in ATDDR1, 2 and 3.
; Consequently, Scan=0, Mult=0 and Channel=001 for the ATDCTL5 control word.

; Passed:   None
; Returns:  Sensor readings in:
;           SENSOR_LINE (0) (Sensor E/F)
;           SENSOR_BOW (1) (Sensor A)
;           SENSOR_PORT (2) (Sensor B)
;           SENSOR_MID (3) (Sensor C)
;           SENSOR_STBD (4) (Sensor D)
;
; Note:
; The sensor number is shown in brackets
;
; Algorithm:
;         Initialize the sensor number to 0
;         Initialize a pointer into the RAM at the start of the Sensor Array storage
; Loop    Store %10000001 to the ATDCTL5 (to select AN1 and start a conversion)
;         Repeat
;             Read ATDSTAT0
;         Until Bit SCF of ATDSTAT0 == 1 (at which time the conversion is complete)
;         Store the contents of ATDDR0L at the pointer
;         If the pointer is at the last entry in Sensor Array, then
;           Exit
;         Else
;           Increment the sensor number
;           Increment the pointer
;         Loop again.
READ_SENSORS      CLR   SENSOR_NUM ; Select sensor number 0
                  LDX   #SENSOR_LINE ; Point at the start of the sensor array
RS_MAIN_LOOP      LDAA  SENSOR_NUM ; Select the correct sensor input
                  JSR   SELECT_SENSOR ; on the hardware
                  LDY   #400 ; 20 ms delay to allow the
                  JSR   del_50us ; sensor to stabilize
                  LDAA  #%10000001 ; Start A/D conversion on AN1
                  STAA  ATDCTL5
                  BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done
                  LDAA  ATDDR0L ; A/D conversion is complete in ATDDR0L
                  STAA  0,X ; so copy it to the sensor register
                  CPX   #SENSOR_STBD ; If this is the last reading
                  BEQ   RS_EXIT ; Then exit
                  INC   SENSOR_NUM ; Else, increment the sensor number
                  INX   ; and the pointer into the sensor array
                  BRA   RS_MAIN_LOOP ; and do it again
RS_EXIT           RTS
              

;---------------------------------------------------------------------------
; Select Sensor
; This routine selects the sensor number passed in ACCA. The motor direction
; bits 0, 1, the guider sensor select bit 5 and the unused bits 6,7 in the
; same machine register PORTA are not affected.
; Bits PA2,PA3,PA4 are connected to a 74HC4051 analog mux on the guider board,
; which selects the guider sensor to be connected to AN1.
; Passed: Sensor Number in ACCA
; Returns: Nothing
; Side Effects: ACCA is changed
; Algorithm:
; First, copy the contents of PORTA into a temporary location TEMP and clear
; the sensor bits 2,3,4 in the TEMP to zeros by ANDing it with the mask
; 11100011. The zeros in the mask clear the corresponding bits in the
; TEMP. The 1’s have no effect.
; Next, move the sensor selection number left two positions to align it
; with the correct bit positions for sensor selection.
; Clear all the bits around the (shifted) sensor number by ANDing it with
; the mask 00011100. The zeros in the mask clear everything except
; the sensor number.
; Now we can combine the sensor number with the TEMP using logical OR.
; The effect is that only bits 2,3,4 are changed in the TEMP, and these
; bits now correspond to the sensor number.
; Finally, save the TEMP to the hardware.
SELECT_SENSOR     PSHA ; Save the sensor number for the moment
                  LDAA PORTA ; Clear the sensor selection bits to zeros
                  ANDA #%11100011 ;
                  STAA TEMP ; and save it into TEMP
                  PULA ; Get the sensor number
                  ASLA ; Shift the selection number left, twice
                  ASLA ;
                  ANDA #%00011100 ; Clear irrelevant bit positions
                  ORAA TEMP ; OR it into the sensor bit positions
                  STAA PORTA ; Update the hardware
                  RTS

;---------------------------------------------------------------------------
; Display Sensor Readings
; Passed: Sensor values in RAM locations SENSOR_LINE through SENSOR_STBD.
; Returns: Nothing
; Side: Everything
; This routine writes the sensor values to the LCD. It uses the ’shadow buffer’ approach.
; The display buffer is built by the display controller routine and then copied in its
; entirety to the actual LCD display. Although simpler approaches will work in this
; application, we take that approach to make the code more re-useable.
; It’s important that the display controller not write over other information on the
; LCD, so writing the LCD has to be centralized with a controller routine like this one.
; In a more complex program with additional things to display on the LCD, this routine
; would be extended to read other variables and place them on the LCD. It might even
; read some ’display select’ variable to determine what should be on the LCD.
; For the purposes of this routine, we’ll put the sensor values on the LCD
; in such a way that they (sort of) mimic the position of the sensors, so
; the display looks like this:
; 01234567890123456789
; ___FF_______________
; PP_MM_SS_LL_________
; Where FF is the front sensor, PP is port, MM is mid, SS is starboard and
; LL is the line sensor.
; The corresponding addresses in the LCD buffer are defined in the following
; equates (In all cases, the display position is the MSDigit).
DP_FRONT_SENSOR   EQU TOP_LINE+3
DP_PORT_SENSOR    EQU BOT_LINE+0
DP_MID_SENSOR     EQU BOT_LINE+3
DP_STBD_SENSOR    EQU BOT_LINE+6
DP_LINE_SENSOR    EQU BOT_LINE+9
              
DISPLAY_SENSORS   LDAA  SENSOR_BOW        ; Get the FRONT sensor value
                  JSR   BIN2ASC           ; Convert to ascii string in D
                  LDX   #DP_FRONT_SENSOR  ; Point to the LCD buffer position
                  STD   0,X               ; and write the 2 ascii digits there
                  
                  LDAA  SENSOR_PORT       ; Repeat for the PORT value
                  JSR   BIN2ASC
                  LDX   #DP_PORT_SENSOR
                  STD   0,X
                  
                  LDAA  SENSOR_MID        ; Repeat for the MID value
                  JSR   BIN2ASC
                  LDX   #DP_MID_SENSOR
                  STD   0,X
                  
                  LDAA  SENSOR_STBD       ; Repeat for the STARBOARD value
                  JSR   BIN2ASC
                  LDX   #DP_STBD_SENSOR
                  STD   0,X
                  
                  LDAA  SENSOR_LINE       ; Repeat for the LINE value
                  JSR   BIN2ASC
                  LDX   #DP_LINE_SENSOR
                  STD   0,X
                  
                  LDAA  #CLEAR_HOME       ; Clear the display and home the cursor
                  JSR   cmd2LCD           ; "
                  
                  LDY   #40               ; Wait 2 ms until "clear display" command is complete
                  JSR   del_50us
                  
                  LDX   #TOP_LINE         ; Now copy the buffer top line to the LCD
                  JSR   putsLCD
                  
                  LDAA  #LCD_SEC_LINE     ; Position the LCD cursor on the second line
                  JSR   LCD_POS_CRSR
                  
                  LDX   #BOT_LINE         ; Copy the buffer bottom line to the LCD
                  JSR   putsLCD
                  RTS

; utility subroutines
; -------------------------------------------------------------------------------------------------      
initLCD:          LDY     #2000
                  JSR     del_50us
                  LDAA    #$28
                  JSR     cmd2LCD
                  LDAA    #$0C
                  JSR     cmd2LCD
                  LDAA    #$06
                  JSR     cmd2LCD
                  RTS
                  
clrLCD:           LDAA  #$01
                  JSR   cmd2LCD
                  LDY   #40
                  JSR   del_50us
                  RTS
                  
del_50us          PSHX ; (2 E-clk) Protect the X register
eloop             LDX   #300 ; (2 E-clk) Initialize the inner loop counter
iloop             NOP ; (1 E-clk) No operation
                  DBNE X,iloop ; (3 E-clk) If the inner cntr not 0, loop again
                  DBNE Y,eloop ; (3 E-clk) If the outer cntr not 0, loop again
                  PULX ; (3 E-clk) Restore the X register
                  RTS ; (5 E-clk) Else return
                  
cmd2LCD:          BCLR  LCD_CNTR, LCD_RS ; select the LCD instruction
                  JSR   dataMov ; send data to IR
                  RTS
                  
putsLCD:          LDAA  1,X+     ; get one character from  string
                  BEQ   donePS    ; get NULL character
                  JSR   putcLCD
                  BRA   putsLCD
donePS            RTS

putcLCD:          BSET  LCD_CNTR, LCD_RS  ; select the LCD data register (DR)c
                  JSR   dataMov            ; send data to DR
                  RTS
                  
dataMov:          BSET  LCD_CNTR, LCD_E ; pull LCD E-signal high
                  STAA  LCD_DAT         ; send the upper 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E ; pull the LCD E-signal low to complete write oper.
               
                  LSLA                  ; match the lower 4 bits with LCD data pins
                  LSLA                  ; ""
                  LSLA                  ; ""
                  LSLA                  ; ""
               
                  BSET  LCD_CNTR, LCD_E ; pull LCD E-signal high
                  STAA  LCD_DAT         ; send the lower 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E ; pull the LCD E-signal low to complete write oper.
               
                  LDY   #1              ; adding this delay allows
                  JSR   del_50us        ; completion of most instructions
                  RTS

initAD            MOVB  #$C0,ATDCTL2 ;power up AD, select fast flag clear
                  JSR   del_50us ;wait for 50 us
                  MOVB  #$00,ATDCTL3 ;8 conversions in a sequence
                  MOVB  #$85,ATDCTL4 ;res=8, conv-clks=2, prescal=12
                  BSET  ATDDIEN,$0C ;configure pins AN03,AN02 as digital inputs
                  RTS

;*******************************************************************           
;* Integer to BCD Conversion Routine
;* This routine converts a 16 bit binary number in .D into
;* BCD digits in BCD_BUFFER.
;* Peter Hiscocks
;* Algorithm:
;* Because the IDIV (Integer Division) instruction is available on
;* the HCS12, we can determine the decimal digits by repeatedly
;* dividing the binary number by ten: the remainder each time is
;* a decimal digit. Conceptually, what we are doing is shifting
;* the decimal number one place to the right past the decimal
;* point with each divide operation. The remainder must be
;* a decimal digit between 0 and 9, because we divided by 10.
;* The algorithm terminates when the quotient has become zero.
;* Bug note: XGDX does not set any condition codes, so test for
;* quotient zero must be done explicitly with CPX.
;* Data structure:
;* BCD_BUFFER EQU * The following registers are the BCD buffer area
;* TEN_THOUS RMB 1 10,000 digit, max size for 16 bit binary
;* THOUSANDS RMB 1 1,000 digit
;* HUNDREDS RMB 1 100 digit
;* TENS RMB 1 10 digit
;* UNITS RMB 1 1 digit
;* BCD_SPARE RMB 2 Extra space for decimal point and string terminator                  
;***************************************************************************************************
int2BCD           XGDX              ;Save the binary number into .X
                  LDAA #0           ;Clear the BCD_BUFFER
                  STAA TEN_THOUS
                  STAA THOUSANDS
                  STAA HUNDREDS
                  STAA TENS
                  STAA UNITS
                  STAA BCD_SPARE
                  STAA BCD_SPARE+1
                  
                  CPX #0            ; Check for a zero input
                  BEQ CON_EXIT      ; and if so, exit
                  
                  XGDX              ; Not zero, get the binary number back to .D as dividend
                  LDX #10           ; Setup 10 (Decimal!) as the divisor
                  IDIV              ; Divide Quotient is now in .X, remainder in .D
                  STAB UNITS        ; Store remainder
                  CPX #0            ; If quotient is zero,
                  BEQ CON_EXIT      ; then exit
                  
                  XGDX              ; else swap first quotient back into .D
                  LDX #10           ; and setup for another divide by 10
                  IDIV
                  STAB TENS
                  CPX #0
                  BEQ CON_EXIT
                  
                  XGDX              ; Swap quotient back into .D
                  LDX #10           ; and setup for another divide by 10
                  
                  IDIV
                  STAB HUNDREDS
                  CPX #0
                  BEQ CON_EXIT
                  
                  XGDX              ; Swap quotient back into .D
                  LDX #10           ; and setup for another divide by 10
                  IDIV
                  STAB THOUSANDS
                  CPX #0
                  BEQ CON_EXIT
                  
                  XGDX              ; Swap quotient back into .D
                  LDX #10           ; and setup for another divide by 10
                  IDIV
                  STAB TEN_THOUS
            
CON_EXIT          RTS      ; We�re done the conversion

; Cursor Positioning Subroutine
LCD_POS_CRSR      ORAA #%10000000 ; Set the high bit of the control word
                  JSR cmd2LCD ; and set the cursor address
                  RTS
                  
;***************************************************************************************************
BIN2ASC                PSHA                    ; Save a copy of the input number
                      TAB            
                      ANDB #%00001111        ; Strip off the upper nibble
                      CLRA                    ; D now contains 000n where n is the LSnibble
                      ADDD #HEX_TABLE        ; Set up for indexed load
                      XGDX                
                      LDAA 0,X              ; Get the LSnibble character

                      PULB                    ; Retrieve the input number into ACCB
                      PSHA                    ; and push the LSnibble character in its place
                      RORB                    ; Move the upper nibble of the input number
                      RORB                    ;  into the lower nibble position.
                      RORB
                      RORB 
                      ANDB #%00001111        ; Strip off the upper nibble
                      CLRA              ; D now contains 000n where n is the MSnibble 
                  ADDD #HEX_TABLE   ; Set up for indexed load
                  XGDX                                                               
                  LDAA 0,X          ; Get the MSnibble character into ACCA
                      PULB                    ; Retrieve the LSnibble character into ACCB

                      RTS
;***************************************************************************************************
;* BCD to ASCII Conversion Routine
;* This routine converts the BCD number in the BCD_BUFFER
;* into ascii format, with leading zero suppression.
;* Leading zeros are converted into space characters.
;* The flag �NO_BLANK� starts cleared and is set once a non-zero
;* digit has been detected.
;* The �units� digit is never blanked, even if it and all the
;* preceding digits are zero.
BCD2ASC           LDAA    #0            ; Initialize the blanking flag
                  STAA    NO_BLANK
              
C_TTHOU           LDAA    TEN_THOUS     ; Check the �ten_thousands� digit
                  ORAA    NO_BLANK
                  BNE     NOT_BLANK1

ISBLANK1          LDAA    #' '          ; It�s blank
                  STAA    TEN_THOUS     ; so store a space
                  BRA     C_THOU        ; and check the �thousands� digit
              
NOT_BLANK1        LDAA    TEN_THOUS     ; Get the �ten_thousands� digit
                  ORAA    #$30          ; Convert to ascii
                  STAA    TEN_THOUS
                  LDAA    #$1           ; Signal that we have seen a �non-blank� digit
                  STAA    NO_BLANK

C_THOU            LDAA    THOUSANDS     ; Check the thousands digit for blankness
                  ORAA    NO_BLANK      ; If it�s blank and �no-blank� is still zero
                  BNE     NOT_BLANK2

ISBLANK2          LDAA    #' '          ; Thousands digit is blank
                  STAA    THOUSANDS     ; so store a space
                  BRA     C_HUNS        ; and check the hundreds digit

NOT_BLANK2        LDAA    THOUSANDS     ; (similar to �ten_thousands� case)
                  ORAA    #$30
                  STAA    THOUSANDS
                  LDAA    #$1
               
                  STAA    NO_BLANK
              
C_HUNS            LDAA    HUNDREDS      ; Check the hundreds digit for blankness
                  ORAA    NO_BLANK      ; If it�s blank and �no-blank� is still zero
                  BNE     NOT_BLANK3

ISBLANK3          LDAA    #' '          ; Hundreds digit is blank
                  STAA    HUNDREDS       ; so store a space
                  BRA     C_TENS          ; and check the tens digit

NOT_BLANK3        LDAA    HUNDREDS          ; (similar to �ten_thousands� case)
                  ORAA    #$30
                  STAA    HUNDREDS
                  LDAA    #$1
                  STAA    NO_BLANK

C_TENS            LDAA    TENS          ; Check the tens digit for blankness
                  ORAA    NO_BLANK      ; If it�s blank and �no-blank� is still zero
                  BNE     NOT_BLANK4

ISBLANK4          LDAA    #' '          ; Tens digit is blank
                  STAA    TENS          ; so store a space
                  BRA     C_UNITS       ; and check the units digit

NOT_BLANK4        LDAA    TENS          ; (similar to �ten_thousands� case)
                  ORAA    #$30
                  STAA    TENS

C_UNITS           LDAA    UNITS         ; No blank check necessary, convert to ascii.
                  ORAA    #$30
                  STAA    UNITS

                  RTS                 ; We�re done

;***************************************************************************************************
ENABLE_TOF        LDAA    #%10000000
                  STAA    TSCR1 ; Enable TCNT
                  STAA    TFLG2 ; Clear TOF
                  LDAA    #%10000100 ; Enable TOI and select prescale factor equal to 16
                  STAA    TSCR2
                  RTS
;***************************************************************************************************
TOF_ISR           INC     TOF_COUNTER
                  LDAA    #%10000000 ; Clear
                  STAA    TFLG2 ; TOF
                  RTI
;***************************************************************************************************
;*                      Update Display (Battery Voltage + Current State)                           *
;***************************************************************************************************
UPDT_DISPL        MOVB    #$90,ATDCTL5 ; R-just., uns., sing. conv., mult., ch=0, start
                  BRCLR   ATDSTAT0,$80,* ; Wait until the conver. seq. is complete
                  
                ; Display the current state
                ;----------------------------
                  LDAA    #$C9            ; Move LCD cursor to the 2nd row, end of msg2
                  JSR     cmd2LCD         ;
                  LDAB    CRNT_STATE      ; Display current state
                  LSLB                    ; "
                  LSLB                    ; "
                  LSLB
                  LDX     #tab            ; "
                  ABX                     ; "
                  JSR     putsLCD         ; "
                  RTS
                  

;***************************************************************************************************
;*                                Interrupt Vectors                                                *
;***************************************************************************************************
                  ORG     $FFFE
                  DC.W    Entry ; Reset Vector
                  ORG     $FFDE
                  DC.W    TOF_ISR ; Timer Overflow Interrupt Vector