; '2600 for Newbies
; Session 13 - Playfield
        PROCESSOR 6502

        INCLUDE "vcs.h"
        INCLUDE "macro.h"
        
;; ------------------------------------------------------------
;; ---
;; --- CONSTANTS
;; ---
;; ------------------------------------------------------------
FALSE = 0
TRUE = 1

PIC_LINES = 192
PIC_WIDTH = 160

DIR_LEFT = 0        
DIR_RIGHT = 1
        
INFLAGS_XDIR     = %00000001    ;clear=left, set=right
INFLAGS_XPRESSED = %00000010
INFLAGS_YDIR     = %00000100    ;clear=up, set=down
INFLAGS_YPRESSED = %00001000        
INFLAGS_FIRE     = %00010000




MSTATE_Y_GROUND = 0
MSTATE_Y_JUMP = 1
MSTATE_Y_HIGH_JUMP = 2
MSTATE_Y_WALL_JUMP = 3
MSTATE_Y_FALL = 4
MSTATE_Y_GRAB = 5

MSTATE_X_STILL = 0        
MSTATE_X_WALK = 1 
MSTATE_X_RUN = 2

MSTATE_T_NONE = 0
MSTATE_T_RELEASE = 1        
MSTATE_T_LAND = 2    
MSTATE_T_GRAB = 3

MSTATE_D_LEFT = 0        
MSTATE_D_RIGHT = 1

LAND_TIMEOUT = 10
RELEASE_TIMEOUT = 10
GRAB_TIMEOUT = 60

MAX_XACCELERATE = $0004
GRAVITY = -$0015
VJUMP_YVEL_INIT = $0180
VJUMP_HIGH_YVEL_INIT = $0280
        
HJUMP_XVEL_INIT = $0090
HJUMP_YVEL_INIT = $0140
        
PLAYER_XPOS_INIT = 140
PLAYER_YPOS_INIT = $2000
PLAYER_WIDTH = 8

PF_WIDTH = 4        

COLLISION_WALL_LEFT  = %00000001
COLLISION_WALL_RIGHT = %00000010
COLLISION_BELOW      = %00000100
COLLISION_ABOVE      = %00001000

COLLISION_WALL = COLLISION_WALL_LEFT | COLLISION_WALL_RIGHT

;; ------------------------------------------------------------
;; ---
;; --- VARIABLES
;; ---
;; ------------------------------------------------------------
        SEG.U variables
        ORG $80

PlayerXvel ds 2        
PlayerXpos ds 2        
PlayerYvel ds 2
PlayerYpos ds 2
MState_X ds 1
MState_Y ds 1
MState_T ds 1
MState_D ds 1
MState_timeout ds 1
Collision ds 1
Tmp1 ds 1
Tmp2 ds 1        
Tmp3 ds 1        
Tmp4 ds 1        
Tmp5 ds 1        

;; ------------------------------------------------------------
;; ---
;; --- MACROS
;; ---
;; ------------------------------------------------------------

        ;; --------------------------------------------------
        ;; macro JUMP_TABLE
        ;;
        ;; usage: JUMP_TABLE addr_table
        ;;
        ;; Table index must be in A
        ;;
        ;; Contents of X are overwritten
        ;; --------------------------------------------------
        MAC JUMP_TABLE
            asl
            tax
            lda {1}+1,x
            pha
            lda {1},x
            pha
            rts
        ENDM

        ;; --------------------------------------------------
        ;; macro NEGATE_A
        ;; --------------------------------------------------
        MAC NEGATE_A
            eor #$ff
	        sec
	        adc #0
        ENDM
        
        ;;------------------------------------------------------------------------------
	    ;; macro NEGATE16
        ;;
        ;; usage: NEGATE16 addr
	    ;; 
	    ;; Adapted from http://www.obelisk.demon.co.uk/6502/algorithms.html
	    ;;------------------------------------------------------------------------------
	    MAC NEGATE16
            sec               ;Ensure carry is set
            lda #0            ;Subtract the two least significant bytes
            sbc {1}+0
            sta {1}+0         ;... and store the result
            lda #0            ;Subtract the two most significant bytes
            sbc {1}+1         ;... and any propagated borrow bit
            sta {1}+1         ;... and store the result
        ENDM
        
        ;;------------------------------------------------------------------------------
	    ;; macro SUB16
        ;;
        ;; usage: SUB16 addr_a, addr_b, addr_c
        ;;
        ;; result: (addr_a) = (addr_b) - (addr_c)
	    ;; 
	    ;; Adapted from http://www.obelisk.demon.co.uk/6502/algorithms.html
	    ;;------------------------------------------------------------------------------
	    MAC SUB16
            sec            ;Ensure carry is set
            lda {2}+0      ;Subtract the two least significant bytes
            sbc {3}+0
            sta {1}+0      ;... and store the result
            lda {2}+1      ;Subtract the two most significant bytes
            sbc {3}+1      ;... and any propagated borrow bit
            sta {1}+1      ;... and store the result
        ENDM

        ;;------------------------------------------------------------------------------
	    ;; macro ADD16
        ;;
        ;; usage: ADD16 addr_a, addr_b, addr_c
        ;;
        ;; result: (addr_a) = (addr_b) + (addr_c)
	    ;; 
	    ;; Adapted from http://www.obelisk.demon.co.uk/6502/algorithms.html
	    ;;------------------------------------------------------------------------------
	    MAC ADD16
	        clc             ;Ensure carry is clear
	        lda {2}+0       ;Add the two least significant bytes
	        adc {3}+0
	        sta {1}+0       ;... and store the result
	        lda {2}+1       ;Add the two most significant bytes
	        adc {3}+1       ;... and any propagated carry bit
	        sta {1}+1       ;... and store the result
        ENDM
        
        ;;------------------------------------------------------------------------------
	    ;; macro LDA16_MSB
        ;;
        ;; usage: LDA16_MSB addr
        ;;
        ;; Load msb of 16-bit number at addr into A
        ;;------------------------------------------------------------------------------
        MAC LDA16_MSB
            lda {1}+1
        ENDM

        ;;------------------------------------------------------------------------------
	    ;; macro STA16_MSB
        ;;
        ;; usage: STA16_MSB addr
        ;;
        ;; Treats A value as if it's the MSB of a 16-bit value, and stores it to
        ;; the argument address, as well as a 0 to the LSB location.
        ;;
        ;; Results in A = 0
	    ;;------------------------------------------------------------------------------
        MAC STA16_MSB
            sta {1}+1           ;msb
            lda #0
            sta {1}+0           ;lsb
        ENDM

        ;;------------------------------------------------------------------------------
	    ;; macro STI16
        ;;
        ;; usage: STI16 imm, addr
        ;;
        ;; Store 16-bit immediate value to memory.
	    ;;------------------------------------------------------------------------------
        MAC STI16
            lda #>{1}
            sta {2}+1
            lda #<{1}
            sta {2}+0
        ENDM

        ;;------------------------------------------------------------------------------
	    ;; macro STI
        ;;
        ;; usage: STI imm, addr
        ;;
        ;; Put imm into addr via A
	    ;;------------------------------------------------------------------------------
        MAC STI
            lda {1}
            sta {2}
        ENDM

        ;;------------------------------------------------------------------------------
	    ;; macro BITI
        ;;
        ;; usage: BIT mask, addr
	    ;;------------------------------------------------------------------------------
        MAC BITI
            lda {1}
            bit {2}
        ENDM

        ;;------------------------------------------------------------------------------
	    ;; macro CMPI
        ;;
        ;; usage: CMPI imm, addr
	    ;;------------------------------------------------------------------------------
        MAC CMPI
            lda {1}
            cmp {2}
        ENDM
        
        ;;------------------------------------------------------------------------------
	    ;; macro STI_Y
        ;;
        ;; usage: STI imm, addr
        ;;
        ;; Put imm into addr via Y
	    ;;------------------------------------------------------------------------------
        MAC STI_Y
            ldy {1}
            sty {2}
        ENDM

        ;;------------------------------------------------------------------------------
	    ;; macro CMPI16
        ;;
	    ;; usage: CMPI16 addr, imm
        ;;
        ;; sets flags for (addr) - imm
	    ;;------------------------------------------------------------------------------
        MAC CMPI16
            sec            ;Ensure carry is set
            lda {1}+0      ;Subtract the two least significant bytes
            sbc #<{2}
            lda {1}+1      ;Subtract the two most significant bytes
            sbc #>{2}      ;... and any propagated borrow bit
        ENDM
        
        ;;------------------------------------------------------------------------------
	    ;; macro CLAMP16
        ;;
	    ;; usage: CLAMP16 addr_x, imm_constraint
        ;;
        ;; result placed in addr_x
	    ;;------------------------------------------------------------------------------
        MAC CLAMP16
            CMPI16 {1}, {2}
            bpl .clamp_greater
            CMPI16 {1}, -{2}
            bmi .clamp_lessthan
            jmp .clamp_end
.clamp_greater
            STI16 {2}, {1}
            jmp .clamp_end
.clamp_lessthan
            STI16 #[-{2}], {1}
            sta {1}+0
.clamp_end            
        ENDM
        
        ;; --------------------------------------------------
        ;; macro WAIT_SYNC
        ;; 
        ;; Wait till beginning of next scanline
        ;; --------------------------------------------------
        MAC WAIT_SYNC
            sta WSYNC
        ENDM
        
        ;; --------------------------------------------------
        ;; macro WAIT_VSYNC
        ;; 
        ;; Ends VBLANK and waits for end of 3 VSYNC lines
        ;; --------------------------------------------------
        MAC WAIT_VSYNC
_WaitVsync        
            lda #0
	        sta VBLANK
	        lda #2
	        sta VSYNC
        
	        WAIT_SYNC
	        WAIT_SYNC
	        lda #0              ;preload A to minimize cycles used on next scanline
	        WAIT_SYNC

	        sta VSYNC           
        ENDM

        ;; --------------------------------------------------
        ;; macro START_VBLANK_TIMER
        ;; 
        ;; Start the 64-cycle timer to detect start of picture
        ;; --------------------------------------------------
        MAC START_VBLANK_TIMER
            lda #44
            sta TIM64T
        ENDM

        ;; --------------------------------------------------
        ;; macro WAIT_VBLANK_TIMER
        ;; 
        ;; Wait for timer to expire. When complete, on cycle
        ;; 0 of first picture scanline.
        ;; --------------------------------------------------
        MAC WAIT_VBLANK_TIMER
waitVBlankTimer
            ldx INTIM
            bne waitVBlankTimer
            WAIT_SYNC
        ENDM
        
        ;; --------------------------------------------------
        ;; macro INIT_VARIABLES
        ;; 
        ;; Set initial variable values
        ;; --------------------------------------------------
        MAC INIT_VARIABLES
_InitVariables
            lda #PLAYER_XPOS_INIT
            STA16_MSB PlayerXpos
            STI16 #PLAYER_YPOS_INIT, PlayerYpos
        ENDM

        ;; --------------------------------------------------
        ;; macro INIT_RENDER_STATE
        ;; 
        ;; Init render state at beginning of execution
        ;; --------------------------------------------------
        MAC INIT_RENDER_STATE
_InitRenderState
        ENDM
        
        ;; --------------------------------------------------
        ;; macro RENDER_FRAME
        ;; 
        ;; Render the 192 picture scanlines.
        ;;
        ;; Begins at scanline 37, cpu cycle 0
        ;; --------------------------------------------------
        MAC RENDER_FRAME
TMP_SCANLINE = Tmp1        
_RenderFrame
        lda #$f0
        sta COLUPF
        lda #$d4
        sta COLUP0
        lda #191
        sta TMP_SCANLINE
        WAIT_SYNC
        
renderLoop
        LDA16_MSB PlayerYpos
        sec
        sbc TMP_SCANLINE
        bmi noPlayer
        and #$f0
        bne noPlayer
        lda #$ff
        jmp donePlayer
noPlayer
        lda #0
donePlayer
        sta GRP0

        lda #0
        sta PF0
        sta PF2
        SLEEP 16
        lda #%10000000
        sta PF2
        lda #%00010000
        sta PF0
        
        dec TMP_SCANLINE        
        WAIT_SYNC
        bne renderLoop
        
        ENDM

        ;; --------------------------------------------------
        ;; macro CLEAR_RENDER_STATE
        ;; 
        ;; enter vblank/overscan render state.
        ;; --------------------------------------------------
        MAC CLEAR_RENDER_STATE
_ClearRenderState
            lda #0
	        ;; clear background
	        sta COLUBK
	        sta COLUPF
            ;; clear playfield
            sta PF0
            sta PF1
            sta PF2

            sta GRP0
        ENDM
        
        ;; --------------------------------------------------
        ;; macro END_FRAME
        ;; 
        ;; Start VBLANK, clear render state, wait for last
        ;; scanline.
        ;; --------------------------------------------------
        MAC END_FRAME
_EndFrame
            lda #%01000010
	        sta VBLANK          ; end of screen - enter blanking

	        lda #35                 ; Number of 64-cycle clock ticks to wait.
	        WAIT_SYNC               ; Wait for end of first overscan line.
	        sta TIM64T              ; Start timer.

	        CLEAR_RENDER_STATE

.WaitTimer
            ldx INTIM
            bne .WaitTimer

            ;; Timer is now expired. Wait for end of last overscan line.
            WAIT_SYNC        
        ENDM

;; ------------------------------------------------------------
;; ---
;; --- CODE
;; ---
;; ------------------------------------------------------------
        SEG
        ORG $F000

Reset        
        CLEAN_START
        INIT_VARIABLES
        INIT_RENDER_STATE

StartOfFrame
   ; Start of new frame
   ; Start of vertical blank processing

        WAIT_VSYNC
        START_VBLANK_TIMER
        jsr ProcessJoystick
        jsr ProcessMState
        jsr ProcessPhysics
        jsr DoFrame
        jmp StartOfFrame
        
DoFrame SUBROUTINE
        WAIT_VBLANK_TIMER

        RENDER_FRAME
        END_FRAME
        rts

;; --------------------------------------------------
;; subroutine ProcessJoystick
;;
;; Uses Tmp1. X and Y untouched.
;; 
;; Results placed in A. See INFLAGS_
;; --------------------------------------------------
ProcessJoystick SUBROUTINE
        ;; Check joystick left
        lda #%01000000
	    bit SWCHA
        bne no_left
_left
        lda #[DIR_LEFT | INFLAGS_XPRESSED]
        jmp xjoy_done
no_left
        ;; Check joystick right
        lda #%10000000
	    bit SWCHA
        bne no_right
_right
        lda #[DIR_RIGHT | INFLAGS_XPRESSED]
        jmp xjoy_done
no_right
        ;; Neither left nor right is pressed
        lda #0
        ;; fall through to xjoy_done
xjoy_done
        sta Tmp1
check_fire
        ;; Check if fire pressed
        lda INPT4
        bmi no_fire
_fire
        lda Tmp1
        ora #INFLAGS_FIRE
        jmp fire_done
no_fire
        lda Tmp1
fire_done
        rts

;; --------------------------------------------------
;; subroutine ProcessMState
;;
;; Takes INFLAGS parameter in A
;; --------------------------------------------------
ProcessMState SUBROUTINE
TMP_INFLAGS = Tmp1
        ;; Save A parm in Tmp
        sta TMP_INFLAGS

        ;; -----------------------
        ;; --- Handle timeout
        ;; -----------------------
_check_timeout
        ;; Check if any MState flags have expired.
        ldx MState_timeout
        beq timeout_done
        dex
        stx MState_timeout
        bne timeout_done
        ;; Timeout just expired.
        STI #MSTATE_T_NONE, MState_T
timeout_done

        ;; -----------------------
        ;; --- Handle Y
        ;; -----------------------
_mstate_y
        lda MState_Y
        JUMP_TABLE JmpTable_MState_Y
JmpTable_MState_Y
        DC.W CASE_ground-1
        DC.W CASE_jump-1
        DC.W CASE_jump-1
        DC.W CASE_jump-1
        DC.W CASE_fall-1
        DC.W CASE_grab-1
;; case MSTATE_Y_GROUND:
CASE_ground
        BITI #INFLAGS_FIRE, TMP_INFLAGS
        beq y_done
        CMPI #MSTATE_T_LAND, MState_T
        beq jump_high
_jump
        STI #MSTATE_Y_JUMP, MState_Y
        jmp y_done
jump_high        
        STI #MSTATE_Y_HIGH_JUMP, MState_Y
        jmp y_done
;; case MSTATE_Y_JUMP:
;; case MSTATE_Y_HIGH_JUMP:
;; case MSTATE_Y_WALL_JUMP:
CASE_jump
        STI #MSTATE_Y_FALL, MState_Y
        jmp y_done
;; case MSTATE_FALL:
CASE_fall
        BITI #COLLISION_BELOW, Collision
        bne hit_ground
        BITI #[COLLISION_WALL_LEFT | COLLISION_WALL_RIGHT], Collision
        bne hit_wall
_cont_fall
        jmp y_done
hit_ground
        STI #MSTATE_T_LAND, MState_T
        STI #MSTATE_Y_GROUND, MState_Y
        STI #LAND_TIMEOUT, MState_timeout
        jmp y_done
hit_wall
        STI #MSTATE_T_GRAB, MState_T
        STI #MSTATE_Y_GRAB, MState_Y
        STI #GRAB_TIMEOUT, MState_timeout
        jmp y_done
;; case MSTATE_Y_GRAB
CASE_grab
        CMPI #MSTATE_T_GRAB, MState_T
        bne end_grab
        BITI #INFLAGS_FIRE, TMP_INFLAGS
        bne wall_jump
        jmp y_done
end_grab        
        STI #MSTATE_Y_FALL, MState_Y
        jmp y_done
wall_jump
        lda MState_D
        eor #1
        sta MState_D
        STI #MSTATE_Y_WALL_JUMP, MState_Y
        jmp x_skip
y_done        
        ;; -----------------------
        ;; --- Handle X
        ;; -----------------------
        CMPI #MSTATE_Y_GROUND, MState_Y
        bne x_skip
        
        lda MState_X
        JUMP_TABLE JmpTable_MState_X
JmpTable_MState_X
        DC.W CASE_still-1
        DC.W CASE_move-1
        DC.W CASE_move-1
;; case MSTATE_X_STILL:
CASE_still
        BITI #INFLAGS_XPRESSED, TMP_INFLAGS
        beq x_done
        CMPI #MSTATE_T_RELEASE, MState_T
        bne walk
_check_run_xdir
        lda TMP_INFLAGS
        and #INFLAGS_XDIR
        eor MState_D
        bne walk
_run
        STI #MSTATE_X_RUN, MState_X
        STI #MSTATE_T_NONE, MState_T
        jmp x_done
walk
        STI #MSTATE_X_WALK, MState_X
        STI #MSTATE_T_NONE, MState_T
        jmp x_done
;; case MSTATE_X_WALK:
;; case MSTATE_X_RUN:
CASE_move
        BITI #INFLAGS_XPRESSED, TMP_INFLAGS
        bne x_done
        STI #RELEASE_TIMEOUT, MState_timeout
        STI #MSTATE_X_STILL, MState_X
        STI #MSTATE_T_RELEASE, MState_T
        jmp x_done
x_done
        BITI #INFLAGS_XPRESSED, TMP_INFLAGS
        beq x_skip
        lda TMP_INFLAGS
        and #1
        sta MState_D
x_skip      
        rts

;; --------------------------------------------------
;; subroutine ProcessPhysics
;; --------------------------------------------------
ProcessPhysics SUBROUTINE
        ;; -----------------------
        ;; --- Set X/Y Velocity
        ;; -----------------------
        lda MState_Y
        JUMP_TABLE JmpTable_Vel
JmpTable_Vel
        DC.W .CASE_ground-1
        DC.W .CASE_jump-1
        DC.W .CASE_high_jump-1
        DC.W .CASE_wall_jump-1
        DC.W .CASE_fall-1
        DC.W .CASE_grab-1

;; case MSTATE_Y_GROUND:
.CASE_ground
        STI16 #0, PlayerYvel
_load_speed        
        lda MState_X
        asl
        tax
        lda Xspeed+1,x
        sta Tmp1+1
        lda Xspeed,x
        sta Tmp1
_set_direction
        ldx MState_D
        beq .negate_xspeed
        jmp .acceleration
.negate_xspeed
        NEGATE16 Tmp1
.acceleration        
        ;; acceleration = desired - PlayerXvel
        SUB16 Tmp1, Tmp1, PlayerXvel
.clamp        
        ;; Constrain to max acceleration
        CLAMP16 Tmp1, #MAX_XACCELERATE
.newvel
        ;; new velocity = current + acceleration
        ADD16 PlayerXvel, PlayerXvel, Tmp1
        jmp vel_done
;; case MSTATE_Y_JUMP
.CASE_jump
        STI16 #VJUMP_YVEL_INIT, PlayerYvel
        jmp vel_done
;; case MSTATE_Y_HIGH_JUMP        
.CASE_high_jump
        STI16 #VJUMP_HIGH_YVEL_INIT, PlayerYvel
        jmp vel_done
;; case MSTATE_Y_WALL_JUMP
.CASE_wall_jump
        STI16 #HJUMP_YVEL_INIT, PlayerYvel
        lda MState_D
        beq .negate_x
        STI16 #HJUMP_XVEL_INIT, PlayerXvel
        jmp vel_done
.negate_x
        STI16 #[-HJUMP_XVEL_INIT], PlayerXvel
        jmp vel_done
;; case MSTATE_Y_FALL:
.CASE_fall
        STI16 #GRAVITY, Tmp1
        ADD16 PlayerYvel, Tmp1, PlayerYvel
        jmp vel_done
;; case MSTATE_Y_GRAB
.CASE_grab
        STI16 #0, PlayerYvel
        jmp vel_done
vel_done
        ADD16 PlayerXpos, PlayerXvel, PlayerXpos
        ADD16 PlayerYpos, PlayerYvel, PlayerYpos
        ;; ---
        ;; --- X Collision
        ;; ---
_check_xcollision
        LDA16_MSB PlayerXpos
        cmp #[PF_WIDTH]
        bcc wall_left
        clc
        adc PLAYER_WIDTH
        cmp #[PIC_WIDTH - PF_WIDTH + 1]
        bcs wall_right
        lda #[~COLLISION_WALL]
        and Collision
        sta Collision
        jmp .setpos
wall_left
        STI16 #[PF_WIDTH << 8], PlayerXpos
        lda #COLLISION_WALL_LEFT
        jmp wall_collision
wall_right
        STI16 #[(PIC_WIDTH - PF_WIDTH - PLAYER_WIDTH) << 8], PlayerXpos
        lda #COLLISION_WALL_RIGHT
        jmp wall_collision
wall_collision
        ora Collision
        sta Collision
        STI16 #0, PlayerXvel
        jmp .setpos
.setpos
        ldx #0                  ;specify P0
        lda PlayerXpos+1        ;use MSB of xpos
        jsr PosObject

        ;; ---
        ;; --- Y Collision
        ;; ---
_check_ycollision
        ADD16 PlayerYpos, PlayerYvel, PlayerYpos

        lda PlayerYpos+1
        cmp #>PLAYER_YPOS_INIT
        bmi floor_collision
        lda #[~COLLISION_BELOW]
        and Collision
        sta Collision
        jmp ydone
floor_collision
        lda #COLLISION_BELOW
        ora Collision
        sta Collision
        STI16 #0, PlayerYvel
        STI16 #PLAYER_YPOS_INIT, PlayerYpos
ydone        
        WAIT_SYNC
        sta HMOVE
        
        rts
        
Xspeed
        DC.W $0000               ;still
        DC.W $0040               ;walk
        DC.W $0080               ;run

;------------------------------------------------------------------------------
; subroutine PosObject
; 
; Positions an object horizontally
;
; Inputs: A = Desired position. Must be in [0, 159].
;         X = Desired object to be positioned. Must be in [0, 4].
;
; scanlines: If control comes on or before cycle 73 then 1 scanline is consumed,
;            otherwise 2 scanlines.
;
; Outputs: X = unchanged
;          A = Fine Adjustment value.
;          Y = the "remainder" of the division by 15 minus an additional 15.
;
;          Control is returned on cycle 6 of the next scanline.
;
        ORG $FE00               ;Place this at a page boundary in order to guarantee that none
                                ;of the branches cross a boundary. Timing is critical to what
                                ;this does. Jumping across a page adds 1 cycle, breaking this
                                ;function.
PosObject SUBROUTINE
        WAIT_SYNC   ; Sync to start of scanline.
	    sec         ; Set the carry flag so no borrow will be applied during the division.
.divideby15
	    sbc #15     ; Waste the necessary amount of time dividing X-pos by 15!
	    bcs .divideby15
	   
	    tay         ; A is in [-1,-15]
	    lda fineAdjustTable,Y

	    sta HMP0,x ; Store the fine adjustment value.
        SLEEP 2  ; Waste cycles to correctly time RESET
	    sta RESP0,x ; Trigger RESET at cycle 19/24/29/.../69, leaving cycle <= 73
	    WAIT_SYNC 
	    rts 

;-----------------------------
; This table converts the "remainder" of the division by 15 (-1 to -15) to the correct
; fine adjustment value. It must be on the same page as the PosObject subroutine.
fineAdjustBegin
            DC.B %01100000; Left 6
            DC.B %01010000; Left 5
            DC.B %01000000; Left 4
            DC.B %00110000; Left 3
            DC.B %00100000; Left 2
            DC.B %00010000; Left 1
            DC.B %00000000; None
            DC.B %11110000; Right 1
            DC.B %11100000; Right 2
            DC.B %11010000; Right 3
            DC.B %11000000; Right 4
            DC.B %10110000; Right 5
            DC.B %10100000; Right 6
            DC.B %10010000; Right 7
            DC.B %10000000; Right 8

fineAdjustTable EQU fineAdjustBegin - %11110001; NOTE: %11110001 = -15
        
        ORG $FFFA
        
InterruptVectors

        .word Reset          ; NMI
        .word Reset          ; RESET
        .word Reset          ; IRQ

        END