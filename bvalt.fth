\ code that is used on the actual robot

: 2VARIABLE CREATE 2 CELLS ALLOT ;

: DEBUG ( e-addr ulen -- )
    TYPE
;


        150 0 do
            I FUDGEFACTOR
            SETPWMMATCH2                                                      
            SETPWMLER2
            20 @ ms
        LOOP


: INIT
    SETPWMON
;



\ \\\\\\\\\\\\\\\\\\\\\\\\\\\\\
\ motor stuff
\ \\\\\\\\\\\\\\\\\\\\\\\\\\\\\

70 CONSTANT zero_position
30 CONSTANT delay

VARIABLE ms_angle1
VARIABLE ms_angle2
VARIABLE ms_angle3

VARIABLE THETA1
VARIABLE THETA2
VARIABLE THETA3

: THETA2MOTOR ( t -- m ) 80 + 8 * 9 / zero_position + ;
: THETAOK? ( theta -- flag ) DUP -81 > SWAP 81 < AND ;
\ SETTHETAn ( theta -- )
: SETTHETA1 DUP THETAOK? INVERT IF ABORT" THETA1 VALUE OVERFLOW" THEN DUP THETA1 ! THETA2MOTOR ms_angle1 ! MOVETO ;
: SETTHETA2 DUP THETAOK? INVERT IF ABORT" THETA3 VALUE OVERFLOW" THEN DUP THETA2 ! THETA2MOTOR ms_angle2 ! MOVETO ;
: SETTHETA3 DUP THETAOK? INVERT IF ABORT" THETA3 VALUE OVERFLOW" THEN DUP THETA3 ! THETA2MOTOR ms_angle3 ! MOVETO ;

: FUDGEFACTOR 22177 * 150 / ;
: MOVEMOTORITER ( -- )
    ms_angle1 @ FUDGEFACTOR SETPWMMATCH5
    SETPWMLER5
    ms_angle2 @ FUDGEFACTOR SETPWMMATCH4
    SETPWMLER4
    ms_angle3 @ FUDGEFACTOR SETPWMMATCH6
    SETPWMLER6

    delay ms
;

: PENDOWN
    \ TODO
;

: PENUP
    \ TODO
;
