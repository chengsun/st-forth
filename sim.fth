\ code that is used on the simulation


: TELL ;

0 CONSTANT MOTOR1
1 CONSTANT MOTOR2
2 CONSTANT MOTOR3

: MOVETO ( motor theta -- )
    SWAP ." M" . . CR
    KEY DROP
;

: PENDOWN
    ." P1" CR
    KEY DROP
;

: PENUP
    ." P0" CR
    KEY DROP
;

: DEBUG ( e-addr ulen -- )
    ." *" TYPE CR
;

: DEBUGSTACK ( -- )
    ." *" .S CR
;

: DEBUGPLOT ( x y -- )
    SWAP ." +" . . CR
;

: INIT ;

VARIABLE THETA1
VARIABLE THETA2
VARIABLE THETA3

: THETA2MOTOR ( t -- m ) 80 + ;
: THETAOK? ( theta -- flag ) DUP -81 > SWAP 81 < AND ;
\ SETTHETAn ( theta -- )
: SETTHETA1 DUP THETAOK? INVERT IF ABORT" THETA1 VALUE OVERFLOW" THEN DUP THETA1 ! >R TELL MOTOR1 R> THETA2MOTOR MOVETO ;
: SETTHETA2 DUP THETAOK? INVERT IF ABORT" THETA3 VALUE OVERFLOW" THEN DUP THETA2 ! >R TELL MOTOR2 R> THETA2MOTOR MOVETO ;
: SETTHETA3 DUP THETAOK? INVERT IF ABORT" THETA3 VALUE OVERFLOW" THEN DUP THETA3 ! >R TELL MOTOR3 R> THETA2MOTOR MOVETO ;


\ bv-forth
: us 0 do loop ;

