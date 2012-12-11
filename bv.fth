\ code that is used on the actual robot

: 2VARIABLE CREATE 2 CELLS ALLOT ;

: DEBUG ( e-addr ulen -- )
    TYPE
;

20 delay !
100 t1 !


VARIABLE THETA1
VARIABLE THETA2
VARIABLE THETA3

: THETA2MOTOR ( t -- m ) 80 + ;
: THETAOK? ( theta -- flag ) DUP -81 > SWAP 81 < AND ;
\ SETTHETAn ( theta -- )
: SETTHETA1 DUP THETAOK? INVERT IF ABORT" THETA1 VALUE OVERFLOW" THEN DUP THETA1 ! >R TELL MOTOR1 R> THETA2MOTOR MOVETO ;
: SETTHETA2 DUP THETAOK? INVERT IF ABORT" THETA3 VALUE OVERFLOW" THEN DUP THETA2 ! >R TELL MOTOR2 R> THETA2MOTOR MOVETO ;
: SETTHETA3 DUP THETAOK? INVERT IF ABORT" THETA3 VALUE OVERFLOW" THEN DUP THETA3 ! >R TELL MOTOR3 R> THETA2MOTOR MOVETO ;

: INIT ;


: PENDOWN
    \ TODO
;

: PENUP
    \ TODO
;
