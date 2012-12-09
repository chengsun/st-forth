: TELL ;

1 CONSTANT MOTOR1
2 CONSTANT MOTOR2
3 CONSTANT MOTOR3

: MOVETO ( motor theta -- )
    ." M" . ."  " .
    KEY DROP
;
: MOVE ( motor theta -- )
    ." D" . ."  " .
    KEY DROP
;

: START ( -- )
    ." S"
    KEY DROP
;

: CALIBRATE ( -- )
    ." C"
    KEY DROP
;

\ bv-forth

VARIABLE RNDSTATE

: SEED ( u -- )
    RNDSTATE !
;

: RND ( -- u )
    RNDSTATE @
    1664525 UM* DROP 1013904223 +
    DUP RNDSTATE !
;
