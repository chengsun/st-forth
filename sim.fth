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
    ." +" . . CR
;


\ bv-forth
: us 0 do loop ;


VARIABLE RNDSTATE
: SEED ( u -- ) RNDSTATE ! ;
: RND ( -- u )
    RNDSTATE @
    1664525 UM* DROP 1013904223 +
    DUP RNDSTATE !
;
