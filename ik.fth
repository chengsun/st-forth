\ INVERSE KINEMATICS
\ the main file that tells the robot what to do
\ this file doesn't care whether it is running as a
\ simulation or on the actual robot. The two libraries
\ bv.fth and sim.fth deal with those intricacies

2147483647 CONSTANT SIGNEDMAX


VARIABLE RRNDSTATE
: RSEED ( u -- ) RRNDSTATE ! ;
: RRND ( -- u )
    RRNDSTATE @
    1664525 UM* DROP 1013904223 +
    DUP RRNDSTATE !
;

\ \\\\\\\\\\\\\\\\\\\\\\\\\\\\\
\ fixed point (1/10000) stuff
\ \\\\\\\\\\\\\\\\\\\\\\\\\\\\\

31416 CONSTANT FPPI
62832 CONSTANT FP2PI
15708 CONSTANT FPPI_2
7854 CONSTANT FPPI_4
23562 CONSTANT FP3PI_4

: FP* ( n1 n2 -- n )
    * 10000 /
;

: FP/ ( n1 n2 -- n )
    SWAP 10000 * SWAP /
;

: FPCOSLIM ( r -- n )
    \ http://www.ganssle.com/approx/approx.pdf
    \ r between 0 and pi/2
    DUP 0 < OVER FPPI_2 > OR IF ABORT" invalid cosine parameter" THEN
    DUP FP* DUP 368 FP* 4956 - FP* 9994 +
;

: FPCOS ( r -- n )
    \ r between -pi*2 and pi*2
    DUP FPPI > IF FP2PI - THEN
    DUP FPPI NEGATE < IF FP2PI + THEN
    DUP 0< IF NEGATE THEN
    DUP FPPI_2 > DUP >R IF FPPI SWAP - THEN
    FPCOSLIM
    R> IF NEGATE THEN
;

: FPSIN ( r -- n )
    \ r between -pi and pi
    DUP FPPI > IF FP2PI - THEN
    DUP FPPI NEGATE < IF FP2PI + THEN
    DUP 0< DUP >R IF NEGATE THEN
    DUP FPPI_2 < IF FPPI_2 SWAP - ELSE FPPI_2 - THEN
    FPCOSLIM
    R> IF NEGATE THEN
;

: FPCOSSIN ( r - x y )
    DUP FPCOS OVER FPSIN ROT DROP
;

: FPHYPOTSQR ( x1 y1 x2 y2 -- h^2 )
    ROT - DUP FP*       \ x1 x2 yd^2
    >R - DUP FP*        \ xd^2          / yd^2
    R> +
;

: FPRAD2DEG ( r -- d )
    180 * FPPI /
;

: DEG2FPRAD ( d -- r )
    FPPI * 180 /
;

\ \\\\\\\\\\\\\\\\\\\\\\\\\\\\\
\ inverse kinematics (2d)
\ \\\\\\\\\\\\\\\\\\\\\\\\\\\\\

\ the arm has length 10000 (fixed point 1.0000)


: 2DKPOS ( d1 d2 d3 -- x y )
    0 0 >R >R
    DEG2FPRAD SWAP DEG2FPRAD ROT DEG2FPRAD      \ r3 r2 r1
    DUP FPCOS R> + >R DUP FPSIN R> R> ROT + >R >R +
    DUP FPCOS R> + >R DUP FPSIN R> R> ROT + >R >R +
    DUP FPCOS R> + >R FPSIN R> R> ROT +
;

VARIABLE IKTHETA1
VARIABLE IKTHETA2
VARIABLE IKTHETA3

: 2DIKPOINTCBNOCHANGE -1 ;
: 2DIKPOINTCBTHETA1+ IKTHETA1 @ 1+ IKTHETA1 ! 0 ;
: 2DIKPOINTCBTHETA1- IKTHETA1 @ 1- IKTHETA1 ! 0 ;
: 2DIKPOINTCBTHETA2+ IKTHETA2 @ 1+ IKTHETA2 ! 0 ;
: 2DIKPOINTCBTHETA2- IKTHETA2 @ 1- IKTHETA2 ! 0 ;
: 2DIKPOINTCBTHETA3+ IKTHETA3 @ 1+ IKTHETA3 ! 0 ;
: 2DIKPOINTCBTHETA3- IKTHETA3 @ 1- IKTHETA3 ! 0 ;

: 2DIKPOINTITER ( x y -- stopflag )
    2DUP
    IKTHETA1 @ IKTHETA2 @ IKTHETA3 @
    2DKPOS FPHYPOTSQR
    ['] 2DIKPOINTCBNOCHANGE SWAP

    \ callback best_score

    >R >R 2DUP
    IKTHETA1 @ 1+ IKTHETA2 @ IKTHETA3 @
    2 PICK THETAOK? IF
        2DKPOS FPHYPOTSQR
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ['] 2DIKPOINTCBTHETA1+ R>
        ELSE DROP THEN
    ELSE 2DROP 2DROP DROP R> R> THEN

    >R >R 2DUP
    IKTHETA1 @ 1- IKTHETA2 @ IKTHETA3 @
    2 PICK THETAOK? IF
        2DKPOS FPHYPOTSQR
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ['] 2DIKPOINTCBTHETA1- R>
        ELSE DROP THEN
    ELSE 2DROP 2DROP DROP R> R> THEN

    >R >R 2DUP
    IKTHETA1 @ IKTHETA2 @ 1+ IKTHETA3 @
    OVER THETAOK? IF
        2DKPOS FPHYPOTSQR
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ['] 2DIKPOINTCBTHETA2+ R>
        ELSE DROP THEN
    ELSE 2DROP 2DROP DROP R> R> THEN

    >R >R 2DUP
    IKTHETA1 @ IKTHETA2 @ 1- IKTHETA3 @
    OVER THETAOK? IF
        2DKPOS FPHYPOTSQR
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ['] 2DIKPOINTCBTHETA2- R>
        ELSE DROP THEN
    ELSE 2DROP 2DROP DROP R> R> THEN

    >R >R 2DUP
    IKTHETA1 @ IKTHETA2 @ IKTHETA3 @ 1+
    DUP THETAOK? IF
        2DKPOS FPHYPOTSQR
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ['] 2DIKPOINTCBTHETA3+ R>
        ELSE DROP THEN
    ELSE 2DROP 2DROP DROP R> R> THEN

    >R >R 2DUP
    IKTHETA1 @ IKTHETA2 @ IKTHETA3 @ 1-
    DUP THETAOK? IF
        2DKPOS FPHYPOTSQR
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ['] 2DIKPOINTCBTHETA3- R>
        ELSE DROP THEN
    ELSE 2DROP 2DROP DROP R> R> THEN

    DROP EXECUTE >R
    2DROP R>
;

VARIABLE IKBESTTHETA1
VARIABLE IKBESTTHETA2
VARIABLE IKBESTTHETA3

: 2DIK ( x y -- t1 t2 t3 )
    SIGNEDMAX

    SIGNEDMAX IKBESTTHETA1 !
    SIGNEDMAX IKBESTTHETA2 !
    SIGNEDMAX IKBESTTHETA3 !

    \ do 10 minima searches with random initial positions
    10 0 DO
        >R

        RRND 160 MOD 80 - IKTHETA1 !
        RRND 160 MOD 80 - IKTHETA2 !
        RRND 160 MOD 80 - IKTHETA3 !

        BEGIN
            2DUP 2DIKPOINTITER
        UNTIL

        ." *BEST IS"  IKTHETA1 @ . IKTHETA2 @ . IKTHETA3 @ . CR

        2DUP
        IKTHETA1 @ IKTHETA2 @ IKTHETA3 @
        2DKPOS FPHYPOTSQR
        ." *THIS GIVES" DUP . CR
        R>
        \ x y hs besths
        2DUP < IF
            DROP
            IKTHETA1 @ IKBESTTHETA1 !
            IKTHETA2 @ IKBESTTHETA2 !
            IKTHETA3 @ IKBESTTHETA3 !
        ELSE
            SWAP DROP
        THEN

        \ break early if small error
        DUP 10 < IF LEAVE THEN
    LOOP

    DROP 2DROP

    \ return the best one
    IKBESTTHETA1 @ IKBESTTHETA2 @ IKBESTTHETA3 @
;

VARIABLE _2DIKLINEOLDSEED

: 2DIKLINE ( x1 y1 x2 y2 -- )
    RRNDSEED @ _2DIKLINEOLDSEED !

    2OVER ROT SWAP - >R - R>        \ x1 y1 dx dy

    3 1 DO
        _2DIKLINEOLDSEED @ I + RSEED

        2OVER
        2DIK
        IKTHETA3 ! IKTHETA2 ! IKTHETA1 !
                                    \ x1 y1 dx dy
        100 0 DO
            2OVER                   \ x1 y1 dx dy x1 y1
            ROT                     \ x1 y1 dx x1 y1 dy
            I * 100 / +             \ x1 y1 dx x1 y
            >R SWAP
            I * 100 / +
            R>                      \ x1 y1 x y

            2DUP 2DIKPOINTITER
            IF
                \ couldn't improve at all
                2DUP _2DIKLINELASTY ! _2DIKLINELASTX ! FPHYPOTSQR
                3000 > IF
                    \ and we've moved quite far (>0.1 lengths)
                    \ give up, go back to the last good position and try again
                THEN
            ELSE
                2DUP _2DIKLINELASTY ! _2DIKLINELASTX !
                BEGIN
                    2DUP 2DIKPOINTITER
                UNTIL
            THEN
        LOOP
    LOOP

    _2DIKLINEOLDSEED @ RSEED
;


: TESTELLIPSE
    3000 0 DO
        \ get a point on the ellipse to plot
        RRND FP2PI MOD
        DUP FPCOS 2 * SWAP FPSIN 3 *
        2DUP DEBUGPLOT

        \ inverse kinematics calculation
        2DIK

        \ perform
        SETTHETA3 SETTHETA2 SETTHETA1
        PENDOWN
        PENUP
    LOOP
;

: TESTPOINT
    09000 -29000 DEBUGPLOT
    30 0 DO
        RRND 160 MOD 80 - IKTHETA1 !
        RRND 160 MOD 80 - IKTHETA2 !
        RRND 160 MOD 80 - IKTHETA3 !

        BEGIN
            09000 -29000 2DIKPOINTITER
        UNTIL

        IKTHETA1 @ SETTHETA1
        IKTHETA2 @ SETTHETA2
        IKTHETA3 @ SETTHETA3
        PENDOWN
        PENUP
    LOOP
;

: TESTLINE
    -10000 -20000 10000 20000
;

: MAIN 
    12345 RSEED
    INIT
    PENUP

    TESTLINE
;

\ : MAINTEST3
\     80 -80 DO
\         I SETTHETA1
\         MOVEMOTORITER
\     LOOP
\ ;
