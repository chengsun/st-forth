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
    10000 */
;

: FP/ ( n1 n2 -- n )
    10000 SWAP */
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

: 2DIKCBNO -1 ;
: 2DIKCB1+ IKTHETA1 @ 1+ IKTHETA1 ! 0 ;
: 2DIKCB1- IKTHETA1 @ 1- IKTHETA1 ! 0 ;
: 2DIKCB2+ IKTHETA2 @ 1+ IKTHETA2 ! 0 ;
: 2DIKCB2- IKTHETA2 @ 1- IKTHETA2 ! 0 ;
: 2DIKCB3+ IKTHETA3 @ 1+ IKTHETA3 ! 0 ;
: 2DIKCB3- IKTHETA3 @ 1- IKTHETA3 ! 0 ;


\ takes a point and a set of motor positions in IKTHETAn, and tries to move
\ one of the motors closer to the point
: 2DIKPOINTITER ( x y -- stopflag )
    2DUP
    IKTHETA1 @ IKTHETA2 @ IKTHETA3 @
    2DKPOS FPHYPOTSQR
    ['] 2DIKCBNO SWAP

    \ callback best_score

    >R >R 2DUP
    IKTHETA1 @ 1+ IKTHETA2 @ IKTHETA3 @
    2 PICK THETAOK? IF
        2DKPOS FPHYPOTSQR
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ['] 2DIKCB1+ R>
        ELSE DROP THEN
    ELSE 2DROP 2DROP DROP R> R> THEN

    >R >R 2DUP
    IKTHETA1 @ 1- IKTHETA2 @ IKTHETA3 @
    2 PICK THETAOK? IF
        2DKPOS FPHYPOTSQR
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ['] 2DIKCB1- R>
        ELSE DROP THEN
    ELSE 2DROP 2DROP DROP R> R> THEN

    >R >R 2DUP
    IKTHETA1 @ IKTHETA2 @ 1+ IKTHETA3 @
    OVER THETAOK? IF
        2DKPOS FPHYPOTSQR
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ['] 2DIKCB2+ R>
        ELSE DROP THEN
    ELSE 2DROP 2DROP DROP R> R> THEN

    >R >R 2DUP
    IKTHETA1 @ IKTHETA2 @ 1- IKTHETA3 @
    OVER THETAOK? IF
        2DKPOS FPHYPOTSQR
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ['] 2DIKCB2- R>
        ELSE DROP THEN
    ELSE 2DROP 2DROP DROP R> R> THEN

    >R >R 2DUP
    IKTHETA1 @ IKTHETA2 @ IKTHETA3 @ 1+
    DUP THETAOK? IF
        2DKPOS FPHYPOTSQR
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ['] 2DIKCB3+ R>
        ELSE DROP THEN
    ELSE 2DROP 2DROP DROP R> R> THEN

    >R >R 2DUP
    IKTHETA1 @ IKTHETA2 @ IKTHETA3 @ 1-
    DUP THETAOK? IF
        2DKPOS FPHYPOTSQR
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ['] 2DIKCB3- R>
        ELSE DROP THEN
    ELSE 2DROP 2DROP DROP R> R> THEN

    DROP EXECUTE >R
    2DROP R>
;

VARIABLE IKBESTTHETA1
VARIABLE IKBESTTHETA2
VARIABLE IKBESTTHETA3

\ takes a point and gives a set of motor positions at the point
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

        2DUP
        IKTHETA1 @ IKTHETA2 @ IKTHETA3 @
        2DKPOS FPHYPOTSQR
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

VARIABLE _2DIKOLDSEED
VARIABLE _2DIKLASTI
VARIABLE _2DIKPERFORM
VARIABLE _LINELENGTH

\ takes a line and a starting point and tries to draw down the line as far as
\ possible
: 2DIKLINEPART ( x1 y1 dx dy jstart -- x1 y1 dx dy jend ) \ uses _LINELENGTH
    >R                      \ stash jstart

    2OVER 2OVER
    ROT SWAP                \ x1 dx y1 dy
    R@ * _LINELENGTH @ / +   \ x1 dx y
    ROT ROT                 \ y x1 dx
    R@ * _LINELENGTH @ / +
    SWAP                    \ x y

    2DIK
    IKTHETA3 ! IKTHETA2 ! IKTHETA1 !
    _2DIKPERFORM @ IF
        PENUP
        IKTHETA1 @ SETTHETA1
        IKTHETA2 @ SETTHETA2
        IKTHETA3 @ SETTHETA3
        PENDOWN
    THEN
    
    BEGIN
        R> 1+ >R

        R@ _LINELENGTH @ = IF
            -1
        ELSE
            2OVER 2OVER             \ x1 y1 dx dy
            ROT SWAP                \ x1 dx y1 dy
            R@ * _LINELENGTH @ / +    \ x1 dx y
            ROT ROT                 \ y x1 dx
            R@ * _LINELENGTH @ / +
            SWAP                    \ x y

            2DUP 2DIKPOINTITER
            IF
                2DROP
                \ couldn't improve at all
                R@ _2DIKLASTI @ - 5 > IF
                    \ we've tried moving quite far (5*0.02 of the
                    \ length of an arm), so give up
                    R> 5 - >R
                    -1
                ELSE
                    0
                THEN
            ELSE
                R@ _2DIKLASTI !
                _2DIKPERFORM @ IF
                    IKTHETA1 @ SETTHETA1
                    IKTHETA2 @ SETTHETA2
                    IKTHETA3 @ SETTHETA3
                THEN
                BEGIN
                    2DUP 2DIKPOINTITER
                    _2DIKPERFORM @ IF
                        IKTHETA1 @ SETTHETA1
                        IKTHETA2 @ SETTHETA2
                        IKTHETA3 @ SETTHETA3
                    THEN
                UNTIL
                IKTHETA1 @ IKTHETA2 @ IKTHETA3 @ 2DKPOS FPHYPOTSQR 40 > IF
                    \ too much error
                    -1
                ELSE
                    0
                THEN
            THEN
        THEN
    UNTIL

    R>
;

VARIABLE _2DIKLBSEED
VARIABLE _BESTSCORE
VARIABLE _CURSCORE

: 2DIKLINE ( x1 y1 x2 y2 -- bestseed )
    RRNDSTATE @ _2DIKOLDSEED !

    2OVER 2OVER FPHYPOTSQR 200 / _LINELENGTH ! \ store the line length
    2OVER ROT SWAP - >R - R>        \ x1 y1 dx dy

    _LINELENGTH @ 10 / 3 + _BESTSCORE !
    0 _2DIKPERFORM !

    6 1 DO
        ." * DOING ITERATION " I . CR
        _2DIKOLDSEED @ I + RSEED

        0 _CURSCORE !        \ reset the current score
        0                   \ set the current position along the line
        BEGIN
            \ x1 y1 dx dy jmax jstart
            _CURSCORE @ 1+ DUP _CURSCORE !    \ increment the current score
            _BESTSCORE = IF
                \ bail out early if we've had to change 3 times
                -1
            ELSE
                2DIKLINEPART ( x1 y1 dx dy jstart -- x1 y1 dx dy jend )
                DUP _LINELENGTH @ =
            THEN
        UNTIL
        DROP

        _CURSCORE @ _BESTSCORE @ < IF
            _CURSCORE @ _BESTSCORE !
            _2DIKOLDSEED @ I + _2DIKLBSEED !
        THEN
    LOOP

    _2DIKLBSEED @ RSEED
    ." * BEST IS " _2DIKLBSEED @ _2DIKOLDSEED @ - .  ." WITH " _BESTSCORE @ . CR
    -1 _2DIKPERFORM !

    0 _CURSCORE !        \ reset the current score
    0        \ set the current position along the line
    BEGIN
        _CURSCORE @ 1+ DUP _CURSCORE !    \ increment the current score
        _BESTSCORE = IF
            \ bail out early if we've had to change 3 times
            -1
        ELSE
            2DIKLINEPART ( x1 y1 dx dy jstart -- x1 y1 dx dy jend )
            DUP _LINELENGTH @ =
        THEN
    UNTIL
    DROP

    2DROP 2DROP \ drop x1 y1 dx dy

    PENUP
    _2DIKOLDSEED @ RSEED
;


: TESTELLIPSE
    100 0 DO
        \ get a point on the ellipse to plot
        RRND FP2PI MOD
        DUP FPCOS 2 * SWAP FPSIN 3 *
        \ 2DUP DEBUGPLOT

        \ inverse kinematics calculation
        2DIK

        \ perform
        SETTHETA3 SETTHETA2 SETTHETA1
        PENDOWN
        PENUP
        
        ." * ITERATION " I . CR
    LOOP
;

: TESTPOINT
    \ 09000 -29000 DEBUGPLOT
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
    25000 -20000 15000 -20000 2DIKLINE
    20000 -20000 20000 -15000 2DIKLINE
    25000 -15000 15000 -15000 2DIKLINE

    25000 -12000 15000 -12000 2DIKLINE
    25000 -12000 25000 -08000 2DIKLINE
    20000 -12000 20000 -08000 2DIKLINE
    15000 -12000 15000 -08000 2DIKLINE

    25000 -05000 15000 -05000 2DIKLINE
    15000 -05000 15000 -00000 2DIKLINE

    25000 02000 15000 02000 2DIKLINE
    15000 02000 15000 07000 2DIKLINE

    25000 10000 15000 10000 2DIKLINE
    15000 10000 15000 15000 2DIKLINE
    25000 10000 25000 15000 2DIKLINE
    25000 15000 15000 15000 2DIKLINE

    25000 17000 18000 17000 2DIKLINE
    15000 17000 2DIK SETTHETA3 SETTHETA2 SETTHETA1 PENDOWN PENUP

    13000 -20000 13000 20000 2DIKLINE

    0 0 2DIK SETTHETA3 SETTHETA2 SETTHETA1
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
