\ fixed point (1/10000) stuff

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
    ROT -               \ x1 x2 yd
    DUP FP*             \ x1 x2 yd^2
    ROT ROT -           \ yd^2 xd
    DUP FP*             \ yd^2 xd^2
    +                   \ xd^2+yd^2
;

: FPRAD2DEG ( r -- d )
    150 * FPPI /
;

: DEG2FPRAD ( d -- r )
    FPPI * 150 /
;


\ motor stuff

2147483647 CONSTANT SIGNEDMAX

VARIABLE THETA1
VARIABLE THETA2
VARIABLE THETA3

: THETA2MOTOR ( theta -- motor ) 80 + ;
: THETAOK? ( theta -- flag ) DUP -81 > SWAP 81 < AND ;
\ SETTHETAn ( theta -- )
: SETTHETA1 DUP THETAOK? IF THETA1 ! TELL MOTOR1 THETA1 @ THETA2MOTOR MOVETO ELSE ABORT" THETA1 VALUE OVERFLOW" THEN ;
: SETTHETA2 DUP THETAOK? IF THETA2 ! TELL MOTOR2 THETA2 @ THETA2MOTOR MOVETO ELSE ABORT" THETA2 VALUE OVERFLOW" THEN ;
: SETTHETA3 DUP THETAOK? IF THETA3 ! TELL MOTOR3 THETA3 @ THETA2MOTOR MOVETO ELSE ABORT" THETA3 VALUE OVERFLOW" THEN ;


\ inverse kinematics (2d)
\ the arm has length 10000


: 2DKPOS ( d1 d2 d3 -- x y )
    0 0 >R >R
    DEG2FPRAD SWAP DEG2FPRAD ROT DEG2FPRAD      \ r3 r2 r1
    DUP FPCOS R> R> 2 PICK + >R >R DROP DUP FPSIN R> + >R +
    DUP FPCOS R> R> 2 PICK + >R >R DROP DUP FPSIN R> + >R +
    DUP FPCOS R> R> 2 PICK + >R >R DROP FPSIN R> + R>
;

VARIABLE IKTHETA1
VARIABLE IKTHETA2
VARIABLE IKTHETA3
VARIABLE IKBESTTHETA1
VARIABLE IKBESTTHETA2
VARIABLE IKBESTTHETA3
VARIABLE IKBESTHYPOTSQR

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

: MAINTESt
    12345 SEED
    PENUP
    3000 0 DO
        \ get a point to plot
        RND FP2PI MOD
        DUP FPCOS 2 * SWAP FPSIN 3 *
        2DUP DEBUGPLOT

        SIGNEDMAX IKBESTHYPOTSQR !

        \ do 10 minima searches with random initial positions
        10 0 DO
            RND 160 MOD 80 - IKTHETA1 !
            RND 160 MOD 80 - IKTHETA2 !
            RND 160 MOD 80 - IKTHETA3 !

            BEGIN
                2DUP 2DIKPOINTITER
            UNTIL

            2DUP
            IKTHETA1 @ IKTHETA2 @ IKTHETA3 @
            2DKPOS FPHYPOTSQR
            DUP IKBESTHYPOTSQR @ < IF
                DUP IKBESTHYPOTSQR !
                IKTHETA1 @ IKBESTTHETA1 !
                IKTHETA2 @ IKBESTTHETA2 !
                IKTHETA3 @ IKBESTTHETA3 !
            THEN

            \ break early if < 0.05 error
            10 < IF LEAVE THEN
        LOOP

        \ perform the best one
        IKBESTTHETA1 @ SETTHETA1
        IKBESTTHETA2 @ SETTHETA2
        IKBESTTHETA3 @ SETTHETA3

        PENDOWN
        PENUP

        2DROP
    LOOP
;

: MAIN
    12345 SEED
    PENUP
    
    15000 15000 DEBUGPLOT
    30 0 DO
        RND 160 MOD 80 - IKTHETA1 !
        RND 160 MOD 80 - IKTHETA2 !
        RND 160 MOD 80 - IKTHETA3 !

        BEGIN
            15000 15000 2DIKPOINTITER
        UNTIL

        IKTHETA1 @ SETTHETA1
        IKTHETA2 @ SETTHETA2
        IKTHETA3 @ SETTHETA3
        PENDOWN
        PENUP
    LOOP
;

