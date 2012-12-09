12345 SEED

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
    \ r between -pi and pi
    DUP 0< IF NEGATE THEN
    DUP FPPI_2 > DUP >R IF FPPI SWAP - THEN
    FPCOSLIM
    R> IF NEGATE THEN
;

: FPSIN ( r -- n )
    \ r between -pi and pi
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
    180 * FPPI /
;

: DEG2FPRAD ( d -- r )
    FPPI * 180 /
;

\ inverse kinematics (2d)
\ the arm has length 10000

2147483647 CONSTANT SIGNEDMAX

VARIABLE THETA1
VARIABLE THETA2
VARIABLE THETA3

: 2DKPOS ( d1 d2 d3 -- x y )
    0 0 >R >R
    DEG2FPRAD SWAP DEG2FPRAD ROT DEG2FPRAD      \ r3 r2 r1
    DUP FPCOS R> R> 2 PICK + >R >R DROP DUP FPSIN R> + >R +
    DUP FPCOS R> R> 2 PICK + >R >R DROP DUP FPSIN R> + >R +
    DUP FPCOS R> R> 2 PICK + >R >R DROP FPSIN R> + R>
;

: THETA2MOTOR ( theta -- motor )
    80 + 
;

2VARIABLE ERROR
: ERROR? ( -- flag ) ERROR CELL+ @ 0= ;
: NOERROR? ( -- flag ) ERROR? INVERT;
: SETERROR ( e-addr ulen -- ) 2DUP ERROR 2! CR ." ERROR: " TYPE ;
: RESETERROR ( -- ) 0 0 SETERROR CR ." ERROR CLEARED";

\ SETTHETAn ( theta -- )
: SETTHETA1 THETA1 ! TELL MOTOR1 THETA1 @ THETA2MOTOR MOVETO ;
: SETTHETA2 THETA2 ! TELL MOTOR2 THETA2 @ THETA2MOTOR MOVETO ;
: SETTHETA3 THETA3 ! TELL MOTOR3 THETA3 @ THETA2MOTOR MOVETO ;
: _2DIKPOINTCBNOCHANGE " could not find an improved arm position" SETERROR ;
: _2DIKPOINTCBTHETA1+ THETA1 @ 1+ SETTHETA1 ;
: _2DIKPOINTCBTHETA1- THETA1 @ 1- SETTHETA1 ;
: _2DIKPOINTCBTHETA2+ THETA2 @ 1+ SETTHETA2 ;
: _2DIKPOINTCBTHETA2- THETA2 @ 1- SETTHETA2 ;
: _2DIKPOINTCBTHETA3+ THETA3 @ 1+ SETTHETA3 ;
: _2DIKPOINTCBTHETA3- THETA3 @ 1- SETTHETA3 ;

: 2DIKPOINT ( x y -- )
    2DUP
    THETA1 @ THETA2 @ THETA3 @
    2DKPOS FPHYPOTSQR
    ' _2DIKPOINTCBNOCHANGE SWAP

    \ callback best_score

    >R >R 2DUP
    THETA1 @ 1+ THETA2 @ THETA3 @
    2DKPOS FPHYPOTSQR
    NOERROR? IF
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ' _2DIKPOINTCBTHETA1+ R>
        ELSE DROP THEN
    ELSE DROP RESETERROR THEN

    >R >R 2DUP
    THETA1 @ 1- THETA2 @ THETA3 @
    2DKPOS FPHYPOTSQR
    NOERROR? IF
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ' _2DIKPOINTCBTHETA1- R>
        ELSE DROP THEN
    ELSE DROP RESETERROR THEN

    >R >R 2DUP
    THETA1 @ THETA2 @ 1+ THETA3 @
    2DKPOS FPHYPOTSQR
    NOERROR? IF
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ' _2DIKPOINTCBTHETA2+ R>
        ELSE DROP THEN
    ELSE DROP RESETERROR THEN

    >R >R 2DUP
    THETA1 @ THETA2 @ 1- THETA3 @
    2DKPOS FPHYPOTSQR
    NOERROR? IF
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ' _2DIKPOINTCBTHETA2- R>
        ELSE DROP THEN
    ELSE DROP RESETERROR THEN

    >R >R 2DUP
    THETA1 @ THETA2 @ THETA3 @ 1+
    2DKPOS FPHYPOTSQR
    NOERROR? IF
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ' _2DIKPOINTCBTHETA3+ R>
        ELSE DROP THEN
    ELSE DROP RESETERROR THEN

    >R >R 2DUP
    THETA1 @ THETA2 @ THETA3 @ 1-
    2DKPOS FPHYPOTSQR
    NOERROR? IF
        R> R> ROT   \ callback best_score this_score
        2DUP > IF
            >R 2DROP ' _2DIKPOINTCBTHETA3+ R>
        ELSE DROP THEN
    ELSE DROP RESETERROR THEN

    DROP EXECUTE

    ERROR? IF
        RND 160 MOD THETA1 !
        RND 160 MOD THETA2 !
        RND 160 MOD THETA3 !
    THEN
;
