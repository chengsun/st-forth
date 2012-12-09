: 2VARIABLE CREATE 2 CELLS ALLOT ;

: DEBUG ( e-addr ulen -- )
    TYPE
;

\ START, but modified a bit

250 max_position !
150 center !
70 zero_position !

0 flag !
0 error !

0 movelog_motor1 !
0 movelog_motor2 !
0 movelog_motor3 !
0 movelog_motor4 !
0 movelog_motor5 !
0 ms_angle1 !
0 ms_angle2 !
0 ms_angle3 !
0 ms_angle4 !
21 delay !
0 flag !
data pnext !
\ 1000 t1 !               \ this seems to trigger a delay of 1 second every
                          \ time MOVE* is called?
200 t1 !
SETALLPCR

50 SETSPEED
