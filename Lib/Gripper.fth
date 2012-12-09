
// Code for controlling the motors in the small motor3
// ************Geena Chacko, ST Robotics********************

variable angle
variable center
variable max_position
variable zero_position
variable port
variable input_angle
Variable port
variable flag
variable error
variable t1


2500 max_position !
1500 center !
500 zero_position !
0 flag !
0 error !

: us 0 do loop ;

: delay t1 @ 100000000 * ms drop ;

: tell

0 input_angle !
0 angle !
0 port !
2 flag !
0 error !


;


: motor1

flag @  0 = if ." try the format as specified "
            1 error !
            drop
	    else 8 io0-out 8 port !  
	    then  
;

: motor2

flag @  0 = if ." try the format as specified "
            1 error !
            drop
	    else 7 io0-out 7 port !  
	    then 

;

: motor3

flag @  0 = if ." try the format as specified "
            1 error !
            drop
	    else 4 io0-out 4 port !  
	    then 

;

: motor4
flag @  0 = if ." try the format as specified "
            1 error !
            drop
	    else 5 io0-out 5 port !  
	    then 


;

: motor5

flag @  0 = if ." try the format as specified "
            drop
            1 error !
	    else 6 io0-out 6 port !  
	    then 

;


variable movelog_motor1
variable movelog_motor2
variable movelog_motor3
variable movelog_motor4
variable movelog_motor5

variable old_angle
variable high_time 
variable low_time
variable increment

: servocontrol

	high_time !
	port @ 4 = if 
		100 9 / movelog_motor3 @ * zero_position @ + old_angle ! else then
	port @ 5 = if 
		100 9 / movelog_motor4 @ * zero_position @ + old_angle ! else then   
	port @ 6 = if 
		100 9 / movelog_motor5 @ * zero_position @ + old_angle ! else then   
	port @ 7 = if 
		100 9 / movelog_motor2 @ * zero_position @ + old_angle ! else then   
	port @ 8 = if 
		 100 9 / movelog_motor1 @ * zero_position @ + old_angle ! else then
  
	port @ io0-out
	0 port @ p0!
	10 increment !



// high_time @ old_angle @ - abs 10 / 0 do

10 0 do

   1 port @ p0!   
   high_time @ old_angle @ < if
        old_angle @ increment @ - us
   	0 port @ p0!
   	20000 old_angle @ - increment @ + low_Time !
   	low_time @ us
   else
	old_angle @ increment @ + us
   	0 port @ p0!
   	20000 old_angle @ - increment @ - low_Time !
   	low_time @ us
   then

  10 increment @ + increment !
        

loop




;

0 movelog_motor1 !
0 movelog_motor2 !
0 movelog_motor3 !
0 movelog_motor4 !
0 movelog_motor5 !

: moveto

error @ 1 = if
            else 
		input_angle !
	    	100 9 / input_angle @ * zero_position @ + angle !
                input_angle @ 100 > if input_angle @  1000 /  angle @ + angle !
                                    else 1000 input_angle @ / angle @ + angle ! 
                                    then

                
	       angle @ servocontrol 
               

               port @ 4 = if input_angle @ movelog_motor3 ! else then
               port @ 5 = if input_angle @ movelog_motor4 ! else then   
               port @ 6 = if input_angle @ movelog_motor5 ! else then   
               port @ 7 = if input_angle @ movelog_motor2 ! else then   
               port @ 8 = if input_angle @ movelog_motor1 ! else then    
               0 flag !
	    then


;

: calibrate

tell motor1 0 moveto
10 ms
tell motor2 0 moveto
10 ms
tell motor3 0 moveto
10 ms
tell motor5 0 moveto
10 ms
tell motor4 0 moveto

;


variable move_angle
: allmove

move_angle !

tell motor1 move_angle @ moveto
10 us
tell motor2 move_angle @ moveto
10 us
tell motor3 move_angle @ moveto
10 us
tell motor5 move_angle @  moveto
10 us
tell motor4 move_angle @ moveto


;


: where
cr ." motor1    :" movelog_motor1 @ .
cr ." motor2 : " movelog_motor2 @ .
cr ." motor3      : " movelog_motor3 @ .
cr ." motor4    : " movelog_motor4 @ .
cr ." motor5  :" movelog_motor5 @ .
 
;

: demo
	tell motor1 80 moveto
	tell motor5 175 moveto
	tell motor2 100 moveto
	tell motor5 0 moveto
	tell motor2 45 moveto
	tell motor1 0 moveto
	tell motor2 90 moveto
	tell motor5 175 moveto
	calibrate
;


create data 100 allot
variable pnext

: startover

data pnext !


;


variable plad
: place
	create
        pnext @ ,
        movelog_motor1 @ pnext @ !
        4 pnext +!
	movelog_motor2 @ pnext @ !
        4 pnext +!
        movelog_motor3 @ pnext @ !
        4 pnext +!
	movelog_motor4 @ pnext @ !
	4 pnext +!
        movelog_motor5 @ pnext @ !
        16 pnext +!
       

	does> 
        [ decimal ]
        @ 
        dup 16 + @ tell motor5 moveto
        dup @ tell motor1 moveto
        dup 4 + @ tell motor2 moveto
        dup 8 + @ tell motor3 moveto
        dup 12 + @ tell motor4 moveto
        drop
;


: grip
tell motor5 0 moveto
;

: ungrip
tell motor5 175 moveto
;

: relearn
'
24 + @
movelog_motor1 @ over !
movelog_motor2 @ over 4 + !
movelog_motor3 @ over 8 + !
movelog_motor4 @ over 12 + !
movelog_motor5 @ over 16 + !
drop
;

variable motor_value

: energize

motor_value !
25 0 do
tell motor2 motor_value @ moveto
5 us
tell motor2 motor_value @ moveto

loop


;