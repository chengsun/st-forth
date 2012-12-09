
// Code for controlling the motors using PWM signals Version 1.2
// ************Geena Chacko,Applications Engineer,ST Robotics********************

// #LIBRARY "SOFT1.FTH"
// #LIBRARY "PINSEL.FTH"
// #LIBRARY "DECOMPILE.FTH"


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
variable movelog_motor1
variable movelog_motor2
variable movelog_motor3
variable movelog_motor4
variable movelog_motor5
variable old_angle
variable high_time 
variable low_time
variable increment
Variable loop_time
variable temp_time
variable ms_angle1
variable ms_angle2
variable ms_angle3
variable ms_angle4
variable delay
variable t1


create data 100 allot
variable pnext


&E0014004 CONSTANT PWMTCR            // PWM TIMER CONTROL REGISTER
&E0014008 CONSTANT PWMTC             // PWM TIMER COUNTER REGISTER
&E001400C CONSTANT PWMPRE            // PWM PRESCALE REGISTER
&E001400C CONSTANT PWMPR             // PWM PRESCALE CONTROL REGISTER
&E0014010 CONSTANT PWMPC             // PWM PRESCALE COUNTER REGISTER
&E0014014 CONSTANT PWMMCR            // PWM MATCH CONTROL REGISTER

&E0014018 CONSTANT PWMMR0            // PWM MATCH REGISTER 0
&E0014020 CONSTANT PWMMR2            // PWM MATCH REGISTER 2
&E0014040 CONSTANT PWMMR4            // PWM MATCH REGISTER 4
&E0014044 CONSTANT PWMMR5            // PWM MATCH REGISTER 5
&E0014048 CONSTANT PWMMR6            // PWM MATCH REGISTER 6


&E001404C CONSTANT PWMPCR            // PWM CONTROL REGISTER
&E0014050 CONSTANT PWMLER            // PWM LATCH ENABLE REGISTER

// Initialization
: SETP0PWM 0 2 PINSELECT ;           // P00 PWM1
: SETP1PWM 1 2 PINSELECT ;           // P01 PWM3
: SETP7PWM 7 2 PINSELECT ;           // P07 PWM2
: SETP8PWM 8 2 PINSELECT ;           // P08 PWM4
: SETP21PWM 21 1 PINSELECT ;         // P21 PWM5
: SETP9PWM 9 2 PINSELECT ;           // P09 PWM6

: RSTCNTR &02 PWMTCR ! ;             // RESET COUNTER
: SETPWMON &09 PWMTCR ! ;            // COUNTER ENABLE PWM ENABLE
: SETPWMPRESET PWMPRE ! ;            // PWM PRESET
: SETPWMCONTROL &00000002 PWMMCR ! ; // PWM MATCH ON PWMMR0

: SETPWMMATCH0 PWMMR0 ! ;            // PWMMATCH 0 REGISTER
: SETPWMMATCH2 PWMMR2 ! ;            // PWMMATCH 2 REGISTER
: SETPWMMATCH4 PWMMR4 ! ;            // PWMMATCH 4 REGISTER
: SETPWMMATCH5 PWMMR5 ! ;            // PWMMATCH 5 REGISTER
: SETPWMMATCH6 PWMMR6 ! ;            // PWMMATCH 6 REGISTER


: SETPWMLER2 &5 PWMLER ! ;            // PWM LATCH ENABLE REGISTER  for PWM2
: SETPWMLER4 &11 PWMLER ! ;            // PWM LATCH ENABLE REGISTER for PWM4
: SETPWMLER5 &21 PWMLER ! ;            // PWM LATCH ENABLE REGISTER for PWM5
: SETPWMLER6 &41 PWMLER ! ;            // PWM LATCH ENABLE REGISTER FOR PWM6


: SETPWM2PCR &0400 PWMPCR ! ;        // PWM PRESCALE CONTROL REGISTER for PWM2
: SETPWM4PCR &1000 PWMPCR ! ;        // PWM PRESCALE CONTROL REGISTER for PWM4
: SETPWM5PCR &2000 PWMPCR ! ;        // PWM PRESCALE CONTROL REGISTER FOR PWM5
: SETPWM6PCR &4000 PWMPCR ! ;        // PWM PRESCALE CONTROL REGISTER FOR PWM6
: SETALLPCR &7E00 PWMPCR ! ;         // All pwm is set

: us 0 do loop ;

: delay2 t1 @ 100000000 * ms drop ;


variable grip_port1
variable grip_port2 
variable grip_hightime
variable grip_lowtime
variable grip_increment

variable gripflag

0 gripflag !

// port 5 travel from 0 to 45
// port 6 travel from 45 to 0

: ungrip


// gripflag @ 0 = if 

	5 io0-out 
        5 grip_port1 !
	6 io0-out 
	6 grip_port2 !

	// for 0 the high_time is 2500 and old_angle is 500 that is at zero


   	0 grip_port1 @ p0!
        0 grip_port2 @ p0!
   	
   	10 grip_increment !
	1 gripflag !

100 0 do


       1 grip_port1 @ p0!         
       1300  us
       0 grip_port1 @ p0!        
       18700 us

         1 grip_port2 @ p0!
	 800 us
         0 grip_port2 @ p0!
         19200  us 
	
loop


// then

;


// port 5 travel from 45 to 0
// port 6 travel from 0 to 45
// This words contain magic variables	

: grip


// gripflag @ 1 = if 

	5 io0-out 
        5 grip_port1 !
	6 io0-out 
	6 grip_port2 !

	// for 0 the high_time is 2500 and old_angle is 500 that is at zero


   	0 grip_port1 @ p0!
        0 grip_port2 @ p0!
   	0 grip_port2 @ p0!
   	10 grip_increment !
	0 gripflag !


100 0 do 
       1 grip_port1 @ p0!         
       800  us
       0 grip_port1 @ p0!        
       19200 us

         1 grip_port2 @ p0!
	 1300 us
         0 grip_port2 @ p0!
         18700  us
        
loop

// then

;


// updated the start position to 0.7ms which corresponds to the motor spec

: START

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
	1000 t1 !
	SETALLPCR

;

: ? @ . ;


: TELL

0 input_angle !
0 angle !
0 port !
2 flag !
0 error !


;


: MOTOR4

flag @  0 = if ." try the format as specified "
               
            	1 error !
            	drop
	    else 
            	SETP7PWM 
    	    	SETPWMCONTROL 
    	    	// SETPWM2PCR 	
		// RSTCNTR
                &0 SETPWMPRESET                    // NO PRESCALING
		// &59FEE SETPWMMATCH0                 // 25 MS PWERIOD
		&47FF1 SETPWMMATCH0                // 20MS PWERIOD
            	7 port ! 
	    then  
;


: MOTOR2

flag @  0 = if ." try the format as specified "
            	1 error !
            	drop
	    else 
            	SETP8PWM 
    	    	SETPWMCONTROL 
    	    	// SETPWM4PCR 	
		// RSTCNTR
                &0 SETPWMPRESET                    // NO PRESCALING
		// &59FEE SETPWMMATCH0                 // 25 MS PWERIOD
                &47FF1 SETPWMMATCH0                // 20MS PWERIOD
            	8 port ! 
	    then  
;


: MOTOR3

flag @  0 = if ." try the format as specified "
            	1 error !
            	drop
	    else 
            	SETP9PWM 
    	    	SETPWMCONTROL 
    	    	// SETPWM6PCR 	
		// RSTCNTR
                &0 SETPWMPRESET                    // NO PRESCALING
		// &59FEE SETPWMMATCH0                 // 25 MS PWERIOD
		&47FF1 SETPWMMATCH0                // 20MS PWERIOD
            	9 port ! 
	    then  
;


: MOTOR1

flag @  0 = if ." try the format as specified "
            	1 error !
            	drop
	    else 
            	SETP21PWM 
    	    	SETPWMCONTROL 
    	    	// SETPWM5PCR 	
		// RSTCNTR
                &0 SETPWMPRESET                    // NO PRESCALING
		// &59FEE SETPWMMATCH0                 // 25 MS PWERIOD
		&47FF1 SETPWMMATCH0                // 20MS PWERIOD
            	21 port ! 
	    then  
;


: SETSPEED

delay ! 


101 delay @ - delay !



;

: SERVOCONTROL

	high_time !                        
	SETPWMON                            // TURN ON PWM

	
        22117 high_time @ *  150 / temp_time !       
	
        temp_time @ loop_time !

	port @ 7 = if 
		      
                      high_time @ ms_angle1 @ > if 
                      
                      			begin
                                        22117 ms_angle1 @ *  150 / temp_time !
                                        temp_time @ SETPWMMATCH2                                                      
		      			SETPWMLER2
                                        delay @ ms
                      			ms_angle1 @ 1+ ms_angle1 !
                      			ms_angle1 @ high_time @ = until
 
					then
                        high_time @ ms_angle1 @ < if

                                        begin
                                        22117 ms_angle1 @ *  150 / temp_time !
                                        temp_time @ SETPWMMATCH2                                                      
		      			SETPWMLER2
                                        delay @  ms
                      			ms_angle1 @ 1- ms_angle1 !
                      			ms_angle1 @ high_time @ = until
                             		then      

		      else then

        port @ 8 = if 

                      high_time @ ms_angle2 @ > if 
                      
                      			begin
                                        22117 ms_angle2 @ *  150 / temp_time !
                                        temp_time @ SETPWMMATCH4                                                      
		      			SETPWMLER4
                                        delay @ ms
                      			ms_angle2 @ 1+ ms_angle2 !
                      			ms_angle2 @ high_time @ = until
 
					then
                        high_time @ ms_angle2 @ < if

                                        begin
                                        22117 ms_angle2 @ *  150 / temp_time !
                                        temp_time @ SETPWMMATCH4                                                     
		      			SETPWMLER4
                                        delay @  ms
                      			ms_angle2 @ 1- ms_angle2 !
                      			ms_angle2 @ high_time @ = until
                             		then      
                   else then
		                            
	port @ 9 = if 

			high_time @ ms_angle3 @ > if 
                      
                      			begin
                                        22117 ms_angle3 @ *  150 / temp_time !
                                        temp_time @ SETPWMMATCH6                                                     
		      			SETPWMLER6
                                        delay @ ms
                      			ms_angle3 @ 1+ ms_angle3 !
                      			ms_angle3 @ high_time @ = until
 
					then
                        high_time @ ms_angle3 @ < if

                                        begin
                                        22117 ms_angle3 @ *  150 / temp_time !
                                        temp_time @ SETPWMMATCH6                                                    
		      			SETPWMLER6
                                        delay @  ms
                      			ms_angle3 @ 1- ms_angle3 !
                      			ms_angle3 @ high_time @ = until
                                        then
                     else then                     
                      
         port @ 21 = if
                      
                      high_time @ ms_angle4 @ > if 
                      
                      			begin
                                        22117 ms_angle4 @ *  150 / temp_time !
                                        temp_time @ SETPWMMATCH5                                                     
		      			SETPWMLER5
                                        delay @ ms
                      			ms_angle4 @ 1+ ms_angle4 !
                      			ms_angle4 @ high_time @ = until 
					then
                        high_time @ ms_angle4 @ < if

                                        begin
                                        22117 ms_angle4 @ *  150 / temp_time !
                                        temp_time @ SETPWMMATCH5                                                    
		      			SETPWMLER5
                                        delay @  ms
                      			ms_angle4 @ 1- ms_angle4 !
                      			ms_angle4 @ high_time @ = until
                             		then      
		      else then         



;


: MOVETO

error @ 1 = if
            else 
		input_angle !             
		 8 input_angle @ * 9 / angle !
                 // 10 input_angle @ *  9 / angle !
                 //  15 input_angle @ * 9 /  angle !
                angle @ zero_position @ + angle !
               
     

               angle @ servocontrol
                
              port @ 21 = if input_angle @ movelog_motor1 ! 
			     angle @ ms_angle1 ! else then
              port @ 8 = if input_angle @ movelog_motor2 !
				angle @ ms_angle2 ! else then 
              port @ 9 = if input_angle @ movelog_motor3 !
				angle @ ms_angle3 ! else then       
	      port @ 7 = if input_angle @ movelog_motor4 ! 
				angle @ ms_angle4 ! else then


t1 @ ms            

  
               
then

;

variable next_angle

: MOVE

             input_angle !

              port @ 21 = if  
			     movelog_motor1 @ input_angle @ + next_angle !
                             next_angle @ 180 > if cr ." sorry not acceptable" 
                                                   cr ." angle more than 180" 
						else next_angle @ MOVETO 
						then
                         else then

                                      
              port @ 8 = if     
				movelog_motor2 @ input_angle @ + next_angle !
                             	next_angle @ 180 > if cr ." sorry not acceptable" 
                                                   cr ." angle more than 180" 
						else next_angle @ MOVETO 
						then
                         
			  else then
              port @ 9 = if 
				movelog_motor3 @ input_angle @ + next_angle !
                             	next_angle @ 180 > if cr ." sorry not acceptable" 
                                                   cr ." angle more than 180" 
						else next_angle @ MOVETO 
						then
                          else then       
	      port @ 7 = if 
				movelog_motor4 @ input_angle @ + next_angle !
                             	next_angle @ 180 > if cr ." sorry not acceptable" 
                                                   cr ." angle more than 180" 
						else next_angle @ MOVETO 
						then
                          else then    




;
 

variable plad
: PLACE
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
	dup @ tell motor1 moveto              
	dup 4 + @ tell motor2 moveto        
	dup 8 + @ tell motor3 moveto
	dup 12 + @ tell motor4 moveto
drop
;



// DC motor control for the gripper, bit tricky
// Speed variables
variable m0
500 m0 !
variable m+
1000 m+ !
variable M-
800 m- !


: us 0 do loop ;

// Second Motor control
: motorb
4 io0-out
500 0 do
   1 4 p0!
   dup us
   0 4 p0!
   2 ms
loop
drop
;

// Stop the second motor
: stopb
m0 @ motorb
0 4 p0!
;



: CALIBRATE

   TELL MOTOR1 0 MOVETO
   100 MS
   TELL MOTOR1 90 MOVETO
   100 MS
   TELL MOTOR2 90 MOVETO
   100 MS
   TELL MOTOR2 35 MOVETO
   100 MS
   TELL MOTOR3 90 MOVETO
   100 MS 
   TELL MOTOR3 140 MOVETO
   100 MS

;


: HOME

   TELL MOTOR1 90 MOVETO
   100 MS
   TELL MOTOR2 35 MOVETO
   100 MS
   TELL MOTOR3 140 MOVETO 


;




: WHERE

DECIMAL
cr ." motor1    :" movelog_motor1 @  .
cr ." motor2    :" movelog_motor2 @  .
cr ." motor3    :" movelog_motor3 @  .
cr ." motor4    :" movelog_motor4 @  .


;

: DEENERGIZE

RSTCNTR

;

: grip1

1 4 p0!

;

: ungrip1

0 4 p0!

;





: dance

tell motor2 180 moveto
tell motor3 0 moveto


;



variable a
variable b

variable sum


: add


   a !
   b !

a @ b @ + sum !
sum @ .

;

variable diff


: sub

a !
b !

 a @ b @ - diff !
diff @ .

;

variable mod

: calc

mod !
a !
b !

a @ . b @ . mod @ .

mod @ 1 = if a @ b @ add then
mod @ 2 = if a @ b @ sub then

;

variable switchval
variable flag


: firstpos

	tell motor1 180 moveto
	tell motor3 180 moveto



;

: fold

	tell motor3 90 moveto
	tell motor1 90 moveto

	cr ." YES I CAME HERE "

	tell motor3 0 moveto
	tell motor1 0 moveto


	cr ." next "
;




: foldwhile

	0 flag !
	4 2 pinselect
	4 io0-in
	
	begin 4 p0@ 0 = until
	
	FOLD
	
	begin 4 p0@ 0 = until

        FIRSTPOS

;




: HI


TELL MOTOR3 120 MOVETO
TELL MOTOR3 180 MOVETO
TELL MOTOR3 120 MOVETO
TELL MOTOR3 180 MOVETO
TELL MOTOR3 140 MOVETO


;




: SHOW_MOVE

TELL MOTOR1 120 MOVETO
TELL MOTOR1 45 MOVETO
TELL MOTOR1 90 MOVETO
TELL MOTOR2 0 MOVETO
TELL MOTOR2 120 MOVETO
TELL MOTOR3 45 MOVETO
TELL MOTOR3 180 MOVETO

;


: UP 


TELL MOTOR2 80 MOVETO



;




: PICK


TELL MOTOR1 0 MOVETO
TELL MOTOR2 140 MOVETO
GRIP

;



: DROP

TELL MOTOR1 180 MOVETO
TELL MOTOR2 130 MOVETO
TELL MOTOR3 180 MOVETO
UNGRIP


;



: DEMO


PICK
UP
DROP
CALIBRATE

;