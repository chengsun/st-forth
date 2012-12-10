\ code that is used on the actual robot

: 2VARIABLE CREATE 2 CELLS ALLOT ;

: DEBUG ( e-addr ulen -- )
    TYPE
;




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









: PENDOWN
    \ TODO
;

: PENUP
    \ TODO
;
