
// Code for controlling the motors using PWM signals Version 1.2
// ************Geena Chacko,Applications Engineer,ST Robotics********************

// #LIBRARY "SOFT1.FTH"
// #LIBRARY "PINSEL.FTH"
// #LIBRARY "DECOMPILE.FTH"


variable high_time
variable ms_angle1
variable ms_angle2
variable ms_angle3
variable ms_angle4
70 constant zero_position
21 constant delay
100 constant t1



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


5 constant grip_port1
6 constant grip_port2

// port 5 travel from 0 to 45
// port 6 travel from 45 to 0

: ungrip


	grip_port1 io0-out 
	grip_port2 io0-out

   	0 grip_port1 p0!
    0 grip_port2 p0!

100 0 do


       1 grip_port1 p0!         
       1300  us
       0 grip_port1 p0!        
       18700 us

       1 grip_port2 p0!
	   800 us
       0 grip_port2 p0!
       19200  us 
	
loop


;


// port 5 travel from 45 to 0
// port 6 travel from 0 to 45
// This words contain magic variables	

: grip

	grip_port1 io0-out
	grip_port2 io0-out

   	0 grip_port1 p0!
    0 grip_port2 p0!


100 0 do 
       1 grip_port1 p0!         
       800  us
       0 grip_port1 p0!        
       19200 us

       1 grip_port2 p0!
	   1300 us
       0 grip_port2 p0!
       18700  us
        
loop

;


// updated the start position to 0.7ms which corresponds to the motor spec

: START
	0 ms_angle1 !
	0 ms_angle2 !
	0 ms_angle3 !
	0 ms_angle4 !
	SETALLPCR

;

: TELL ;


: MOTOR4

            	SETP7PWM 
    	    	SETPWMCONTROL 
    	    	// SETPWM2PCR 	
		// RSTCNTR
                &0 SETPWMPRESET                    // NO PRESCALING
		// &59FEE SETPWMMATCH0                 // 25 MS PWERIOD
		&47FF1 SETPWMMATCH0                // 20MS PWERIOD
            	7
;


: MOTOR2

            	SETP8PWM 
    	    	SETPWMCONTROL 
    	    	// SETPWM4PCR 	
		// RSTCNTR
                &0 SETPWMPRESET                    // NO PRESCALING
		// &59FEE SETPWMMATCH0                 // 25 MS PWERIOD
                &47FF1 SETPWMMATCH0                // 20MS PWERIOD
            	8
;


: MOTOR3

            	SETP9PWM 
    	    	SETPWMCONTROL 
    	    	// SETPWM6PCR 	
		// RSTCNTR
                &0 SETPWMPRESET                    // NO PRESCALING
		// &59FEE SETPWMMATCH0                 // 25 MS PWERIOD
		&47FF1 SETPWMMATCH0                // 20MS PWERIOD
            	9
;


: MOTOR1

            	SETP21PWM 
    	    	SETPWMCONTROL 
    	    	// SETPWM5PCR 	
		// RSTCNTR
                &0 SETPWMPRESET                    // NO PRESCALING
		// &59FEE SETPWMMATCH0                 // 25 MS PWERIOD
		&47FF1 SETPWMMATCH0                // 20MS PWERIOD
            	21
;

: SERVOCONTROL ( port ms_angle -- )

	high_time !                        
	SETPWMON                            // TURN ON PWM


	dup 7 = if 
		      
                      high_time @ ms_angle1 @ > if 
                      
                      			begin
                                        22117 ms_angle1 @ *  150 / SETPWMMATCH2                                                      
		      			SETPWMLER2
                                        delay ms
                      			ms_angle1 @ 1+ dup ms_angle1 !
                      			high_time @ = until
 
					then
                        high_time @ ms_angle1 @ < if

                                        begin
                                        22117 ms_angle1 @ *  150 / SETPWMMATCH2                                                      
		      			SETPWMLER2
                                        delay  ms
                      			ms_angle1 @ 1- dup ms_angle1 !
                      			high_time @ = until
                             		then      
		      then

    dup 8 = if 

                      high_time @ ms_angle2 @ > if 
                      
                      			begin
                                        22117 ms_angle2 @ *  150 / SETPWMMATCH4                                                      
		      			SETPWMLER4
                                        delay ms
                      			ms_angle2 @ 1+ dup ms_angle2 !
                      			high_time @ = until
 
					then
                        high_time @ ms_angle2 @ < if

                                        begin
                                        22117 ms_angle2 @ *  150 / SETPWMMATCH4                                                     
		      			SETPWMLER4
                                        delay  ms
                      			ms_angle2 @ 1- dup ms_angle2 !
                      			high_time @ = until
                             		then      
                   then
		                            
	dup 9 = if 

			high_time @ ms_angle3 @ > if 
                      
                      			begin
                                        22117 ms_angle3 @ *  150 / SETPWMMATCH6                                                     
		      			SETPWMLER6
                                        delay ms
                      			ms_angle3 @ 1+ dup ms_angle3 !
                      			high_time @ = until
 
					then
                        high_time @ ms_angle3 @ < if

                                        begin
                                        22117 ms_angle3 @ *  150 / SETPWMMATCH6                                                    
		      			SETPWMLER6
                                        delay  ms
                      			ms_angle3 @ 1- dup ms_angle3 !
                      			high_time @ = until
                                        then
                     then                     
                      
    21 = if
                      
                      high_time @ ms_angle4 @ > if 
                      
                      			begin
                                        22117 ms_angle4 @ *  150 / SETPWMMATCH5                                                     
		      			SETPWMLER5
                                        delay ms
                      			ms_angle4 @ 1+ dup ms_angle4 !
                      			high_time @ = until 
					then
                        high_time @ ms_angle4 @ < if

                                        begin
                                        22117 ms_angle4 @ *  150 / SETPWMMATCH5                                                    
		      			SETPWMLER5
                                        delay  ms
                      			ms_angle4 @ 1- dup ms_angle4 !
                      			high_time @ = until
                             		then      
		      then         



;


: MOVETO ( port angle -- )
            
     8 * 9 / zero_position +
   servocontrol
    t1 ms
;
