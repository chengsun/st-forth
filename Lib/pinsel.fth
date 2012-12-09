// Pinsel.FTH
// Pinselect function words 
// APPLIES ONLY TO PORT 0
// 
// Select functions for pins and if GPIO
// specify in/out, fetch and set
//
// REQUIRES: SOFT1.FTH
//
// *******************************************************
// History:
// Mar 2007 ANSI - changed cpi to invert, 
//          Added port 1
// *******************************************************
//
//
// constants for port 0
&e002c000 constant PINSEL0
&e002c004 constant PINSEL1
&e002c014 constant PINSEL2
&e0028000 constant IO0PIN
&e0028004 constant IO0SET
&e0028008 constant IO0DIR
&e002800c constant IO0CLR
&e0028010 constant IO1PIN
&e0028014 constant IO1SET
&e0028018 constant IO1DIR
&e002801c constant IO1CLR

// move value to mask position, this is used for setting
// particular bits using a mask, e.g. if the value is
// 3 and the mask is &fff0ffff, then the value will
// be &00030000
//  ( val, mask ---- val)
: (B!)
    31 for      
        1 rshiftc
        if
            swap 1 lshift swap  // value
        else
            leave
        then
    next
    drop
;        

// bit store. This is used for clearing and settings
// bits of a word, needed for most registers
// example:
// to set bits 25:25 of pinsel1 to 2
// 2 &FCFFFFFF PINSEL0,
// any of the other bits in pinsel0 will not be affected
// but bits 25:24 will be changed to 2
// (value, mask, address ----)
: B!
    rot 2 pick      // mask, address, value, mask
    (b!)            // mask, address, new-value
    rot             // address, new-value, mask
    2 pick @        // address, new-val, mask, address-contents
    and             // address, new-val, new-contents(mask)
    or              // update contents
    swap !          // save back
;

// used as part of pinselect
// pin is a value from 0 to 15 and function
// is a value from 0 to 3, 0 is GPIO, 2 to 3
// are the alternate functions
// ( pin, function --- )
: (sel0)
    swap            // function, pin
    2 *                 // 2 bits per pin
    3 swap lshift invert  // create mask
    PINSEL0 b!          // bit store, function, mask, address
;    
// used as part of pinselect
// pin is a value from 0 to 15 and function
// is a value from 0 to 3, 0 is GPIO, 2 to 3
// are the alternate functions
// ( pin, function --- )
: (sel1)
    swap            // function, pin
    2 *                 // 2 bits per pin
    3 swap lshift invert  // create mask
    PINSEL1 b!          // bit store, function, mask, address
;    

// pin is a value from 0 to 31 and function
// is a value from 0 to 3, 0 is GPIO, 2 to 3
// are the alternate functions
// determines which pinsel to use
// Example: To select function 2 on pin 25
// 25 2 pinselect
// ( pin, function --- )
: pinselect
    swap        // PINSEL0 is 0-15, PINSEL1 is 16-31
    dup 15 > if  
        16 - swap (sel1) 
        else 
             swap (sel0)
        then
;            

// sets selected pin to GPIO output
// pin specified from 0 to 31
// sets either PINSEL0 or PINSEL1
// and then direction to o/p
// ( pin --- )
: io0-out
    dup 0 pinselect     // set pin select and GPIO
    1 swap lshift       // convert pin to mask
    IO0DIR @
    or
    IO0DIR !
;

// sets selected pin to GPIO output
// pin specified from 0 to 31
// sets either PINSEL0 or PINSEL1
// and then direction to o/p
// ( pin --- )
: io1-out
    0 PINSEL2 !      // make sure poert 1 is GPIO
    1 swap lshift       // convert pin to mask
    IO1DIR @
    or
    IO1DIR !
;

// sets selected pin to GPIO input
// pin specified from 0 to 31
// sets either PINSEL0 or PINSEL1
// and then direction to i/p
// ( pin --- )
: io0-in
    dup 0 pinselect     // set pin select
    1 swap lshift       // convert pin to mask
    invert              // complement
    IO0DIR @
    and
    IO0DIR !
;

// sets selected pin to GPIO input
// pin specified from 0 to 31
// sets either PINSEL0 or PINSEL1
// and then direction to i/p
// ( pin --- )
: io1-in
    0 PINSEL2 !      // make sure poert 1 is GPIO
    1 swap lshift       // convert pin to mask
    invert              // complement
    IO1DIR @
    and
    IO1DIR !
;

// sets pin high, must be initialised first
// by using gpio0-out
// pin is in range 0..31
// ( pin ---)    
: io0-low
    1 swap lshift       // convert pin to mask
    IO0CLR !
;

// sets pin high, must be initialised first
// by using gpio0-out
// pin is in range 0..31
// ( pin ---)    
: io1-low
    1 swap lshift       // convert pin to mask
    IO1CLR !
;

// sets pin high, must be initialised first
// by using gpio0-out
// pin is in range 0..31
//  (pin ---)
: io0-high
    1 swap lshift       // convert pin to mask
    IO0SET !
;

// sets pin high, must be initialised first
// by using gpio0-out
// pin is in range 0..31
//  (pin ---)
: io1-high
    1 swap lshift       // convert pin to mask
    IO1SET !
;

// an alternative method of changing a pin
// state, by specifying a value 1=high
// must of course be set to o/p to be any use
//  ( 1|0, pin --- )
: p0!
    swap
    if
        io0-high
    else
        io0-low    
    then
;
    
// an alternative method of changing a pin
// state, by specifying a value 1=high
// must of course be set to o/p to be any use
//  ( 1|0, pin --- )
: p1!
    swap
    if
        io1-high
    else
        io1-low    
    then
;

// input from selected pin, initialise with
// gpio0-in first, pin values 0..31
// ( pin --- t/f)    
: p0@
    1 swap lshift
    IO0PIN @
    and
    0= invert      // test for value
;
  
// input from selected pin, initialise with
// gpio0-in first, pin values 0..31
// ( pin --- t/f)    
: p1@
    1 swap lshift
    IO1PIN @
    and
    0= invert      // test for value
;

: pinsel -1 ;   // marker
     
;s
