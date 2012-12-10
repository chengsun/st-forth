// Decompile & Words
//

// this must be held as a constant for it not to become
// confused with the proper exit

// -------- decompile ------
// simply emits tab or spaces to align
// columns
: tab1
    9 emit
;    
// -------- decompile ------
// ( ---- )
// checks for exit signature and aborts if found
: dc1   // ( sig --- [sig][abort]) checks termination
dup
exitaddress    // terminating signature
-
0= if
    ph. tab1 ." Exit"
    abort 
then ;

// -------- decompile ------
// fetches a char from the address and if printable
// prints it else prints a dot
: dc2a  // ( addr --- [dot or ASCII])
  dup ?c@ 32 122 between
  if 
    ?c@ emit 
  else 
    drop 46 emit    // 46 is dot 
  then
;    

// -------- decompile ------
// prints out 7 characters from given address
: dc2   // ( addr ---) just o/p 8 char
7 0 do
    dup dc2a
    1+
loop 
drop   
;
// -------- decompile ------
// given the cfa, prints the address followed by the 
// text word
: dc3  // (cfa --- ) prints word or number
dup ph. tab1 // print address
dup 12 - ?@ &f00 and &900 =   // test for valid header
if          // print header
    8 - // move to name field 
    dc2 
else 
    ?@ ph.      // just print contents
then 
cr
;

// -------- decompile ------
: decompile   // use decompile xxx
cr
." Addr" tab1  // tab
." Wrd-Adr" tab1
." Name" tab1
cr
'     // get cfa of word
dup u. tab1 // print address of word to decompile
dup 8 - stype   // and its name
dup 12 - @ 16 rshift tab1 ." Hash = " ph. cr  // hash
begin
  4 +   // advance to next word
  dup u. tab1  // print address
  dup ?@     // fetch CFA of next word
  dc1       // abort when exit found
  dc3       // print word or number
again  
;

// name field address
//  ( lfa --- nfa )
: nfa
    8 +
;

// type will return &2000 or 0
//  ( lfa --- type)
: wtype
    4 + @ &2000 and
;
// immediate will return $8000 or 0
// (lfa --- imed)
: wimed
    4 + @ &8000 and
;                

: whead
    cr
    ." Name    Hash    Type    Immediate" cr
;
// outputs one line of text
//  (lfa ---)
: wline
    dup nfa stype space // name
    dup 4 + @ &ffff0000 and 16 rshift <# # # # # #> stype 4 spaces
    dup wtype if ." Code    " else ." Forth   " then
    wimed if ." Immediate" then  
;        

: words
    whead
    latest        // LFA of latest
    begin
        @ dup 0= if drop escape then   // no more
        dup wline cr 
    again
;      

// prints out hash of a word
: hash
    bl word toupper rshash &0000ffff and ph. 
;        


;s

