// high level words loaded after kernel is built
// This file contains all of the basic code to extend
// the standard Forth words
//
// *******************************************************
// History:
// Jan 2007	Removed dotquote, now in core
//          Removed for next, added to core
// Feb 2007 Addred while / repeat
//          changed name of (loop) to (next)
// Mar 2007 Definition of create changed to match ANSI
//          altered vspace$
//          Max,Min moved to core v0.09
//          removed 0until
//          moved ms to core
// *******************************************************
//
&e01fc100 constant VPBDIV
//
// ========== Program Control ======================
// The technique here is to use a multiple if with else, the
// else is resolved normally by then but as this has several
// else's they are resolved at the end with endcase. The purpose
// of CASE is to stop endcase resolving addresses
// Used in the form:
// : x
//      case
//            1 of ." is 1" endof
//            2 of ." is 2" endof
//            otherwise ." not any"
//        endcase ;   
: case 0 ; immediate
: of
   ['] over ,
   ['] = ,
   ['] 0branch ,    // this is mosly IF
   ['] 0 ,          // filled by endof [1]
   [']  drop ,     // case
   here 8 -         // leave address of [1]
; immediate  
: endof   postpone else ; immediate
: otherwise ['] drop , ; immediate
: endcase   
      begin  ?dup while postpone then  repeat
; immediate
            
// ============= comments not included in core =======
// This will accept ( comment )
// or ( without the closing bracket
// immediate word so that it can be used within a word
: (
    >in @ 
    begin
        1+
        dup tib + c@ 
        dup 41 =        // )
        swap 13 = or     // CR
    until
    1+ >in !
; immediate                    

// ============ Strings ========================
// ----------- string input --------------
// use: <variable> input$" Name ?"
: input$"
['] lit , // compile as lit
here     // address for type
32 + ,     // offset past type etc.
['] type ,  // comile type
['] lit ,   // compile max len
80   ,       // max #
['] accept ,  
['] drop ,  // length
['] drop ,  // address
['] exit ,
,"
; immediate


// ============== Utilities ================================
// needed for many timings, PCLK should return the 
// value of the actual crystal used
: PCLK
    cclk 1000 *     // on stack for later
    vpbdiv @ 2 and // VPBDIV
    dup 0= if drop 4 then
    u/mod           // device
    swap drop       // drop remainder
; 

// ======== Other / various =================
// Tuck 4 7 20 >> 4 20 7 20
//  ( a,b,c --- a,c,b,c)
: tuck
    dup >r swap r>
;    

: soft1 -1 ; //  marker to say this is loaded

;s
  
