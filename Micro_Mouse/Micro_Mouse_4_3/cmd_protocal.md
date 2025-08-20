# original commands format

f: move forward for length of 18 mm
r: turn right wrt. current orientation
l: turn right wrt. current orientation

full cmd: frlfrl
a simple series of f/r/l

# extended command

f<length>: move forward for <length> mm (TODO: Adjust to cm)
o<deg>: turn to the orientaion wrt. {F0}

{F0} means the world coordinate frame. In the initial state (if robot are setup in (00) heading the north) the value of orientaion is 0. positive direction: CCW

if it needs to turn right wrt. the initial state use cmd: o<90>

to connect each other put a  | at the end of each cmd

an example:

f18|o90|f18|f18|o0|


    //     {          _   _   _   _          } // row 0
    //     {      _ | _ | _ | _ | _ | _      } // row 1
    //     {  _ | _ | _ | _ | _ | _ | _ | _  } // row 2
    //     {| _ | _ | _ | _ | _ | _ | _ | _ |} // row 3
    //     {| _ | _ | _ | _ | _ | _ | _ | _ |} // row 4
    //     {| _ | _ | _ | _ | _ | _ | _ | _ |} // row 5
    //     {| _ | _ | _ | _ | _ | _ | _ | _ |} // row 6
    //     {| _ | _ | _ | _ | _ | _ | _ | _ |} // row 7
    //     {    | _ | _ | _ | _ | _ | _ |    } // row 8
    //     {        | _ | _ | _ | _ |        } 
            0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6
              0   1   2   3   4   5   6   7
              2x + 1
