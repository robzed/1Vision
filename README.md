# 1Vision
Code for 1Vision Micromouse Maze Solver

## Overview

This is the code for my and Alan Henness' Micromouse robot that has has vision with two tiny ARM7 
processors. It used two DC motors in a wheelchair configuration. 

The hardware suffered with a skidding problem, but the vision code seemed to work well. It contained no 
other sensors for position other than the camera and the two motor shaft encoders. 

See http://robprobin.com/pmwiki.php?n=Main.MicromouseRobot for some pictures and technical details.

## Software Architecture

The original mouse used a single ARM7 micro (NXP LPC2106) with 60K RAM. That was problematic for various reasons, so 
the design was changed to have a second camera board. CMUCAM3 had just come out, which also used a NXP LPC2106. The 
same software binary was loaded onto both boards, and the software auto-detects and configures. 

Nearly all of the code is custom written - but a few low-level hardware driver files (labelled camera_board_*) were 
used from the reference CMUCAM3 implementation - but the vision software is all custom written - and shares no code 
from CMUCam3.

The code is all C++, and all all bare metal. I added a very basic memory allocator to allow terminal commands to 
operate and non-static object construction on boot. 

There is a simple debug terminal to allow interactive testing, a bit like a Python or Forth prompt. This was usually
used via a very thin 3-wire long hanging cable while moving, although the plan was to use one of those cheap tiny 
Bluetooth to serial converter boards.

See http://robprobin.com/pmwiki.php?n=Main.BasicTechnicalSpecsFor1Vision for some hardware details.

## Hardware

Not much here, it's pretty simple but there is a presentation with part of the schematic here: 
http://www.micromouseonline.com/files/minos09/minos09-rob-probin.pdf

## Vision System Algorithm

RAM constraints (60K on each board) coupled with no direct camera interface drove some of the vision design.

Presentation which gives an overview of the principle of operation of the vision system, can be located here:
http://www.micromouseonline.com/wp/wp-content/uploads/files/minos10/minos10-probin-debugging.pdf


## License

A few files in the project (for the base CMUCam3) use Apache 2.0. 
Since this is incompatible with GPL v2, I've made the other files GPL v3. 
If you have an alternative use case, then let me know and I'll consider dual licensing under a different license.

