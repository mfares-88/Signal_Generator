#pragma once
// dsl_help.h - verified ASCII quick reference for the pattern DSL.
// Sole includer: ui_lvgl.cpp. Grammar traced against lib/dsl/
// (Lexer/Parser/Validator/Compiler) - no invented syntax.
static const char DSL_HELP_TEXT[] = R"DSLHELP(DSL QUICK REFERENCE

A pattern is 1-4 wheels joined by ':'
Each wheel:  pin , rot , kind , tail

pin   1=crank 2=cam1 3=cam2 (4=knock)
rot   C = crank, 360 deg
      c = cam,   720 deg
kind  S=symmetric  M=missing  A=angular
duty  n/d  (0<n<d, d<=32)  e.g. 1/2

TAILS
S (symmetric):  duty , teeth
   -> teeth*d slots. Each tooth =
      n slots HIGH then (d-n) LOW.
M (missing):    duty , teeth , run-list
   run-list = list of Nt / Nm,
   e.g. 58t,2m  ('t'=present teeth,
   'm'=gap teeth). Each tooth, present
   or gap, takes d slots; gap slots are
   LOW. sum(t+m)=teeth, need >=1 m.
A (angular):    deg , deg , ...
   one slot per degree, alternating
   HIGH then LOW, starting HIGH.
   C sums to 360, c sums to 720,
   each deg > 0.

EXAMPLES
Crank only (60-2 missing tooth):
  1,C,M,1/2,60,58t,2m
Crank + cam (60-2 + 1-tooth cam):
  1,C,M,1/2,60,58t,2m : 2,c,S,1/2,1

Limits: <=4 wheels, <=4096 slots,
source <=512 chars, pins unique.
)DSLHELP";
