# Servo-DC-STM32
Build a basic Servo motor DC with Encoder and Blue pill, Step, Dir, Enable

Hello Guys, This is my first repository.

I will try to share, all the files needed to build a DC Servo with the Step/Dir/Enable inputs as comercial Steeper Driver.

For test I use the components below:
1- DC Servo Motor PITMAN  with 500CPR 38Volts
     The DC conection is the same as all DC Motors.
     The encoder, red for +5V, Black for ground, White and Green A,B cuadrature signals. It doesn't have Z signal.
     
2- Home Made Board with and H bridge mosfet, any comercial H bridge board will work. 

3- Blue Pill, with a basic DC servo control algoritm based on PID control.
   - Timer2 to use as encoder input.
   - Timer4 to use as PWM signals for H bridge.
   - B12, B13, B14 used as inputs for step, dir, enable signals.
   
 4-Software control, GRBL on arduino UNO or Mach 3.
  
 
 FILES:
 
 DC Servo Eclipse: Is the file used to program Blue Pill. I Use Eclipse compiler, if you use this only need to modify the PID values according your aplication.
 
 Is a precompiled firmware Hex file on DEBUG Folder.
 
 
 
 
