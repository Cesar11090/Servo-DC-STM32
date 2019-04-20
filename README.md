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
      
 4-Software control, GRBL on arduino UNO or Mach 3.
  
 
 FILES:
 
 DC Servo Eclipse: Is the file used to program Blue Pill. I Use Eclipse compiler, if you use this only need to modify the PID values according your aplication.
 
 Is a precompiled firmware Hex file on DEBUG Folder.
 
 DC Servo Eclipse / New Folder: Proteus schematic, is not for simulation because Proteus can't simulate interrups for PWM signals and encoder.

 
THEORY OF OPERATION:
 
The code is generated on Eclipse, use STM32  CubeMX to generate the basic code.

The blue pill, used:
- timer2 to read directly from encoder, A and B signals, 5 Volts tolerant pins.
- timer4 B9 and B8 to generate PWM signals for High side of the bridge. Frecuency aprox. 2kHz.
- B6 and B7 as output to control low side of the bridge.
- B12,B13 and B14 as input. Step, Dir and Enable signals.

The H Bridge board.
- 20Vdc input.
- LM7805 to reduce the input voltaje to +5V to Blue Pill.
- 74HC14 use to conect the blue pill to the Mosfet N Channel.
- J380 P channel mosfet used for high leg, H Bridge. Use an N channel mosfet to drive the gate.
- P30NS15LFP N channel mosfet used for low leg, H Bridge. Use 74HC14 to drive the gate.
- 334K Capacitor, to filter the PWM.
- S3L56  Diode for recovery diode on H bridge.
- A nice diode to show when the power supply is working ( I use a Laptop, this is short circuit protected and 3 Amps limited)

The algoritm is based on error and PID control, the control try to make the error 0. If the error is not 0 the PWM move the motor to reduce the error to 0. You can change PID variables to match your project.

The step, dir signals, add error on positive or negative values, causing the motor run on CW or CCW direction. 

The enable signal force 0 PWM and stop motor on brake mode.

Important: You need to verify the Servo is on position (Green LED on Blue Pill) as the same time the pulses from controller is off. If no the motor will be running because the cummulative pulses is not cero (The pulses is faster than motor. In my case to match the pulses the maximum velocity is 1500 rpm @20VDC, I think need to increase voltaje to increase velocity, but I don't need to test)

Thanks.

 
 
 
 
