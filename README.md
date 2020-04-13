# Autonomous Catamaran

The aim of this project is to implement a basic control system for an autonomous catamaran using [DsPICDEM 2 Development board](https://github.com/cesca95/AutonomousCatamaran/blob/master/docs/dsPICDEM2.pdf).

<p align="center">
  <img src="images/Overview.png" width="700">
</p>

A microcontroller board is connected to two outboard motors. Each outboard motor is composed of a DC motor and a propeller installed at the end of its shaft. Together, the two outboard motors allow the catamaran to move and rotate in the water. The microcontroller receives desired reference values for the rotation speed of the motors from a control PC, in terms of motor RPMs (rounds per minute). These
reference signals are sent through a serial interface. The microcontroller sends feedback messages back to the control PC to report a few status information.

## About the project 
### Hardware specifications

- Each motor can run from -10000 to +10000 RPMs
- The RPMs are controlled through a PWM signal
  - The frequency must be 1 kHz
  - 50% duty cycle corresponds to 0 RPM, 0% corresponds to -10000 RPM and 100% corresponds to 10000 RPMs
- The propeller installed on the shaft of each motor is rated for maximum RPMs of +-8000. Running the motor above +-8000 RPMs might damage the propeller and must be avoided

### Software requirements

##### A. Check correct functioning
- The control system should blink **led D3** at 1 Hz to signal a correct functioning of the main loop at all times, regardless of any state

##### B. Communication
- Given the chosen UART baudrate, the firmware should never lose a message due to its implementation, even with full use of the bandwidth

##### C. Different modes for the firmware

| Mode        	| Symbol | Effect  |
| :-----------: |:------:| :------|
| Controlled    | C 	 | Motors velocity are set equal to *n1* and *n2* for left motor and right one respectively |
| Time out      | T      | No references are received from the PC for more than 5 seconds so both motors velocity are set to zero |
| Safe mode 	| H      | Motors are stopped immediately and new reference signals are ignored until the microcontroller receives the enable message from the PC |

###### C.1 Control mode
- The firmware must support receiving references at least at 10 Hz frequency (**HLREF**)
- The control system must never generate PWM signals outside of the specifications of the motor and its propeller
  - If any reference value is given outside the specifications, the system should saturate it to the minimum/maximum allowed value
- The firmware must refresh the PWM value at least at 10 Hz frequency
- The user can set new minimum and maximum values through a dedicated command (**HLSAT**):
  - The firmware must check that these values are within the allowed range of the propeller
  - The firmware must check that the min, max values are correctly set (i.e., min < max)
  - The zero value should be always allowed (i.e. min <= 0 and max >= 0)
    - If the above conditions are not met, the new values are not applied, and the firmware sends a negative ack message
    - Otherwise, the new values are stored, the PWM is refreshed to comply with the new saturation values, and a positive ack is sent

###### C.2 Timeout mode
- If no references are received from the PC for more than 5 seconds, the firmware should enter a **timeout mode**:
  - Both motors velocity should be set to zero
  - **Led D4** should blink to signal timeout
  - When a new reference is read, then the led D4 should stop blinking and commands should be given again to the motor

###### C.3 Safe mode
- If button S5 is pressed, the firmware should enter a **safe mode**:
  - Motors are stopped immediately and reference signals are ignored until the microcontroller receives an enable message (**HLENA**)
  - After exiting safe mode, the motors should be set to zero. Motors should move only after receiving a new reference
  - Once an enable command is received, the firmware should send a positive ack to the PC


##### D. Display  

- The firmware should write on the LCD different information based on the current modality:
  - *First modality*:

| Row        	| Information 		| Explanation  								| 
| :-----------: |:---------------------:| :---------------------------------------------------------------------|
| First    	| **STA: x TEM: y** 	| *x* = H/T/C (halt/timeout/controlled), and *y* is the temperature 	| 
| Second      	| **RPM: n1,n2**     	| *n1* and *n2* are the applied RPM 					| 

![LCD modality 1](/images/LCD_mod1.jpg)


  - If the button S6 is pressed, the LCD should change into *second modality*:

| Row        	| Information 			 | Explanation  								| 
| :-----------: |:------------------------------:| :-------------------------------------------------------			|
| First    	| **SAT: x/y**			 | *x* and *y* are the minimum and maximum current saturation values set	| 
| Second      	| **RPM: PDC1,PDC2**    	 | *PDC1* and *PDC2* are the values of the duty cycle PWM registers		| 

![LCD modality 2](/images/LCD_mod2.jpg)

##### D. Feedback to the PC
- The firmware must acquire the temperature sensor at 10 Hz frequency and average the last 10 readings. The averaged value is sent to the PC at 1 Hz frequency with the **MCTEM** message
- The firmware must send the feedback message **MCFBK** at 5 Hz frequency
- The firmware must send a feedback message **MACK** as a response to the message received


## Getting Started
### Host system requirements

- PC-compatible system with an Intel class processor, or equivalent
- A minimum of 16 MB RAM
- A minimum fo 40 MB available hard drive space 

### Software Prerequisite

- **MPBLAB X IDE** 	*If you don't already have it on your PC you can download it [here](https://www.microchip.com/mplab/mplab-x-ide) for free*
- **MPBLAB XC16 Compiler** 	*If you don't already have it on your PC you can download it [here](https://www.microchip.com/mplab/compilers) for free*

### Set up

1. Check that the board has all the jumpers and switches in the same position of the yellow rectangles in the image below

<p align="center">
  <img src="images/dsPICDEM2.jpg" width="500">
</p>

2. Attach all the cables as depicted in the following image:

<p align="center">
  <img src="images/Connections.jpg" width="900">
</p>

3. Attach the motors in **H8** as follow:

| Motor        	| PWM pin 			|
| :-----------: |:-----------------------------:| 
| Left    	| RE1		 		| 
| Right      	| RE3    			| 

**NOTE:** For more detailed information about the connections check [dsPICDEM2_drawings.pdf](https://github.com/cesca95/AutonomousCatamaran/blob/master/docs/dsPICDEM2_drawings.pdf) file



## How to run the project

1.  Clone the repository in your computer through the command:

    ```bash
    git clone https://github.com/cesca95/AutonomousCatamaran.git
    ```
2. Create a new project in **MPBLAB X IDE** and add the files contained in **code** folder into the **src** folder of your new project
4. Upload the code on your board
3. Run the executable file *hterm.exe* 

<p align="center">
  <img src="images/hterm.jpg" width="900">
</p>

  - Change the UART baudrate from **115200** bps to **9600** bps
  - Click on **Connect** botton
  - You are going to receive these three different type of feedback messages from the board:
    - $MCFBK,n1,n2,state*
    - $MCTEM,temp*
    - $MCACK,msg_type,value*

4. To control you catamaran you can use three different type of messages: 
  - $HLREF,n1,n2*
  - $HLSAT,min,max*
  - $HLENA*

5. To simulate **safe mode** you have to press button **S5** of the board

<p align="center">
  <img src="images/Example_hterm.PNG" width="900">
</p>


### More information about messages
##### Messages from the PC to the board
| Message        			   | First parameter 	   	   	| Second parameter       		| Third parameter	|
| :---------------------------------------:|:---------------------:	   	| :---------------------:		|:---------------------:|
| <code> <b> $HLREF,n1,n2* </b> </code>   | To send the reference values  	| *n1* is the RPM for the left motor 	| *n2* is the RPM for the right motor |
| <code> <b> $HLSAT,min,max* </b> </code>  | To change the saturation values 	| *min* is the minimum RPMs allowed 	| *max* is the maximum RPMs allowed|
|  <code> <b> $HLENA* </b> </code> 	   | To exit safe mode 			| 					| 					|

##### Feedback from the board to the PC
| Message        			   	| First parameter 	   	   	| Second parameter       		| Third parameter	| Fourth parameter 
| :---------------------------------------:	|:---------------------:	   	| :---------------------:		|:---------------------:|:---------------------:|
| <code> <b> $MCFBK,n1,n2,state* </b> </code>   | Feedback to HLREF message  | *n1* is the applied reference signal for left motor| *n2* is the applied reference signal for right motor  | *state* is 2 if the microcontroller is in safe mode, 1 if it is in timeout mode, 0 otherwise |
| <code> <b> $MCTEM,temp* </b> </code>  	| Feedback to know the temperature of the board | *min* is the minimum RPMs allowed | *max* is the maximum RPMs allowed|	|
| <code> <b> $MCACK,msg\_type,value* </b> </code>| Acknowledgment feedback| *msg_type* is the command (e.g. REF, ENA, SAT)| *value* is 1 if the message was applied and 0 otherwise|	|




## Authors

* **Francesca Cantoni:** 	francescacantoni95@gmail.com
* **Kenza Boubakri:** 		kenza.boubakri@gmail.com 

