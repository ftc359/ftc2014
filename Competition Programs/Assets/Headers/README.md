#Gaulbots2 #359 Cascade Effect
Our source code utilized during the 2014-2015 season of Cascade Effect.
##[Competition Programs](https://github.com/ftc359/ftc2014/tree/master/Competition%20Programs)
These are the programs that we run during TeleOp and Autonomous.
##[Assets](https://github.com/ftc359/ftc2014/tree/master/Competition%20Programs/Assets)
These are reusable programs meant to ease certain tasks.

1. [**_Dual Motor Tester_**](https://github.com/ftc359/ftc2014/blob/master/Competition%20Programs/Assets/Dual%20Motor%20Tester.c)
    * _Outdated_
    * Predecessor to the HTC_Tester
2. [**_HTC_Tester_**](https://github.com/ftc359/ftc2014/blob/master/Competition%20Programs/Assets/HTC_Tester%20v1.0.c)
    * **Features**
        * Autoconfiguration based upon controllers attached
        * Motor and servo control
        * Great UI
    * **Instructions**
        * Utilize the arrow buttons to change the numbers
        * Press and release the orange button to select a port or servo controller or to switch lines
        * Press and hold the orange button to run the selected motor or servo
        * Utilize the grey exit button to repick ports or controllers and to exit
    * **Uses**
        * **General**
            * Great for testing prototypes!
            * Excellent for finding good motor speeds and servo positions before programming them into your code
            * Provides a reliable way to test if a motor or servo controller is broken - if the HTC_Tester cannot detect it, it is permanently or temporarily broken
            * Provides a reliable way to test if a motor or servo itself is broken
        * **During A Tournament**
            * Use it before a match to determine if the battery is low enough to replace, if motors are dead, if controllers are dead, etc.
            * Perfect for reseting part of your robot! - For example, a lift powered by a motor or a servo used to control a deadbolt
	* **To Be Done**
            * Add preferences which will be carried between runs via a plaintext file
3. [**_Prototyper_**](https://github.com/ftc359/ftc2014/blob/master/Competition%20Programs/Assets/Prototyper.c)
    * _Undeveloped_
    * GUI to prototype Autonomous routines
    * Routines able to be saved/reloaded utilizing plaintext files
        * **WARNING**: Will take up large amounts of space on the robot
    * _TBD next season_

##[Headers](https://github.com/ftc359/ftc2014/tree/master/Competition%20Programs/Assets/Headers)
These are reusable functions that you can utilize in your programs. To see how these are used in examples, check out our [Competition Programs](https://github.com/ftc359/ftc2014/tree/master/Competition%20Programs/Assets/Headers) page.

1. [**_Autonomous_Funcs_**](https://github.com/ftc359/ftc2014/blob/master/Competition%20Programs/Assets/Headers/Autonomous_Funcs.h)
    * _Incomplete_
    * Contains various moving functions with and without encoders
    * Contains a sophisticated parameter selector menu for various autonomous strategies that saves your last selected options in a plaintext file to reuse
        * **WARNING**: Has bugs when trying to access the variable address of every other boolean variable that causes the NXT to freeze. To avoid this, initialize all of your boolean variables in your menu with a "filler" boolean in between
2. [**_HTC_**](https://github.com/ftc359/ftc2014/blob/master/Competition%20Programs/Assets/Headers/HTC.h)
    * _Incomplete_
    * Contains the functions to access the basic properties of Hi-Technic motor and servo controllers
3. [**_Telop_Funcs_**](https://github.com/ftc359/ftc2014/blob/master/Competition%20Programs/Assets/Headers/Teleop_Funcs.h)
    * _Incomplete_
    * Contains three algorithms, including one that gives your motors exponential drive based upon a given joystick threshold and maximum speed
    * Contains a method to toggle joystick buttons (code inside of the if statement is only activated once either the button is pressed and released or pressed and then does not activate until it is released)
    * Contains incomplete functions for basic TeleOp drives
4. [**_h359_14-15_**](https://github.com/ftc359/ftc2014/blob/master/Competition%20Programs/Assets/Headers/h359_14-15.h)
    * Used specifically for our robot
    * Not recommended for you to use unless you want to build an exact replica of our robot

##Credits
* Xander Soldaat for his invaluable libraries and his help in debugging HTC.h and Autnomous_Funcs.h.
* MHTS (Titan Robotics) because I ripped off their UI style xD

##License
All programs withing this repository are free of charge, available under the [GNU General Public License v3.0](http://www.gnu.org/licenses/gpl.html).
