Coding Conventions

File Structure:

* All files will be sorted into the following folders:  
  * autos:  
    * All autos files  
  * pedro:  
    * All files for tuning and pedro pathing  
  * subsystems:  
    * A file for each subsystem containing hardware mapping and methods involved  
  * teleops:  
    * All teleops files  
  * utils:  
    * Any other files we use  
      * Ex. A file to store information across auto and teleop

Naming Conventions:

* Final variables are upper case with underscores (Make sure to make final variables. There shouldn’t be raw numbers in the code)  
  * (FINAL\_VARIABLE)  
* Classes and files are named using PascalCase  
  * (ThisIsPascalCase)  
* Folders are lowercase and with underscores  
  * (folder)  
* Variables and methods are named using camelCase  
  * (thisIsCamelCase)  
* Hardware Mapping (camelCasing):  
  * Drive Motors:  
    * FLDrive  
    * FRDrive  
    * BLDrive  
    * BRDrive  
  * Other Motors:  
    * A 1-2 word description, followed by the word “motor”   
      * Ex. shooterMotor  
  * Servos:  
    * A 1-2 word description, followed by the word “servo” camel case  
      * Ex. flickerServo  
  * Sensors:  
    * Type of sensor, followed by the word “sensor,” followed by a number (if multiple)  
      * Ex. colorSensor1  
  * IMU:  
    * imu  
  * Odometry Computer:  
    * pinpoint  
* Op Modes:  
  * Auto naming should be in PascalCase. It will say the starting position then what it is meant to accomplish. Names might have to get more complex as we add more autos.  
    * Ex. Garden2Tip  
    * The possible starting positions could be Hive, Garden, AllianceArea, Loading, Zone, CloseFlower, FarFlower

Comments  
If there are more than 2 lines of comments in a row they should be put into block comments. 

* Classes:  
  * A brief description of what the class does  
* Methods: (in order)  
  * What the method does  
  * What do all of the parameters represent  
  * If it returns the value, what does the value represent  
* Variables:  
  * If what it does isn’t clear by the name, add a description  
  * If it has unit, add it in a comment  
  * If any math was done to get the number, include the math or where you got it from  
* Math:  
  * Anywhere that has complex math should have thorough comments (extremely important for portfolio)  
* Changes:  
  * Whenever you make changes in the code make sure that the comments still make sense with what the code is doing  
  * If you are changing important numbers include the previous value in a comment  
* Auto:   
  * Each action in an auto should be described by a comment  
  * At the top of the class have a description of the intended path