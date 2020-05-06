***************************************************************
* MECHENG 706 Project One - Group 2
* by Jake Olliff joll113@aucklanduni.ac.nz
* and Lachlan Barnes lbar461@aucklanduni.ac.nz
* and Freeman Porten fpor552@aucklanduni.ac.nz
* and Calvin Lee clee822@aucklanduni.ac.nz
*
***************************************************************
RUNNING THE CODE:
1) Before you can run any code you may need to install all the 
required libaries namely;
    - Servo V1.1.6
You can install the Servo libary using 
Arduino's library manager in Tools-> Manage Libraries - > Then using the 
search bar. This code uses the PID library of Brett Beauregard under a
GPLv3 License the Header file and .cpp file are included in this download
and thus do not need to be installed separately.

2) To open the files in an Arduino IDE, double click the main.ino file. 
Alternatively, inside of the Arduino IDE, you can go File->Open and then
from there select to open the main.ino file. Doing so will automatically
open the other files.

3) Building and uploading the files is the same for other Arduino files. Just
select 'Verify' to compile all the files or 'Upload' to compile and upload
the files to the selected port/board. Keep in mind that the correct board
must be selected when uploading, for the robots supplied the correct board
is the Arduino Mega 2560. This can be changed through Tools-> Board and
selecting the correct board from the drop down.

CHANGING THE PINS:
If the board or pin arrangement is changed, the code can be changed from
the main.ino file where all the different object's are instantiated.
For example, when the 'SonarSenor' object is instantiated it is done with
the pins of the sonar sensor, if you want to change the pins this is done
during instantiation in main.ino.