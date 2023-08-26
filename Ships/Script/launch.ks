@LAZYGLOBAL OFF.

//Launch Settings

GLOBAL vesselfilename is "Saturn V".     //this is the name of the vessel file to load
//GLOBAL vesselfilename is "Saturn IB".     //this is the name of the vessel file to load

GLOBAL target_orbit IS LEXICON (	
								"periapsis",175,
								"apoapsis",175,
								"inclination",28.8
).

GLOBAL pitchheading is 180.	//heading for pitchover manoeuvre
GLOBAL pitchover is 1	.	//angle to pitch over by.

GLOBAL logdata Is true.

RUNPATH("0:/UPFG_general/src/upfg_launch").