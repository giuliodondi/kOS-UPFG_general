@LAZYGLOBAL OFF.

//Launch Settings

GLOBAL vesselfilename is "Saturn V - Lunar".     //this is the name of the vessel file to load

GLOBAL target_orbit IS LEXICON (	
								"periapsis",195,
								"apoapsis",195,
								"inclination",29,
								"Cutoff Altitude",195,
								"end",0								//don't remove this
).

GLOBAL pitchheading is 180.	//heading for pitchover manoeuvre
GLOBAL pitchover is 1	.	//angle to pitch over by.

GLOBAL logdata Is false.

RUNPATH("0:/UPFG_general/src/upfg_launch").