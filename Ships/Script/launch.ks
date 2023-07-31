@LAZYGLOBAL OFF.

//Launch Settings

GLOBAL vesselfilename is "SLS block 1.ks".     //this is the name of the vessel file to load

GLOBAL target_orbit IS LEXICON (	
								"periapsis",250,
								"apoapsis",35,
								"inclination",29,
								"Cutoff Altitude",120,
								"end",0								//don't remove this
).

//GLOBAL pitchheading is 180.	//heading for pitchover manoeuvre
//GLOBAL pitchover is 1	.	//angle to pitch over by.

GLOBAL logdata Is false.

CD("0:/UPFG_latest").
run upfg_launch.
