@LAZYGLOBAL OFF.

//Launch Settings

GLOBAL vesselfilename is "Atlas V - 431".     //this is the name of the vessel file to load

GLOBAL target_orbit IS LEXICON (	
								"periapsis",170,
								"apoapsis",700,
								"cutoff alt",170.5,
								"inclination",28.8
).

GLOBAL logdata Is true.

RUNPATH("0:/UPFG_general/src/upfg_main_executive").