@LAZYGLOBAL OFF.

//Launch Settings

GLOBAL vesselfilename is "Mini SLS".     //this is the name of the vessel file to load

GLOBAL target_orbit IS LEXICON (	
								"periapsis",0,
								"apoapsis",400,
								"cutoff alt",120,
								"inclination",28.8
).

GLOBAL logdata Is true.

RUNPATH("0:/UPFG_general/src/upfg_main_executive").