@LAZYGLOBAL OFF.

//Launch Settings

GLOBAL vesselfilename is "Saturn C10".     //this is the name of the vessel file to load

// 	moon_transfer
GLOBAL moon_transfer IS LEXICON(
								"body",BODY("moon"),
								"injection","parking",  //set this to either "parking" or "direct", recommend to leave it at parking
								"tgt_site", "Apollo 15"	//set this to "north" or "south"
) . 

GLOBAL target_orbit IS LEXICON (	
								"periapsis",200,
								"apoapsis",200,
								"inclination",29.5
) . 

GLOBAL logdata Is true.

RUNPATH("0:/UPFG_general/src/upfg_main_executive").

