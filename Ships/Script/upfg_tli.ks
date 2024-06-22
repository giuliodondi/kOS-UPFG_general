@LAZYGLOBAL OFF.

//Launch Settings

GLOBAL vesselfilename is "Saturn C10".     //this is the name of the vessel file to load

// 	moon_transfer
GLOBAL moon_transfer IS LEXICON(
								"body",BODY("moon"),
								"injection","parking",  //set this to either "parking" or "direct", recommend to leave it at parking
								"tgt_site_latitude", "north",	//set this to "north" or "south"
								"tgt_site_hemisphere", "east"	//set this to "east" or "west", as seen from the near side of the moon					
) . 

GLOBAL target_orbit IS LEXICON (	
								"periapsis",200,
								"apoapsis",200,
								"inclination",29.5
) . 

GLOBAL logdata Is true.

RUNPATH("0:/UPFG_general/src/upfg_main_executive").

