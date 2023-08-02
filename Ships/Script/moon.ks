

//Launch Settings


//GLOBAL vesselfilename is "Saturn V - Lunar".     //this is the name of the vessel file to load
//GLOBAL vesselfilename is "Saturn IB Centaur.ks".     //this is the name of the vessel file to load
GLOBAL vesselfilename is "Shuttle-Saturn".



GLOBAL moon_transfer IS LEXICON(
								"body",BODY("moon"),
								"injection","parking",  //set this to either "parking" or "direct"
								"rel_angle", "low" ,	//set this to either "low" or "high" , only used in the "parking" case
								"apoapsis", 385000,		//measured in km
								"lead_angle", 4			//rotate the lunar antipode by this angle 
								
) . 


GLOBAL target_orbit IS LEXICON (	
								"periapsis",195,
								"apoapsis",195,
								"inclination",28.8,													
								"Longitude of Periapsis",-150 //will be set to the longitude of the lunar antipode vector at arrival
																// that corresponds to the point of TLI ignition


) . 

GLOBAL pitchheading is 180.	//heading for pitchover manoeuvre
GLOBAL pitchover is 1	.	//angle to pitch over by.

GLOBAL logdata Is true.

RUNPATH("0:/UPFG_general/src/upfg_launch").

