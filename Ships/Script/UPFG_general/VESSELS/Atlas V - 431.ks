GLOBAL vehicle IS LEXICON(
						"name","Atlas V - 431",
						"launchTimeAdvance", 300,
						"roll",180,
						"handover",LEXICON(			//CHOOSE ONLY ONE OF THE FOLLOWING OPTIONS
											//"stage",2			
											"time",110
						),
						"preburn",5.1,			//time to spool up engines at liftoff
						"stages",LIST(0,		//never remove this zero
						
									LEXICON(
											"m_initial",	482.516,
											"m_final",	228.170,
											"staging", LEXICON (
												"type","time",
												"ignition",	TRUE,
												"ullage", "none",
												"ullage_t",	0	
											),
											"Tstage",100,
											"engines",	LIST(
															LEXICON("thrust", 4152, "isp", 338.4, "minThrust", 1951.4)	//1xRD-180
											)					
									),
									
									LEXICON(
											"m_initial",	210.847,
											"m_final",	53.656,
											"staging", LEXICON (
												"type","glim",
												"ignition",	FALSE,
												"ullage", "none",
												"ullage_t",	0	
											),
											"glim",4.8,
											"engines",	LIST(
															LEXICON("thrust", 4152, "isp", 338.4, "minThrust", 1951.4)	//1xRD-180
											)					
									),
									
									LEXICON(
											"m_initial",	28.733,
											"m_final",	8.143,
											"staging", LEXICON (
												"type","depletion",
												"ignition",	TRUE,
												"ullage", "rcs",
												"ullage_t",	8	
											),
											"engines",	LIST(
															LEXICON("thrust", 99.2, "isp", 451)	//1xRL10A-4-2N
											)					
									)
									
									
								)
).
						
GLOBAL events IS LIST(
					LEXICON("time",270,"type", "action", "action", {TOGGLe AG8.})
).