function launch{
	CLEARSCREEN.
	
	GLOBAL IPUset IS 600.
	GLOBAL terminalwidth IS 65.
	GLOBAL terminalheight IS 59.
		//	Required to run the script fast enough.
	//GLOBAL upfgConvergenceDelay IS 10.		//	Transition from passive (atmospheric) to active guidance occurs that many seconds before "upfgActivation" (to give UPFG time to converge).
	GLOBAL upfgFinalizationTime IS 5.		//	When time-to-go gets below that, keep attitude stable and simply count down time to cutoff.
	GLOBAL upfgConvergenceTgo IS 1.	//	Maximum difference between consecutive UPFG T-go predictions that allow accepting the solution.
	GLOBAL upfgConvergenceVec IS 15.	//	Maximum angle between guidance vectors calculated by UPFG between stages that allow accepting the solution.

	SET TERMINAL:WIDTH TO terminalwidth.
	SET TERMINAL:HEIGHT TO terminalheight.
	SET CONFIG:IPU TO IPUset.	
	
	//	Load vessel file
	IF (vesselfilename:ENDSWITH(".ks")=TRUE) {
		SET vesselfilename TO vesselfilename:REMOVE( vesselfilename:FIND(".ks") ,3 ).
	}
	RUNPATH("./VESSELS/" + vesselfilename + ".ks").
	
	PRINT " INITIALISING GLOBAL VARIABLES" AT (0,1).
	
	//	Load libraries
	RUNPATH("0:/Libraries/misc_library").	
	RUNPATH("0:/Libraries/maths_library").	
	RUNPATH("0:/Libraries/navigation_library").	
	
	RUNPATH("interface_library").
	RUNPATH("targeting_library").
	RUNPATH("vehicle_library").
	
	RUNPATH("upfg_library").	
	//RUNPATH("upfg__cser_new").	
	RUNPATH("upfg__cser_sg_simple").	
	//RUNPATH("upfg__cser_rk3_simple").	
	
	if logdata=TRUE {	
		GLOBAL loglex IS LEXICON(
										"Time",0,
										"Altitude",0,
										"Dwnrg Dst",0,
										"Stage",0,
										"Mass",0,
										"TWR",0,
										"Throt",0,
										"AZ(cmd)",0,
										"HAOA",0,
										"Pitch",0,
										"VAOA",0,
										"Surfvel",0,
										"Orbvel",0,
										"Incl",0,
										"Ecctr",0
		).
		log_data(loglex,"./LOGS/" + vesselfilename + "_log").
	}
	
	wait until ship:unpacked and ship:loaded.
	
		
	
	initialise_vehicle().
	prepare_launch().
	countdown().
	open_loop_ascent().
	closed_loop_ascent().
	
}



declare function countdown{
	
	LOCK THROTTLE to throttleControl().
	SAS OFF.
	local line is 30.
	print " COUNTDOWN:" AT (0,line).
	
	
	
	local TT IS TIME:SECONDS + 10 - vehicle["preburn"].
	WHEN  TIME:SECONDS>=TT  THEN {
			set line TO line + 2.
			print " IGNITION SEQUENCE START." at (0,line).
			stage.	
		}
	from { LOCAL i IS 10.} until i = 0 step{ SET i TO i-1.} do{
		
		set line TO line + 2.
		print " T MINUS "+ i at (0,line).
		wait 1.
	}	
	
	SET surfacestate["MET"] TO TIME:SECONDS. 
	SET vehicle["ign_t"] TO TIME:SECONDS. 
	LOCK STEERING TO control["steerdir"].
	stage.	
	wait until ship:unpacked and ship:loaded.
		
}



declare function open_loop_ascent{
	
	drawUI().
	addMessage("LIFT-OFF!").
	
	
	get_mass_bias().
	getState().
	
	//leave this here for debugging
	//until false {print vehicle["stages"].}
		
	IF (vehicle["handover"]:HASKEY("time")) {
		SET vehicle["handover"]["time"] to vehicle["ign_t"] + vehicle["handover"]["time"].
	}
	SET control["steerdir"] TO LOOKDIRUP(-SHIP:ORBIT:BODY:POSITION, SHIP:FACING:TOPVECTOR).
	
	
	IF DEFINED pitchover AND pitchover <> 0 {
		addMessage("PERFORMING PITCHOVER MANOEUVRE.").	
		LOCAL pitchheading IS compass_for(SHIP:FACING:TOPVECTOR,SHIP:GEOPOSITION).
		UNTIL FALSE {
			getState().
			local ttt is TIME:SECONDS - vehicle["ign_t"].
			IF ttt >= 2 {                                    
				SET control["steerdir"] TO up + r(arctan(sin(270-pitchheading)*tan(pitchover)),arctan(cos(270-pitchheading)*tan(pitchover)),ship:facing:roll).
			}	
			IF ttt >= 5 {
				SET control["steerdir"] TO up + r(-arctan(sin(270-pitchheading)*tan(pitchover)),-arctan(cos(270-pitchheading)*tan(pitchover)),ship:facing:roll).
				BREAK.
			}
			dataViz().
			WAIT 0.
		}
	}
	
	local steer_flag IS false.
	
	SET control["refvec"] TO HEADING(control["launch_az"] + 180, 0):VECTOR.
	LOCAL scale IS MIN(0.2,0.15*( (target_orbit["radius"]:MAG - BODY:RADIUS)/250000 - 1)).																				   
	
	SET vehiclestate["ops_mode"] TO 1.
	getState().
	
	WHEN SHIP:VERTICALSPEED > 40 THEN {
		addMessage("BEGINNING ROLL PROGRAM").	
		SET steer_flag TO true.
	}
	
	LOCAL targetspeed IS 85 + ((45 - 90)/(1.5-1))*(vehiclestate["avg_thr"]:average()/(g0*vehicle["stages"][vehiclestate["cur_stg"]]["m_initial"]) - 1).	
	
	WHEN SHIP:VERTICALSPEED > targetspeed THEN { 
		addMessage("PITCHING DOWNRANGE").
	}
	
	
	
	UNTIL FALSE {	
		getState().
		
		IF (vehicle["handover"]:HASKEY("time")) {
			IF TIME:SECONDS >= (vehicle["handover"]["time"] ) {BREAK.}
		}
		ELSE {
			IF vehiclestate["cur_stg"]=vehicle["handover"]["stage"] {
				vehicle["handover"]:ADD("time", TIME:SECONDS ).
				BREAK.
			}
		}
		
		local aimVec is HEADING(
								control["launch_az"],
								pitch(SHIP:VELOCITY:SURFACE:MAG,targetspeed,scale)
		):VECTOR.

		
		IF steer_flag AND (NOT vehiclestate["staging_in_progress"]) { 
			set control["steerdir"] TO aimAndRoll(aimVec,vehicle["roll"]). 
		} ELSE {
			SET control["steerdir"] TO "kill".
		}
		
		dataViz().
		WAIT 0.
	}
	
}


declare function closed_loop_ascent{
	
	
	SET vehiclestate["ops_mode"] TO 2.
	IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
		//hard-coded time shift of 5 minutes
		SET target_orbit TO tgt_j2_timefor(target_orbit,300).
	}													 
	SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	SET control["steerdir"] TO aimAndRoll(SHIP:FACING:FOREVECTOR:NORMALIZED,vehicle["roll"]).	
	addMessage("RUNNING UPFG ALGORITHM").
	drawUI().
	getState().
	SET target_orbit["normal"] TO targetNormal(target_orbit["inclination"], target_orbit["LAN"]).
	LOCAL x IS setupUPFG(target_orbit).
	GLOBAL upfgInternal IS x[0].
	GLOBAL usc IS x[1].
	
	SET usc["lastvec"] TO vecYZ(SHIP:FACING:FOREVECTOR).
	SET usc["lastthrot"] TO vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"].

	dataViz().

	UNTIL FALSE{
		IF usc["itercount"]=0 { //detects first pass or convergence lost
			WHEN usc["conv"]=1 THEN {
				addMessage("GUIDANCE CONVERGED IN " + usc["itercount"] + " ITERATIONS").
			}
		}														  
		
		IF (usc["terminal"]=TRUE) { 
			IF maxthrust=0 {BREAK.}
		} 
		ELSE {																					  
			IF (usc["conv"]=1 AND (upfgInternal["tgo"] < upfgFinalizationTime AND SHIP:VELOCITY:ORBIT:MAG>= 0.9*target_orbit["velocity"])) OR (SHIP:VELOCITY:ORBIT:MAG>= 0.995*target_orbit["velocity"]) {
				IF vehiclestate["ops_mode"]=2 {
					SET vehiclestate["ops_mode"] TO 3.
					SET usc["terminal"] TO TRUE.
					addMessage("WAITING FOR MECO").
					WHEN SHIP:VELOCITY:ORBIT:MAG >= target_orbit["velocity"] THEN {
						LOCK STEERING TO "kill".
						LOCK THROTTLE to 0.
						LIST ENGINES IN Eng.
						FOR E IN Eng {IF e:ISTYPE("engine") {E:SHUTDOWN.}}
					}
				}
			}
			
			IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
				SET target_orbit TO tgt_j2_timefor(target_orbit,upfgInternal["tgo"]).
			}															 
			SET target_orbit["normal"] TO targetNormal(target_orbit["inclination"], target_orbit["LAN"]).
			SET target_orbit["perivec"] TO target_perivec().
		}
		
		getState().
		
		SET upfgInternal TO upfg_wrapper(upfgInternal).
		
		IF NOT vehiclestate["staging_in_progress"] {
			SET control["steerdir"] TO aimAndRoll(vecYZ(usc["lastvec"]):NORMALIZED,vehicle["roll"]).									
		} ELSE {
			SET control["steerdir"] TO "kill".
		}		
		
		IF vehicle["stages"][vehiclestate["cur_stg"]]["mode"] <> 2 {
			SET vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"] TO usc["lastthrot"].		
		}
		dataViz().
		WAIT 0.
	}
	
	
	//put terminal logic in its own block

	
	//just in case it hasn't been done previously
	LOCK THROTTLE to 0.
	LIST ENGINES IN Eng.
	FOR E IN Eng {IF e:ISTYPE("engine") {E:SHUTDOWN.}}
	
	SET vehiclestate["ops_mode"] TO 4.
	UNLOCK THROTTLE.
	UNLOCK STEERING.
	SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
	
	SET vehiclestate["staging_in_progress"] TO TRUE.	//so that vehicle perf calculations are skipped in getState
	
	drawUI().
	UNTIL AG9 {
		getState().
		dataViz().
		WAIT 0.
	}
		
}


launch().