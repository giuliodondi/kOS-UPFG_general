function launch{
	CLEARSCREEN.
	
	//	Load libraries
	RUNPATH("0:/Libraries/misc_library").	
	RUNPATH("0:/Libraries/maths_library").	
	RUNPATH("0:/Libraries/navigation_library").	
	RUNPATH("0:/Libraries/vehicle_library").	
	
	RUNPATH("0:/UPFG_general/src/interface_library").
	RUNPATH("0:/UPFG_general/src/targeting_library").
	RUNPATH("0:/UPFG_general/src/upfg_library").
	RUNPATH("0:/UPFG_general/src/vehicle_library").
	
	//	Load vessel file
	IF (vesselfilename:ENDSWITH(".ks")=TRUE) {
		SET vesselfilename TO vesselfilename:REMOVE( vesselfilename:FIND(".ks") ,3 ).
	}
	RUNPATH("0:/UPFG_general/VESSELS/" + vesselfilename + ".ks").
	
	wait until ship:unpacked and ship:loaded.
	
	initialise_vehicle().
	prepare_launch().
	
	prepare_telemetry().
	
	countdown().
	open_loop_ascent().
	closed_loop_ascent().
	
}



declare function countdown{
	
	LOCK THROTTLE to throttleControl().
	SAS OFF.
	local line is 30.
	print " COUNTDOWN:" AT (0,line).
	
	//this sets the pilot throttle command to some value so that it's not zero if the program is aborted
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"].
	
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

	SET STEERINGMANAGER:ROLLCONTROLANGLERANGE TO 180.
	SET STEERINGMANAGER:ROLLTS TO 8.
	
	SET vehiclestate["ops_mode"] TO 1.
	
	drawUI().
	
	get_mass_bias().
	getState().
	
	WHEN SHIP:VERTICALSPEED > 1 THEN {
		addMessage("LIFT-OFF!").
	}
	
	//leave this here for debugging
	//until false {print vehicle["stages"].}
		
	IF (vehicle["handover"]:HASKEY("time")) {
		SET vehicle["handover"]["time"] to vehicle["ign_t"] + vehicle["handover"]["time"].
	}
	SET control["steerdir"] TO LOOKDIRUP(-SHIP:ORBIT:BODY:POSITION, SHIP:FACING:TOPVECTOR).
	
	IF (vehicle:HASKEY("pitchover") AND vehicle["pitchover"] > 0 ) {
	
		LOCAL pitchheading IS fixangle(compass_for(SHIP:FACING:TOPVECTOR,SHIP:GEOPOSITION) + 180).		
		
		LOCAL ship_topv IS SHIP:FACING:TOPVECTOR.
		
		LOCAL upv IS -SHIP:ORBIT:BODY:POSITION.
		LOCAL headv IS VXCL(upv, V(0,1,0)).
		SET headv TO rodrigues(headv, upv, pitchheading).
		LOCAL normv IS VCRS(upv, headv).
		LOCAL steerdirv IS rodrigues(upv, normv, vehicle["pitchover"]).
		
		WHEN SHIP:VERTICALSPEED > 2 THEN {
			addMessage("PERFORMING PITCHOVER MANOEUVRE.").
			SET control["steerdir"] TO LOOKDIRUP(steerdirv, ship_topv).
		}
	}
	
	local steer_flag IS false.
	
	SET control["refvec"] TO HEADING(control["launch_az"] + 180, 0):VECTOR.																		   
	
	getState().
	
	WHEN SHIP:VERTICALSPEED > 40 THEN {
		addMessage("BEGINNING ROLL PROGRAM").	
		SET steer_flag TO true.
	}

	
	LOCAL targetspeed IS pitch_program_tgt_speed().
	
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
				vehicle["handover"]:ADD("time", TIME:SECONDS + 5 ).
				vehicle["handover"]:REMOVE("stage").
			}
		}
		
		local aimVec is HEADING(
								control["launch_az"],
								pitch(SHIP:VELOCITY:SURFACE:MAG,targetspeed)
		):VECTOR.

		
		IF steer_flag AND (NOT vehiclestate["staging_in_progress"]) { 
			set control["steerdir"] TO aimAndRoll(aimVec, control["refvec"], control["roll_angle"]). 
		} ELSE {
			SET control["steerdir"] TO "kill".
		}
		
		dataViz().
		WAIT 0.
	}
	
}


declare function closed_loop_ascent{
	
	SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.2.
	
	SET vehiclestate["ops_mode"] TO 2.
	IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
		//hard-coded time shift of 5 minutes
		SET target_orbit TO tgt_j2_timefor(target_orbit,300).
	}													 
	SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	
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
				addMessage("TERMINAL GUIDANCE").
				BREAK.
			}
			
			IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
				SET target_orbit TO tgt_j2_timefor(target_orbit,upfgInternal["tgo"]).
			}															 
		}
		
		getState().
		
		SET upfgInternal TO upfg_wrapper(upfgInternal).
		
		IF NOT vehiclestate["staging_in_progress"] {
			SET control["steerdir"] TO aimAndRoll(vecYZ(usc["lastvec"]):NORMALIZED, control["refvec"], control["roll_angle"]).		
			
		} ELSE {
			SET control["steerdir"] TO "kill".
		}		
		
		SET vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"] TO usc["lastthrot"].		
		
		dataViz().
		WAIT 0.
	}
	
	SET vehiclestate["ops_mode"] TO 3.
	SET usc["terminal"] TO TRUE.
	
	//put terminal logic in its own block
	addMessage("WAITING FOR MECO").
	
	UNTIL FALSE {
		getState().
		IF (SHIP:VELOCITY:ORBIT:MAG >= target_orbit["velocity"]) {
			LOCK STEERING TO "kill".
			LOCK THROTTLE to 0.
			SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
			LIST ENGINES IN Eng.
			FOR E IN Eng {IF e:ISTYPE("engine") {E:SHUTDOWN.}}
			BREAK.
		}
		dataViz().
	}
	
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