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
	
	GLOBAL dap IS ascent_dap_factory().
	
	GLOBAL dap_gui_executor IS loop_executor_factory(
												0.15,
												{
													//protection
													if (SAS) {
														SAS OFF.
													}
													
													local cur_stg is get_stage().
													
													set dap:thr_max to 1.
													set dap:thr_min to cur_stg["minThrottle"].
													
													dap:steer_auto_thrvec().
													dap:thr_control_auto().
													
													set cur_stg["Throttle"] to dap:thr_tgt.
													
													if (dap_debug) {
														//clearscreen.
														clearvecdraws().
														
														dap:print_debug(2).
														
														arrow_ship(3 * dap:steer_thrvec,"steer_thrvec").
														arrow_ship(2 * dap:steer_dir:forevector,"forevec").
														arrow_ship(2 * dap:steer_dir:topvector,"topvec").
													
													}
													
													dataViz().
												}
	).
	
	drawUI().
	
	countdown().
	open_loop_ascent().
	closed_loop_ascent().
	
}



declare function countdown{

//needed to update the dap at least once
	wait 0.
	
	SAS OFF.

	set dap:steer_refv to HEADING(target_orbit["launch_az"] + 180, 0):VECTOR.	
	dap:set_steering_low().
	set dap:steer_roll to vehicle["roll"].
	
	local TT IS TIME:SECONDS + 10 - vehicle["preburn"].
	WHEN  TIME:SECONDS>=TT  THEN {
			set line TO line + 2.
			addMessage("IGNITION SEQUENCE START").
			stage.	
		}
		
	addMessage("T MINUS 10").
	
	
	UNTIL (	TIME:SECONDS >= vehicle["ign_t"] ) {
		SET SHIP:CONTROL:PILOTMAINTHROTTLE TO dap:thr_cmd.
		wait 0.1.
	}
	
	SET surfacestate["MET"] TO TIME:SECONDS. 
	SET vehicle["ign_t"] TO TIME:SECONDS. 
	IF (vehicle["handover"]:HASKEY("time")) {
		SET vehicle["handover"]["time"] to vehicle["ign_t"] + vehicle["handover"]["time"].
	}
	
	LOCK STEERING TO dap:steer_dir.
	
	stage.	
	
	until false {
		wait 0.	
		if (SHIP:VERTICALSPEED > 1) {break.}
	}
	
	addMessage("LIFT-OFF CONFIRMED").
	
	SET vehiclestate["ops_mode"] TO 1.
	wait 0.
		
}



declare function open_loop_ascent{

	get_mass_bias().
	getState().

	local steer_flag IS false.																	   
	
	WHEN (surfacestate["vs"] > 40) THEN {
		addMessage("BEGINNING ROLL PROGRAM").	
		SET steer_flag TO true.
	}
	
	LOCAL targetspeed IS pitch_program_tgt_speed().
	
	WHEN (surfacestate["vs"] > targetspeed) THEN { 
		addMessage("PITCHING DOWNRANGE").
	}
	
	
	UNTIL FALSE {	
		getState().
		
		IF (vehicle["handover"]:HASKEY("time")) {
			IF (surfacestate["time"] >= vehicle["handover"]["time"]) {BREAK.}
		}
		ELSE {
			IF vehiclestate["cur_stg"]=vehicle["handover"]["stage"] {
				vehicle["handover"]:ADD("time", surfacestate["time"] + 5 ).
				vehicle["handover"]:REMOVE("stage").
			}
		}
		
		if (NOT vehicle["max_q_reached"]) {
			IF (surfacestate["vs"] <= 100 ) or (surfacestate["q"] >=  surfacestate["maxq"] ) {
				SET surfacestate["maxq"] TO surfacestate["q"].
				set vehicle["max_q_reached"] to FALSE.
			} else {
				addMessage("VEHICLE HAS REACHED MAX-Q").
				set vehicle["max_q_reached"] to TRUE.
			}
		}
		
		local aimVec is HEADING(target_orbit["launch_az"],open_loop_pitch(SHIP:VELOCITY:SURFACE:MAG)):VECTOR.

		
		IF steer_flag AND (NOT vehiclestate["staging_in_progress"]) { 
			dap:set_steering_ramp().
			dap:set_steer_tgt(aimVec).
		}
		
	}
	
}


declare function closed_loop_ascent{
	
	SET vehiclestate["ops_mode"] TO 2.
	
	dap:set_steering_low().
	
	set dap:steer_refv to -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	
	SET target_orbit["normal"] TO upfg_normal(target_orbit["inclination"], target_orbit["LAN"]).
	
	
	addMessage("RUNNING UPFG ALGORITHM").
	
	drawUI().
	getState().

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