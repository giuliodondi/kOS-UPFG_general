@LAZYGLOBAL OFF.
CLEARSCREEN.
SET CONFIG:IPU TO 1200.	
global debug_mode is false.
global dap_debug is false.

//	Load libraries
RUNPATH("0:/Libraries/misc_library").	
RUNPATH("0:/Libraries/maths_library").	
RUNPATH("0:/Libraries/navigation_library").	
RUNPATH("0:/Libraries/vehicle_library").	
RUNPATH("0:/Libraries/aerosim_library").	

RUNPATH("0:/UPFG_general/src/upfg_interface_library").
RUNPATH("0:/UPFG_general/src/upfg_targeting_library").
RUNPATH("0:/UPFG_general/src/upfg_upfg_library").
RUNPATH("0:/UPFG_general/src/upfg_vehicle_library").

//	Load vessel file
IF (vesselfilename:ENDSWITH(".ks")=TRUE) {
	SET vesselfilename TO vesselfilename:REMOVE( vesselfilename:FIND(".ks") ,3 ).
}
RUNPATH("0:/UPFG_general/VESSELS/" + vesselfilename + ".ks").

wait until ship:unpacked and ship:loaded.


function upfg_main_exec{
	CLEARSCREEN.
	
	
	
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
													if (cur_stg["engines"]:HASKEY("minThrottle")) {
														set dap:thr_min to cur_stg["engines"]["minThrottle"].
													} else {
														set dap:thr_min to 0.
													}
													set dap:steer_roll to vehicle["roll"].
													dap:steer_auto_thrvec().
													dap:thr_control_auto().
													
													set cur_stg["Throttle"] to dap:thr_tgt.
													
													if (dap_debug) {
														clearscreen.
														clearvecdraws().
														
														dap:print_debug(2).
														
														arrow_ship(3 * dap:steer_thrvec,"steer_thrvec").
														arrow_ship(2 * dap:steer_dir:forevector,"forevec").
														arrow_ship(2 * dap:steer_dir:topvector,"topvec").
													
													} else {
														dataViz().
													}
													
													
												}
	).
	
	drawUI().
	
	countdown().
	open_loop_ascent().
	closed_loop_ascent().
	post_meco_actions().
	
}



declare function countdown{

//needed to update the dap at least once
	wait 0.
	
	SAS OFF.

	SET dap:thr_tgt TO 1.
	set dap:steer_refv to HEADING(target_orbit["launch_az"] + 180, 0):VECTOR.	
	dap:set_steering_low().
	LOCK THROTTLE to dap:thr_cmd.
	
	local TT IS TIME:SECONDS + 10 - vehicle["preburn"].
	WHEN  TIME:SECONDS>=TT  THEN {
			addMessage("IGNITION SEQUENCE START").
			stage.	
		}
		
	addMessage("T MINUS 10").
	
	
	UNTIL (	TIME:SECONDS >= vehicle["ign_t"] ) {
		SET SHIP:CONTROL:PILOTMAINTHROTTLE TO dap:thr_cmd.
		wait 0.1.
	}
	
	LOCK STEERING TO dap:steer_dir.
	
	stage.	
	SET surfacestate["time"] TO TIME:SECONDS. 
	SET vehicle["ign_t"] TO TIME:SECONDS. 
	IF (vehicle["handover"]:HASKEY("time")) {
		SET vehicle["handover"]["time"] to vehicle["ign_t"] + vehicle["handover"]["time"].
	}
	
	until false {
		wait 0.	
		if (SHIP:VERTICALSPEED > 1) {break.}
	}
	
	addMessage("LIFT-OFF CONFIRMED").
	
	SET vehiclestate["ops_mode"] TO 1.
	wait 0.
		
}



function open_loop_ascent{

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
		
		if (NOT vehicle["max_q_reached"]) AND (surfacestate["vs"] > 100) {
			IF (surfacestate["q"] >=  surfacestate["maxq"] ) {
				SET surfacestate["maxq"] TO surfacestate["q"].
				set vehicle["max_q_reached"] to FALSE.
			} else {
				addGUIMessage("VEHICLE HAS REACHED MAX-Q").
				set vehicle["max_q_reached"] to TRUE.
			}
		}
		
		local aimVec is HEADING(target_orbit["launch_az"],open_loop_pitch(SHIP:VELOCITY:SURFACE:MAG, targetspeed)):VECTOR.

		
		IF steer_flag AND (NOT vehiclestate["staging_in_progress"]) { 
			dap:set_steering_ramp().
			dap:set_steer_tgt(aimVec).
		}
		
	}
	
}


function closed_loop_ascent{
	
	SET vehiclestate["ops_mode"] TO 2.
	
	dap:set_steering_low().
	
	set dap:steer_refv to -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	
	SET target_orbit["normal"] TO upfg_normal(target_orbit["inclination"], target_orbit["LAN"]).
	
	
	addMessage("RUNNING UPFG ALGORITHM").
	
	setupUPFG().
	
	drawUI().
	getState().

	UNTIL FALSE{
		
		getState().
		
		if (vehicle["low_level"]) {
			addMessage("LOW LEVEL").
			BREAK.
		}
		
		if (upfgInternal["s_meco"]) {
			addMessage("TERMINAL GUIDANCE").
			BREAK.
		}	
		
		IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
			tgt_j2_timefor(target_orbit, upfgInternal["tgo"]).
		}
		
		upfg_sense_current_state(upfgInternal).
		
		upfg(
			vehicle["stages"]:SUBLIST(vehiclestate["cur_stg"],vehicle:LENGTH-vehiclestate["cur_stg"]),
			target_orbit,
			upfgInternal
		).
		
		if (NOT vehiclestate["staging_in_progress"]) {
			set dap:thr_tgt to upfgInternal["throtset"].	
		
			IF (upfgInternal["s_conv"]) {
				dap:set_steer_tgt(vecYZ(upfgInternal["steering"])).
			}
		}
		
		
		if (debug_mode) {
			clearvecdraws().
			arrow_ship(vecyz(upfgInternal["steering"]),"steer").
			arrow_ship(vecyz(upfgInternal["ix"]),"ix").
			arrow_ship(vecyz(upfgInternal["iy"]),"iy").
			arrow_ship(vecyz(upfgInternal["iz"]),"iz").
			
			arrow_body(vecyz(vxcl(upfgInternal["iy"], upfgInternal["r_cur"])),"r_proj").
			arrow_body(vecyz(upfgInternal["rd"]),"rd").
			arrow_body(vecyz(target_orbit["normal"]),"rd").
		}
	}
	
	SET vehiclestate["ops_mode"] TO 3.
	
	//put terminal logic in its own block
	addMessage("WAITING FOR MECO").
	
	UNTIL FALSE {
		getState().
		IF (SHIP:VELOCITY:ORBIT:MAG >= target_orbit["velocity"] OR engine_flameout()) {
			BREAK.
		}
	}
	
	//just in case it hasn't been done previously
	LOCK THROTTLE to 0.
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
	
	shutdown_all_engines().
}

function post_meco_actions {
	SET vehiclestate["staging_in_progress"] TO TRUE.	//so that vehicle perf calculations are skipped in getState
	
	SET vehiclestate["ops_mode"] TO 4.
	UNLOCK THROTTLE.
	UNLOCK STEERING.
	SET SHIP:CONTROL:NEUTRALIZE TO TRUE.

	drawUI().
	print_ascent_report().
	getState().
	dataViz().
	
	WAIT 5.
	
	dap_gui_executor["stop_execution"]().
	
	clearscreen.
	clearvecdraws().
}

upfg_main_exec().