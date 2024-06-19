//Global vars

GLOBAL g0 IS 9.80665. 
GLOBAL vehicle_countdown IS 10.
GLOBAL vehicle_pre_staging_t IS 5.


GLOBAL vehiclestate IS LEXICON(
	"ops_mode",0,
	"cur_stg", 1,
	"staging_time", 0,
	"staging_in_progress", FALSE,
	"m_burn_left", 0,
	"avg_thr", average_value_factory(6)
).



GLOBAL events IS LIST().


//VEHICLE INITIALISATION FUNCTION 

FUNCTION initialise_vehicle{
	
	FUNCTION fix_mass_params {
		parameter stg.
		
		IF stg:HASKEY("m_burn") {
			IF stg:HASKEY("m_initial") {
				IF NOT stg:HASKEY("m_final") {
					stg:ADD("m_final",0).
					SET stg["m_final"] TO stg["m_initial"] - stg["m_burn"].
				}
			}
			ELSE IF stg:HASKEY("m_final") {
				IF NOT stg:HASKEY("m_initial") {
					stg:ADD("m_initial",0).
					SET stg["m_initial"] TO stg["m_final"] + stg["m_burn"].
				}
			}
		}
		ELSE IF stg:HASKEY("m_initial") AND stg:HASKEY("m_final") {
			IF NOT stg:HASKEY("m_burn") {
				stg:ADD("m_burn",0).
				SET stg["m_burn"] TO stg["m_initial"] - stg["m_final"].
			}
		}
		ELSE {
			PRINT ("ERROR! VEHICLE MASS PARAMETERS ILL-DEFINED") AT (1,40).
			LOCAL X IS 1/0.
		}
		
	}
	

	LOCAL vehlen IS vehicle["stages"]:LENGTH.

	FROM {LOCAL k IS 1.} UNTIL k > (vehlen - 1) STEP { SET k TO k+1.} DO{
	
		local stg IS vehicle["stages"][k].
	
	
		LOCAL iisspp IS 0.
		LOCAL tthrust IS 0.
		LOCAL minthrust IS 0.
		local can_throttle IS FALSE.
		LOCAL fflow IS 0.
		
		IF (stg["engines"]:ISTYPE("LIST")) {
			//trigger that we should do the initial stage parsing 
			FOR v in stg["engines"] {
				SET tthrust TO tthrust + v["thrust"].
				SET iisspp TO iisspp + v["isp"]*v["thrust"].
				
				IF (v:HASKEY("flow")) {
					SET fflow TO fflow + v["flow"].
				} ELSE {
					SET fflow TO fflow + v["thrust"] * 1000 / (v["isp"] * g0).
				}
				
				IF (v:HASKEY("minThrust")) {
					SET can_throttle TO TRUE.
					SET minthrust TO minthrust + v["minThrust"].
				} else {
					SET minthrust TO minthrust + v["thrust"].
				}
			}
			
			SET stg["engines"] TO LEXICON("thrust", tthrust, "isp", iisspp/tthrust, "flow", fflow).
			
			if (can_throttle) {
				stg["engines"]:ADD("minThrottle", minthrust/tthrust).
			}
			
			fix_mass_params(stg).
		
			SET stg["m_initial"] 			TO stg["m_initial"]*1000.
			SET stg["m_final"] 			TO stg["m_final"]*1000.
			SET stg["m_burn"] 				TO stg["m_burn"]*1000.
			SET stg["engines"]["thrust"] 	TO stg["engines"]["thrust"]*1000.
		}
		
		IF NOT (stg:HASKEY("Throttle")) {stg:ADD("Throttle",1).}
	
		IF NOT (stg:HASKEY("Tstage")) {stg:ADD("Tstage",0).}
		//stg:ADD("ign_t", 0).
		
				
		//SET stg["m_initial"] TO stg["m_initial"] + m_bias.
		//SET stg["m_final"] TO stg["m_final"] + m_bias.

		IF NOT stg:HASKEY("mode") {	
			stg:ADD("mode", 1).	
		}
		
		IF stg["staging"]["type"]="time" {
			SET stg["Tstage"] TO stg["Tstage"].
		} ELSE IF (stg["staging"]["type"]="m_burn") {
			SET stg["Tstage"] TO  const_f_t(stg).
		
		} ELSE IF stg["staging"]["type"]="glim" {
		
			//will the stage exceed the g limit?
			LOCAL x IS glim_t_m(stg).

			
			If (x[0] >= 0) {
				//yes it will exceed the limit

				IF stg["engines"]:HASKEY("minThrottle") {
					//the stage will be followed by a constant-accel stage which has
					//to be created from scratch
					
					LOCAL new_stg  IS LEXICON(
												"m_initial",	x[1],
												"m_final",	stg["m_final"],
												"m_burn", x[1] - stg["m_final"],
												"staging", LEXICON (
															"stg_action",{},
															"type","depletion",
															"ignition",	FALSE,
															"ullage", "none",
															"ullage_t",	0
												),
												"engines",	stg["engines"],
												"Tstage",0,
												"mode", 2,
												"glim",stg["glim"],
												"Throttle", 1
										).
					vehicle["stages"]:INSERT(k+1, new_stg).
					SET vehlen TO vehicle["stages"]:LENGTH.
				}
				ELSE {
					//the stage will be followed by a different stage in the sequence
					//which is however already present and only needs different mass parameters
					
					local nextstg IS vehicle["stages"][k+1].
					
					fix_mass_params(nextstg).
					
					SET nextstg["m_initial"] TO x[1]/1000.
					SET nextstg["m_final"] TO stg["m_final"]/1000.
					SET nextstg["m_burn"] TO nextstg["m_initial"] - nextstg["m_final"].
					SET nextstg["staging"]["type"] TO "depletion".
				
				}	
				
				SET stg["mode"] TO 1.
				SET stg["Tstage"] TO x[0].
				SET stg["m_final"] TO x[1].
				SET stg["m_burn"] TO stg["m_initial"] - x[1].
			
			}
			ELSE {
				//no it will never exceed the limit.
				//convert it to a depletion stage.
				
				SET stg["staging"]["type"] TO "depletion".
			}
		
		} ELSE IF (stg["staging"]["type"]="depletion") {
		
			IF (stg["mode"] = 1) {
				//regular constant thrust depletion stage 
				SET stg["Tstage"] TO  const_f_t(stg).
			} ELSE IF (stg["mode"] = 2) {
				//g-limited stage, figure out if we need to add a minimum constant throttle depletion stage
				
				LOCAL y IS const_G_t_m(stg).
				SET stg["Tstage"] TO y[0].
				
				LOCAL newstg_m_initial IS y[1].
				LOCAL newstg_m_final IS stg["m_final"].
				
				//only add a new stage if it burns for at least 3 seconds 
				LOCAL min_newstg_m_initial IS newstg_m_final + 3 * stg["engines"]["minThrottle"] * stg["engines"]["flow"].
				
				//print "newstg_m_initial " + newstg_m_initial/1000 at (0,29).
				//print "min_newstg_m_initial " + min_newstg_m_initial/1000 at (0,30).
				//print "newstg_m_final " + newstg_m_final/1000 at (0,31).
				
				IF (newstg_m_initial > min_newstg_m_initial) {
					
					SET stg["m_final"] TO newstg_m_initial.
					SET stg["m_burn"] TO stg["m_initial"] - newstg_m_initial.
					SET stg["staging"]["type"] TO "minthrot".
					SET stg["staging"]["stg_action"] TO {}.
					
					//create the new stage 
					
					LOCAL new_stg  IS LEXICON(
												"m_initial",	newstg_m_initial,
												"m_final",	newstg_m_final,
												"m_burn", newstg_m_initial - newstg_m_final,
												"staging", LEXICON (
															"stg_action",{},
															"type","depletion",
															"ignition",	FALSE,
															"ullage", "none",
															"ullage_t",	0
												),
												"engines",	stg["engines"],
												"Tstage",0,
												"mode", 1,
												"glim",stg["glim"],
												"Throttle",stg["engines"]["minThrottle"]
										).
										
					vehicle["stages"]:INSERT(k+1, new_stg).
					SET vehlen TO vehicle["stages"]:LENGTH.
				}
			}
		}
		
		
		
	}
	SET vehiclestate["m_burn_left"] TO vehicle["stages"][1]["m_burn"].
	
	vehicle:ADD("ign_t", 0).
	vehicle:ADD("max_q_reached", FALSE).
	vehicle:ADD("low_level", FALSE).
	
	vehicle:ADD("traj_steepness", 0.9).	//placeholder
	
	set_staging_trigger().
	prepare_events_trigger().
	
	//debug_vehicle().
}

FUNCTION debug_vehicle {
	
	dump_vehicle().
	
	until false{
		print "vehicle debug, press crtl-c to quit" at (0,2).
		wait 0.1.
	}
}


FUNCTION dump_vehicle {
	IF EXISTS("0:/vehicledump.txt") {
		DELETEPATH("0:/vehicledump.txt").
	}
	
	log vehicle:dump() to "0:/vehicledump.txt".
}




									//CONTROL FUNCTIONS

FUNCTION set_vehicle_traj_steepness {
	PARAMETER cut_alt.
	SET vehicle["traj_steepness"] TO (cut_alt/220)^0.5.
}


FUNCTION pitch_program_tgt_speed {

	local stg IS get_stage().
	
	LOCAL twr IS stg["engines"]["thrust"]/stg["m_initial"].
	return CLAMP(85 + ((45 - 90)/(1.5-1))*(twr - 1), 85, 50).	

}


//open-loop pitch profile for pre-UPFG
FUNCTION open_loop_pitch {
	PARAMETER v_.
	PARAMETER v0.			 
	
	LOCAL refv IS 400.
	
	LOCAL steep_fac IS vehicle["traj_steepness"].
	
	IF v_>v0 {
		
		LOCAL p1 IS -0.0048.
		LOCAL p2 IS 28.8.
		LOCAL p3 IS 26300.
		LOCAL q1 IS 3.923.
		
		LOCAL x IS v_ + refv - v0.
	
		LOCAL out IS CLAMP((p1*x^2 + p2*x + p3)/(x + q1), 0, 90).
		
		SET out TO out + (steep_fac - 1)*(90 - out)^0.7.
		
		LOCAL bias IS out - surfacestate["vdir"].
		
		RETURN CLAMP(out + 0.8*bias,0,90).
		
		
	} else {
		RETURN 90.
	}


	RETURN MAX(0,MIN(default,out)).

}








function ascent_dap_factory {

	SET STEERINGMANAGER:ROLLCONTROLANGLERANGE TO 180.
	SET STEERINGMANAGER:ROLLTS TO 8.

	LOCAL this IS lexicon().
	
	this:add("steer_mode", "").
	this:add("thr_mode", "").		//ksp 0-1 throttle value
	
	this:add("thr_cmd", 1).
	
	this:add("last_time", TIME:SECONDS).
	
	this:add("iteration_dt", 0).
	
	this:add("update_time",{
		LOCAL old_t IS this:last_time.
		SET this:last_time TO TIME:SECONDS.
		SET this:iteration_dt TO this:last_time - old_t.
	}).
	
	this:add("cur_dir", SHIP:FACINg).
	this:add("cur_thrvec", v(0,0,0)).
	
	this:add("cur_steer_pitch", 0).
	this:add("cur_steer_az", 0).
	this:add("cur_steer_roll", 0).
	
	this:add("steer_pitch_delta", 0).
	this:add("steer_yaw_delta", 0).
	this:add("steer_roll_delta", 0).
	
	this:add("steer_dir", SHIP:FACINg).
	
	this:add("measure_refv_roll", {
		LOCAL refv IS VXCL(this:steer_thrvec, this:steer_refv):NORMALIZED.
		LOCAL topv IS VXCL(this:steer_thrvec, this:cur_dir:TOPVECTOR):NORMALIZED.
		
		
		set this:cur_steer_roll to signed_angle(refv, topv, this:steer_thrvec, 1).
	}).
	
	this:add("measure_cur_state", {
		this:update_time().
		
		set this:cur_dir to ship:facing.
		set this:cur_thrvec to thrust_vec().
		
		set this:cur_steer_az to get_az_lvlh(this:steer_dir).
		set this:cur_steer_pitch to get_pitch_lvlh(this:steer_dir).
		
		this:measure_refv_roll().
		
		local tgtv_h is vxcl(this:steer_dir:topvector, this:steer_tgtdir:forevector):normalized.
		local tgtv_v is vxcl(this:steer_dir:starvector, this:steer_tgtdir:forevector):normalized.
		local tgttv_p is vxcl(this:steer_dir:forevector, this:steer_tgtdir:topvector):normalized.
		
		
		set this:steer_pitch_delta to signed_angle(tgtv_v, this:steer_dir:forevector, this:steer_dir:starvector, 0).
		set this:steer_yaw_delta to -signed_angle(tgtv_h, this:steer_dir:forevector, this:steer_dir:topvector, 0).
		set this:steer_roll_delta to signed_angle(tgttv_p, this:steer_dir:topvector, this:steer_dir:forevector, 0).
	}).
	
	
	
	this:add("max_steervec_corr", 5).
	this:add("steer_refv", SHIP:FACINg:topvector).
	this:add("steer_thrvec", SHIP:FACINg:forevector).
	this:add("steer_roll", 0).
	this:add("steer_cmd_roll", 0).
	
	this:add("steer_tgtdir", SHIP:FACINg).
	
	

	
	this:add("set_steer_tgt", {
		parameter new_thrvec.
		
		set this:steer_thrvec to new_thrvec.
		
		//required for continuous pilot input across several funcion calls
		LOCAL time_gain IS ABS(this:iteration_dt/0.2).
		
		local max_roll_corr is 13 * time_gain * STEERINGMANAGER:MAXSTOPPINGTIME.
		
		local roll_delta is unfixangle(this:cur_steer_roll - this:steer_roll).
		set roll_delta to sign(roll_delta) * min(abs(roll_delta) ,max_roll_corr).
		
		set this:steer_cmd_roll to this:cur_steer_roll - roll_delta.
		
		set this:steer_tgtdir to aimAndRoll(this:steer_thrvec, this:steer_refv, this:steer_cmd_roll).
	}).
	
	this:add("steer_auto_thrvec", {
		set this:steer_mode to "auto_thrvec".
		
		this:measure_cur_state().
	
		local steer_err_tol is 0.5.
	
		local max_roll_corr is 20.
		
		local cur_steervec is this:cur_dir:forevector.
		local tgt_steervec is this:steer_tgtdir:forevector.
		
		local steer_err is vang(cur_steervec, tgt_steervec).
		
		if (steer_err > steer_err_tol) {
			local steerv_norm is vcrs(cur_steervec, tgt_steervec).
			local steerv_corr is min(this:max_steervec_corr, steer_err).
			
			set tgt_steervec to rodrigues(cur_steervec, steerv_norm, steerv_corr).
		} else {
			set tgt_steervec to tgt_steervec.
		}
		
		local cur_topvec is vxcl(tgt_steervec, this:cur_dir:topvector).
		local tgt_topvec is vxcl(tgt_steervec, this:steer_tgtdir:topvector).
		
		//local roll_err is signed_angle(tgt_topvec, cur_topvec, tgt_steervec, 0).
		//local roll_corr is sign(roll_err) * min(abs(roll_err) ,max_roll_corr).
		//
		//print "roll_corr " + roll_corr + " " at (0,20).
		//
		//set tgt_topvec to rodrigues(cur_topvec, tgt_steervec, -roll_corr).
	
		set this:steer_dir to LOOKDIRUP(tgt_steervec, tgt_topvec ).
	}).
	
	
	this:add("thr_tgt", 0).
	this:add("thr_max", 1).	
	this:add("thr_min", 0).	
	
	this:add("update_thr_cmd", {
		
	}).
	
	this:add("thr_control_auto", {
		set this:thr_mode to "thr_auto".
	
		local new_thr is CLAMP(this:thr_tgt, this:thr_min, this:thr_max).
		set this:thr_cmd to throtteValueConverter(new_thr, this:thr_min).
	}).
	
	
	this:add("print_debug",{
		PARAMETER line.
		
		print "steer_mode : " + this:steer_mode + "    " at (0,line).
		print "thr_mode : " + this:thr_mode + "    " at (0,line + 1).
		
		print "loop dt : " + round(this:iteration_dt(),3) + "    " at (0,line + 3).
		
		print "cur_steer_pitch : " + round(this:cur_steer_pitch,3) + "    " at (0,line + 5).
		print "cur_steer_roll : " + round(this:cur_steer_roll,3) + "    " at (0,line + 6).
		print "steer_cmd_roll : " + round(this:steer_cmd_roll,3) + "    " at (0,line + 7).
		print "thr_tgt : " + round(this:thr_tgt,3) + "    " at (0,line + 8).
		print "thr lims : [" + round(this:thr_min,3) + "," + round(this:thr_max,3) +  "]    " at (0,line + 9).
		print "thr_cmd : " + round(this:thr_cmd,3) + "    " at (0,line + 10).
		
		print "steer_pitch_delta : " + round(this:steer_pitch_delta,3) + "    " at (0,line + 11).
		print "steer_roll_delta : " + round(this:steer_roll_delta,3) + "    " at (0,line + 12).
		print "steer_yaw_delta : " + round(this:steer_yaw_delta,3) + "    " at (0,line + 13).
		
	}).
	
	this:add("set_steering_ramp", {
		local max_steer is 2.
		local steer_ramp_rate is max_steer/5.
		
		SET STEERINGMANAGER:MAXSTOPPINGTIME TO min(max_steer, STEERINGMANAGER:MAXSTOPPINGTIME + steer_ramp_rate * this:iteration_dt).
	}).
	
	this:add("set_steering_high", {
		SET STEERINGMANAGER:MAXSTOPPINGTIME TO 2.
	}).
	
	this:add("set_steering_low", {
		SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.1.
	}).
	
	this:add("set_rcs", {
		PARAMETER on_.
		
		if (on_) {
			RCS ON.
		} else {
			RCS OFF.
		}
	}).
	
	
	
	this:measure_cur_state().

	return this.

}












									//VEHICLE PERFORMANCE & STAGING FUNCTIONS

// LEGACY , no longer used
//simple function to check if vehicle is past maxq
FUNCTION check_maxq {
	PARAMETER newq.
	
	IF (newq >=  surfacestate["q"] ) {
		SET surfacestate["q"] TO newq.
	} ELSE {
		addMessage("VEHICLE HAS REACHED MAX-Q").
		surfacestate:REMOVE("q").
	}
}


FUNCTION get_stage {
	RETURN vehicle["stages"][vehiclestate["cur_stg"]].
}

function prepare_events_trigger {

	for e_ in events {
		IF (NOT e_:HASKEY("triggered")) {
			e_:add("triggered", FALSE).
		}
	}
}

FUNCTION events_handler {
	
	local rem_list IS LIST().
	
	for e_ in events {
		if (NOT e_["triggered"]) {	
		
			IF (surfacestate["MET"] > e_["time"])  {
				IF e_["type"]="jettison" {
					TOGGLE AG8.
					IF e_:HASKEY("mass") {
						FROM { LOCAL i IS j. } UNTIL i > (vehicle["stages"]:LENGTH - 1)  STEP { SET i TO i+1. } DO {
							SET vehicle["stages"][i]["m_initial"] TO vehicle["stages"][i]["m_initial"] - e_["mass"].
							SET vehicle["stages"][i]["m_final"] TO vehicle["stages"][i]["m_final"] - e_["mass"].
						}
					}
					set e_["triggered"] to TRUE.
				}
				ELSE IF e_["type"]="roll" {
					
					set vehicle["roll"] to e_["angle"].
					dap:set_steering_high().
					set e_["triggered"] to TRUE.
					
					local tnext is surfacestate["MET"] + 60.
					
					WHEN(surfacestate["MET"] > tnext) THEN {
						dap:set_steering_low().
					}
				}

				ELSE IF e_["type"]="action" { 
					IF e_:HASKEY("action") {
						e_["action"]:call().
					}
					set e_["triggered"] to TRUE.
				}
			}
		}
	}
}

FUNCTION get_mass_bias {

	LOCAL stg IS vehicle["stages"][1].
		
	IF NOT stg:HASKEY("tankparts") {get_stg_tanks_res(stg).}
	local res_left IS get_prop_mass(stg).

	local dm IS vehiclestate["m_burn_left"] - res_left.
	
	local m_bias IS SHIP:MASS*1000.
	SET m_bias TO m_bias - stg["m_initial"] + dm.
	IF m_bias<0.5 {
		set m_bias to 0.
	}

	FROM {LOCAL k IS 1.} UNTIL k > (vehicle["stages"]:LENGTH - 1) STEP { SET k TO k+1.} DO{	
		local stg IS vehicle["stages"][k].
		SET stg["m_initial"] TO stg["m_initial"] + m_bias.
		SET stg["m_final"] TO stg["m_final"] + m_bias.
	}

}



FUNCTION get_TWR {
	RETURN vehiclestate["avg_thr"]:average()/(1000*SHIP:MASS*g0).
}



//measures everything about the current state of the vehicle, including instantaneous thrust
//thrust only averaged over if staging is not in progress
FUNCTION getState {
	
	update_navigation().
	
	IF DEFINED events {	events_handler().}
	
	//measure and compute vehicle performance parameters
	
	local stg IS get_stage().
	
	LOCAL x IS get_current_thrust_isp().
	
	SET vehiclestate["thr_vec"] TO x[0].
	
	vehiclestate["avg_thr"]:update(vehiclestate["thr_vec"]:MAG).
	
	LOCAL avg_thrust is vehiclestate["avg_thr"]:average().
	LOCAL avg_isp is x[1].
		
	LOCAL m_old IS stg["m_initial"].

	SET stg["m_initial"] TO SHIP:MASS*1000.
	
	LOCAL deltam IS m_old - stg["m_initial"].
	
	IF NOT (vehiclestate["staging_in_progress"]) {
	
		IF NOT stg:HASKEY("tankparts") {get_stg_tanks_res(stg).}
		local res_left IS get_prop_mass(stg).
		SET vehiclestate["m_burn_left"] to res_left.
								 
		IF (stg["staging"]["type"]="m_burn") {
		
			SET stg["m_burn"] TO stg["m_burn"] - deltam.
			SET stg["m_final"] TO stg["m_initial"] -  stg["m_burn"].
			
		} ELSE IF (stg["staging"]["type"]="time") {
		
		    	SET stg["Tstage"] TO stg["Tstage"] - surfacestate["deltat"].
				
		} ELSE IF (stg["staging"]["type"]="glim"){	
			
			SET stg["m_final"] TO stg["m_initial"] - res_left.
			
			LOCAL y IS glim_t_m(stg).
			
			SET stg["Tstage"] TO y[0].
			SET stg["m_final"] TO y[1].
			SET stg["m_burn"] TO stg["m_initial"] - y[1].
			
			//LOCAL nextstg IS vehicle["stages"][vehiclestate["cur_stg"]+1].
			//
			//SET nextstg["m_initial"] TO y[1].
			//
			//IF nextstg["mode"]=1 {
			//	SET nextstg["Tstage"] TO const_f_t(nextstg).
			//	SET nextstg["m_burn"] TO res_left - stg["m_burn"].
			//}
			//ELSE IF nextstg["mode"]=2 {
			//	
			//	SET nextstg["m_final"] TO z[1].
			//	
			//	LOCAL z IS const_G_t_m(nextstg).
			//	
			//	SET nextstg["Tstage"] TO z[0].
			//	SET nextstg["m_final"] TO z[1].
			//	SET nextstg["m_final"] TO z[1].
			//}
			
		}  ELSE IF (stg["mode"]=2){
			//both depletion and minthrot stages
			
			SET stg["m_final"] TO stg["m_initial"] - res_left.
			LOCAL y IS const_G_t_m(stg).
			SET stg["Tstage"] TO y[0].
			SET stg["m_final"] TO y[1].
			SET stg["m_burn"] TO stg["m_initial"] - y[1].
			
			//LOCAL nextstg IS vehicle["stages"][vehiclestate["cur_stg"]+1].
			//
			//SET nextstg["m_initial"] TO y[1].
			//SET nextstg["Tstage"] TO const_f_t(nextstg).
			
		}ELSE IF (stg["mode"]=1 AND stg["staging"]["type"]="depletion") {

			SET stg["m_burn"] TO res_left.
			SET stg["m_final"] TO stg["m_initial"] -  res_left.
			SET stg["Tstage"] TO const_f_t(stg).
			 
		}
	}	
	
	if (debug_mode) {
		dump_vehicle().
	}
}





//Staging function.
FUNCTION STAGING{

	LOCAL depletion_ is (vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["type"]="depletion").
	
	IF depletion_ {
		SET vehiclestate["staging_in_progress"] TO TRUE.
	}
	
	addMessage("CLOSE TO STAGING").
	SET vehiclestate["staging_time"] TO surfacestate["time"]+100.		//bias of 100 seconds to avoid premature triggering of the staging actions
	

	WHEN (depletion_ AND engine_flameout()) or ((NOT depletion_) AND vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <=0.01 ) THEN {	
		SET vehiclestate["staging_in_progress"] TO TRUE.
		addMessage("STAGING").
		SET vehiclestate["staging_time"] TO surfacestate["time"].
		IF (vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["type"]="depletion") {set vehiclestate["staging_time"] to vehiclestate["staging_time"] + 1.5 .}
	}
	
	
	local stagingaction IS {STAGE.}.
	IF vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]:HASKEY("stg_action") {
		SET stagingaction TO vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]["stg_action"].
	}
	
	WHEN (surfacestate["time"] > vehiclestate["staging_time"]) THEN {
		staging_reset(stagingaction).	
	}
	
	IF vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]["ignition"]=TRUE {
		WHEN (surfacestate["time"] > (vehiclestate["staging_time"] + vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["ullage_t"])) THEN { 
			wait until stage:ready.
			
			STAGE.
			
			addMessage("SPOOLING UP").
			WHEN vehiclestate["avg_thr"]["latest_value"]>= 0.95*vehicle["stages"][vehiclestate["cur_stg"]]["engines"]["thrust"] THEN {
				addMessage("STAGING SEQUENCE COMPLETE").
				SET vehiclestate["staging_in_progress"] TO FALSE.
			}
		}
	}
	ELSE {
		WHEN (surfacestate["time"] > vehiclestate["staging_time"] + 0.5) THEN {
			addMessage("STAGING SEQUENCE COMPLETE").
			SET vehiclestate["staging_in_progress"] TO FALSE.
		}
	}
}


FUNCTION staging_reset {

	FUNCTION handle_ullage {
		PARAMETER stg.
	
		IF stg["staging"]["ullage"]="none" OR stg["staging"]["ullage_t"]=0 {
			RETURN.
		}
		addMessage("ULLAGE THRUST").
		IF stg["staging"]["ullage"]="srb"{
			RETURN.
		}
		ELSE IF stg["staging"]["ullage"]="rcs"{
			RCS ON. 
			SET SHIP:CONTROL:FORE TO 1.0.
			WHEN TIME:SECONDS > (vehiclestate["staging_time"] + stg["staging"]["ullage_t"]+1) THEN {
				SET SHIP:CONTROL:FORE TO 0.0.
			}
		}
	}

	PARAMETER stagingaction.
	wait until stage:ready.
	
	stagingaction:call().
		
	SET vehiclestate["cur_stg"] TO vehiclestate["cur_stg"]+1.
	local stg is get_stage().
	SET stg["ign_t"] TO surfacestate["time"].
	
	vehiclestate["avg_thr"]:reset().
	
	SET vehiclestate["m_burn_left"] TO stg["m_burn"].
	handle_ullage(stg).
	set_staging_trigger().
	//this is necessary to reset throttle if we have g-throttling before another stage
	set upfgInternal["throtset"] to stg["Throttle"].
}

FUNCTION set_staging_trigger {

	WHEN (vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <= vehicle_pre_staging_t) THEN {
		if (vehiclestate["cur_stg"]< (vehicle["stages"]:LENGTH - 1)) {
			STAGING().
		} else {
			set vehicle["low_level"] to TRUE.
		}
	}
}



//small function that, given time, computes how much burned mass that equates to
//used for pre-convergence of upfg
FUNCTION decrease_mass {
	parameter stg.
	parameter dt.
	
	local dm is 0.
	
	IF stg["mode"]=1 {
		set dm to dt*stg["engines"]["flow"].
	}
	ELSE IF stg["mode"]=2 {
		set dm to stg["m_initial"]*(1 - CONSTANT:E^(-stg["glim"]*dt/stg["engines"]["isp"])).
	}
	
	set stg["m_initial"] to stg["m_initial"] - dm.
	set stg["m_burn"] to stg["m_burn"] - dm.
	SET stg["Tstage"] TO stg["Tstage"] - dt.
} 
	
	
	
FUNCTION shutdown_all_engines {
	LIST ENGINES IN all_eng.
	FOR e IN all_eng {
		e:shutdown.
	}	
}

