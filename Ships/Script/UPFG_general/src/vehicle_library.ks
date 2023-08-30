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


GLOBAL control Is LEXICON(
	"launch_az",0,
	"steerdir", LOOKDIRUP(SHIP:FACING:FOREVECTOR, SHIP:FACING:TOPVECTOR),
	"roll_angle",0,
	"refvec", v(0,0,0)
).



GLOBAL events IS LIST().


//VEHICLE INITIALISATION FUNCTION 

declare function initialise_vehicle{

	RUNPATH("0:/Libraries/resources_library").	

	
	FUNCTION add_resource {
		parameter lexx.
		parameter reslist.
		
		
		for res in reslist {
			IF NOT lexx:HASKEY(res) {lexx:ADD(res,0).}
		}
		RETURN lexx.
	}
	
	
	
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
	
	
	
	
	//local m_bias IS ship:mass.
	//SET m_bias TO m_bias - vehicle["stages"][1]["m_initial"].
	//IF m_bias<0.5 {
	
	//set m_bias to 0.
	//}

	//IF NOT vehicle:HASKEY("offaxis_thrust") {vehicle:ADD("offaxis_thrust",v(0,0,0)).}

	LOCAL vehlen IS vehicle["stages"]:LENGTH.

	FROM {LOCAL k IS 1.} UNTIL k > (vehlen - 1) STEP { SET k TO k+1.} DO{
	
		local stg IS vehicle["stages"][k].
	
	
		LOCAL iisspp IS 0.
		LOCAL tthrust IS 0.
		LOCAL fflow IS 0.
		local stage_res IS LEXICON().
		
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
				
				
				SET stage_res TO add_resource(stage_res,v["resources"]).
			}
			SET stg["engines"] TO LEXICON("thrust", tthrust, "isp", iisspp/tthrust, "flow", fflow).
			
			SET stage_res TO res_dens_init(stage_res).
		
			stg:ADD("resources",stage_res).
			
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

			
			If x[0] > 0 {
				//yes it will exceed the limit

				IF stg:HASKEY("minThrottle") {
					//the stage will be followed by a constant-accel stage which has
					//to be created from scratch
			
					//	In case user accidentally entered throttle as percentage instead of a fraction
					IF stg["minThrottle"] > 1.0	{ SET stg["minThrottle"] TO stg["minThrottle"]/100. }
					
					
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
												"resources",stg["resources"],
												"Tstage",0,
												"mode", 2,
												"glim",stg["glim"],
												"Throttle",stg["minThrottle"],
												"minThrottle",stg["minThrottle"],
												"throt_mult",0
										).
					
					SET new_stg["throt_mult"] TO new_stg["glim"]*g0/new_stg["engines"]["thrust"].
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
				LOCAL min_newstg_m_initial IS newstg_m_final + 3 * stg["minThrottle"] * stg["engines"]["flow"].
				
				print "newstg_m_initial " + newstg_m_initial/1000 at (0,29).
				print "min_newstg_m_initial " + min_newstg_m_initial/1000 at (0,30).
				print "newstg_m_final " + newstg_m_final/1000 at (0,31).
				
				IF (newstg_m_initial > min_newstg_m_initial) {
					//new stage will have a new type minThrottle
					
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
												"resources",stg["resources"],
												"Tstage",0,
												"mode", 1,
												"glim",stg["glim"],
												"Throttle",stg["minThrottle"],
												"minThrottle",stg["minThrottle"]
										).
										
					vehicle["stages"]:INSERT(k+1, new_stg).
					SET vehlen TO vehicle["stages"]:LENGTH.
				}
			}
		}
		
		
		
	}
	SET vehiclestate["m_burn_left"] TO vehicle["stages"][1]["m_burn"].
	
	vehicle:ADD("ign_t", 0).
	
	vehicle:ADD("traj_steepness", 0.9).	//placeholder

	SET control["roll_angle"] TO vehicle["roll"].
	
	set_staging_trigger().
	
	PRINT " INITIALISATION COMPLETE" AT (0,3).
	
	//debug_vehicle().
}


FUNCTION debug_vehicle {
	IF EXISTS("0:/vehicledump.txt") {
		DELETEPATH("0:/vehicledump.txt").
	}
	
	log vehicle:dump() to "0:/vehicledump.txt".
	
	until false{wait 0.1.}
}




									//CONTROL FUNCTIONS

FUNCTION set_vehicle_traj_steepness {
	PARAMETER cut_alt.
	SET vehicle["traj_steepness"] TO (cut_alt/220)^0.5.
}


//open-loop pitch profile for pre-UPFG
FUNCTION pitch {
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








//	Throttle controller
FUNCTION throttleControl {

	local stg IS get_stage().
	local throtval is stg["Throttle"].
	
	IF stg["mode"] = 2   {
		SET throtval TO stg["throt_mult"]*SHIP:MASS*1000.
		SET usc["lastthrot"] TO throtval.
	}
	
	set stg["Throttle"] to throtval.

	LOCAL minthrot IS 0.
	IF stg:HASKEY("minThrottle") {
		SET minthrot TO stg["minThrottle"].
	}
	
	RETURN throtteValueConverter(throtval, minthrot).
}


//control individually the throttle value of engines 
//using the nametag system and adjusting the throttle limit 

FUNCTION engthrottle {
	PARAMETER tagg.
	PARAMETER throtval.
	
	LOCAL engtaglist IS  SHIP:PARTSTAGGED(tagg).
	
	FOR eng IN engtaglist {
		IF eng:ISTYPE("engine") {
			SET eng:THRUSTLIMIT to throtval.
		}
	}


}












									//VEHICLE PERFORMANCE & STAGING FUNCTIONS

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


FUNCTION events_handler {

	local met is TIME:SECONDS - vehicle["ign_t"].

	local x IS events:LENGTH.
	
	local rem_list IS LIST().

	FROM {LOCAL k IS 0.} UNTIL k >= x STEP { SET k TO k+1.} DO{
		
		IF met>events[k]["time"]  {
			IF events[k]["type"]="jettison" {
				TOGGLE AG8.
				IF events[k]:HASKEY("mass") {
					FROM { LOCAL i IS j. } UNTIL i > (vehicle["stages"]:LENGTH - 1)  STEP { SET i TO i+1. } DO {
						SET vehicle["stages"][i]["m_initial"] TO vehicle["stages"][i]["m_initial"] - events[k]["mass"].
						SET vehicle["stages"][i]["m_final"] TO vehicle["stages"][i]["m_final"] - events[k]["mass"].
					}
				}
				rem_list:ADD(k).
				SET x TO x-1.
				SEt k TO k-1.
			}
			ELSE IF events[k]["type"]="roll" {
				//SET STEERINGMANAGER:MAXSTOPPINGTIME TO 0.4.
								
				IF ABS(events[k]["angle"] - vehicle["roll"])<5 {
					set vehicle["roll"] TO events[k]["angle"].
					rem_list:ADD(k).
					SET x TO x-1.
					SEt k TO k-1.
					
				} ELSE {
					//local rollsign is SIGN(events[k]["angle"] - vehicle["roll"]).
					
					local rollsign is SIGN( unfixangle( events[k]["angle"] - vehicle["roll"] ) ).
					set vehicle["roll"] TO fixangle(vehicle["roll"] + rollsign*5).
				} 
			}

			ELSE IF events[k]["type"]="action" { 
				IF events[k]:HASKEY("action") {
					events[k]["action"]:call().
				}
				rem_list:ADD(k).
			}
				
			
			
		}
	}
	
	FOR j IN rem_list {
		events:REMOVE(rem_list[j]).
	}
}

FUNCTION get_mass_bias {

	LOCAL stg IS vehicle["stages"][1].
		
	IF NOT stg:HASKEY("tankparts") {get_stg_tanks(stg).}
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

//calculates burn time for a constant thrust stage 
FUNCTION const_f_t {
	PARAMETER stg.

	LOCAL red_flow IS stg["engines"]["flow"] * stg["throttle"].
	RETURN stg["m_burn"]/red_flow.	
}


//calculates when the g limit will be violated and the vehicle mass at that moment
//returns (0,0) if the g-lim is never reached
FUNCTION glim_t_m {
	PARAMETER stg.
	local out is LIST(0,0).
	
	local mbreak is stg["engines"]["thrust"] * stg["Throttle"]/(stg["glim"]*g0).
	IF mbreak > stg["m_final"]  {
		SET out[1] TO mbreak.
		SET out[0] TO (stg["m_initial"] - mbreak)/(stg["engines"]["flow"] * stg["Throttle"]).
	}
	
	RETURN out.
}

//given a constant g stage calculates the burn time until the lower throttle limit will be reached and the vehicle mass at that moment
FUNCTION const_G_t_m {
	PARAMETER stg.
	local out is LIST(0,0).
	
	//calculate mass of the vehicle at throttle violation 
	LOCAL mviol IS stg["engines"]["thrust"] * stg["minThrottle"]/( stg["glim"] * g0 ).
	
	//initialise final mass to stage final mass
	LOCAL m_final IS stg["m_final"].
	
	IF mviol > m_final  {
		SET out[1] TO mviol.
		SET m_final TO mviol.
	}
	
	local red_isp is stg["engines"]["isp"]/stg["glim"].
		
	//calculate burn time until we reach the final mass 
	SET out[0] TO red_isp * LN( stg["m_initial"]/m_final ).
		
	RETURN out.
}


//calculates new stage burn time  as a sum of constant g burn time
//and constant t burn time at minimum throttle
FUNCTION glim_stg_time {
	PARAMETER stg_lex.
	
	local glim is stg_lex["glim"].
	LOCAL tt Is 0.
	
	//compute burn time until  we deplete the stage.	
	
	LOCAL maxtime IS (stg_lex["engines"]["isp"]/glim) * LN(1 + stg_lex["m_burn"]/stg_lex["m_final"] ).

	//compute burn time until  we reach minimum throttle.	
	LOCAL limtime IS - stg_lex["engines"]["isp"]/glim * LN(stg_lex["minThrottle"]).
	LOCAL constThrustTime IS 0.
	IF limtime < maxtime {
		//	First we calculate mass of the fuel burned until violation
		LOCAL burnedFuel IS stg_lex["m_initial"]*(1 - CONSTANT:E^(-glim*limtime/stg_lex["engines"]["isp"])).
		//	Then, time it will take to burn the rest on constant minimum throttle
		SET constThrustTime TO (stg_lex["m_burn"] - burnedFuel  )/(stg_lex["minThrottle"]*stg_lex["engines"]["flow"]).
		SET tt TO limtime + constThrustTime.
	}
	ELSE {
		SET tt TO maxtime.
	}
	SET stg_lex["throt_mult"] TO glim*g0/stg_lex["engines"]["thrust"].
	
	RETURN tt.								
}



FUNCTION get_TWR {
	RETURN vehiclestate["avg_thr"]:average()/(1000*SHIP:MASS*g0).
}




FUNCTION get_stg_tanks {

	FUNCTION parts_tree {
		parameter part0.
		parameter partlist.
		parameter reslist.
	
		
		
		//UNTIL FALSE {
		//	wait 0.
		//	FOR partres IN parentpart:RESOURCES {
		//		IF reslist:KEYS:CONTAINS(partres:NAME) { SET breakflag TO TRUE.}
		//	}
		//	IF parentpart=CORE:PART { SET breakflag TO TRUE.}
		//	IF breakflag { BREAK.}
		//	SET parentpart TO parentpart:PARENT.
		//}
		
		FOR res IN reslist:KEYS {
			LOCAL  parentpart IS part0.
			local breakflag IS FALSE.
			UNTIL FALSE {
				wait 0.
				
				FOR partres IN parentpart:RESOURCES {
					IF res=partres:NAME { 
						IF NOT partlist:CONTAINS(parentpart) {partlist:ADD( parentpart ).}
						SET breakflag TO TRUE.
					}
				}
				
				
				IF parentpart=CORE:PART OR parentpart=SHIP:ROOTPART { SET breakflag TO TRUE.}
				IF breakflag { BREAK.}
				SET parentpart TO parentpart:PARENT.
			}
			
		}
		
		return partlist.
	}


	PARAMETER stg.

	local tanklist IS LIST().
	local reslist is stg["resources"].
	
	list ENGINES in all_eng.
	LOCAL parentpart IS 0.
	FOR e IN all_eng {
		IF e:ISTYPE("engine") {
			IF e:IGNITION {
				SET tanklist TO parts_tree(e:PARENT,tanklist,reslist).
			}
		}
	}
	
	//ignore fuel ducts if already found parts
	IF tanklist:LENGTH=0 {
		LOCAL duct_list IS SHIP:PARTSDUBBED("fuelLine").
		FOR d IN duct_list {
			SET tanklist TO parts_tree(d:PARENT,tanklist,reslist).
		}
	}
	stg:ADD("tankparts", tanklist).	
	
}

FUNCTION get_prop_mass {
	PARAMETER stg.
	
	local tanklist is stg["tankparts"].
	local reslist is stg["resources"].
	local prop_mass IS 0.
	
	FOR tk IN tanklist {
		FOR tkres In tk:RESOURCES {
			FOR res IN reslist:KEYS {
				IF tkres:NAME = res {
					set prop_mass TO prop_mass + tkres:amount*reslist[res].
				}
		
			}
		}
	}
	set prop_mass to prop_mass*1000.
    RETURN prop_mass.
}


//measures everything about the current state of the vehicle, including instantaneous thrust
//thrust only averaged over if staging is not in progress
FUNCTION getState {

	LOCAL deltat IS surfacestate["MET"].
	
	update_navigation().
	
	SET deltat TO surfacestate["MET"] - deltat.
	
	IF (surfacestate:HASKEY("q")  AND surfacestate["vs"] > 50 ) {
		check_maxq(SHIP:Q).
	}
	
	IF DEFINED events {	events_handler().}
	
	//measure and compute vehicle performance parameters
	
	local stg IS get_stage().
	
	LOCAL x IS get_current_thrust_isp().
	
	SET vehiclestate["thr_vec"] TO x[0].
	
	vehiclestate["avg_thr"]:update(vehiclestate["thr_vec"]:MAG).
	
	LOCAL avg_thrust is vehiclestate["avg_thr"]:average().
	LOCAL avg_isp is x[1].

	IF NOT stg:HASKEY("tankparts") {get_stg_tanks(stg).}
		
	LOCAL m_old IS stg["m_initial"].

	SET stg["m_initial"] TO SHIP:MASS*1000.
	
	LOCAL deltam IS m_old - stg["m_initial"].
	
	local res_left IS get_prop_mass(stg).
	
	SET vehiclestate["m_burn_left"] to res_left.
	
	IF NOT (vehiclestate["staging_in_progress"]) {
		
								 
		IF (stg["staging"]["type"]="m_burn") {
		
			SET stg["m_burn"] TO stg["m_burn"] - deltam.
			SET stg["m_final"] TO stg["m_initial"] -  stg["m_burn"].
			
		} ELSE IF (stg["staging"]["type"]="time") {
		
		    	SET stg["Tstage"] TO stg["Tstage"] - deltat.
				
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
}





//Staging function.
FUNCTION STAGING{
	
	local stg_staginginfo IS vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["type"].
	
	local flameout IS (stg_staginginfo="depletion").
	
	IF flameout {
		SET vehiclestate["staging_in_progress"] TO TRUE.
		SET P_steer TO "kill".
	}
	
	addMessage("CLOSE TO STAGING").
	SET vehiclestate["staging_time"] TO TIME:SECONDS+100.		//bias of 100 seconds to avoid premature triggering of the staging actions
	
	
	

	WHEN (flameout AND maxthrust=0) or ((NOT flameout) AND vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <=0.0005 ) THEN {	
		SET vehiclestate["staging_in_progress"] TO TRUE.
		addMessage("STAGING").
		SET vehiclestate["staging_time"] TO TIME:SECONDS.
		IF (vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["type"]="depletion") {set vehiclestate["staging_time"] to vehiclestate["staging_time"] + 1.5 .}
	}
	
	
	local stagingaction IS {STAGE.}.
	IF vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]:HASKEY("stg_action") {
		SET stagingaction TO vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]["stg_action"].
	}
	
	WHEN TIME:SECONDS > vehiclestate["staging_time"] THEN {
		staging_reset(stagingaction).	
	}
	
	IF vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]["ignition"]=TRUE {
		WHEN TIME:SECONDS > (vehiclestate["staging_time"] + vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["ullage_t"]) THEN { 
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
		WHEN TIME:SECONDS > vehiclestate["staging_time"] + 0.5 THEN {
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
	SET stg["ign_t"] TO TIME:SECONDS.
	
	vehiclestate["avg_thr"]:reset().
	
	SET vehiclestate["m_burn_left"] TO stg["m_burn"].
	handle_ullage(stg).
	set_staging_trigger().
	IF vehiclestate["ops_mode"]=2 {SET usc["lastthrot"] TO stg["Throttle"].	}
}

FUNCTION set_staging_trigger {
	WHEN ( vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <= vehicle_pre_staging_t AND vehiclestate["cur_stg"]< (vehicle["stages"]:LENGTH - 1) ) THEN {
		STAGING().
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
	

