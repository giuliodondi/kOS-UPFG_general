//Global vars


GLOBAL g0 IS 9.80665. 


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

	FROM {LOCAL k IS 1.} UNTIL k > (vehicle["stages"]:LENGTH - 1) STEP { SET k TO k+1.} DO{
	
		local stg IS vehicle["stages"][k].
	
	
		LOCAL iisspp IS 0.
		LOCAL tthrust IS 0.
		LOCAL fflow IS 0.
		local stage_res IS LEXICON().
		
		FOR v in stg["engines"] {
			SET tthrust TO tthrust + v["thrust"].
			SET iisspp TO iisspp + v["isp"]*v["thrust"].
			SET fflow TO fflow + v["flow"].
			SET stage_res TO add_resource(stage_res,v["resources"]).
		}
		SET stg["engines"] TO LEXICON("thrust", tthrust, "isp", iisspp/tthrust, "flow", fflow).
		

		SET stage_res TO res_dens_init(stage_res).
		
		stg:ADD("resources",stage_res).
		IF NOT (stg:HASKEY("Throttle")) {stg:ADD("Throttle",1).}
	
		IF NOT (stg:HASKEY("Tstage")) {stg:ADD("Tstage",0).}
		//stg:ADD("ign_t", 0).
		
		fix_mass_params(stg).
		
		SET stg["m_initial"] 			TO stg["m_initial"]*1000.
		SET stg["m_final"] 			TO stg["m_final"]*1000.
		SET stg["m_burn"] 				TO stg["m_burn"]*1000.
		SET stg["engines"]["thrust"] 	TO stg["engines"]["thrust"]*1000.
		
				
		//SET stg["m_initial"] TO stg["m_initial"] + m_bias.
		//SET stg["m_final"] TO stg["m_final"] + m_bias.

		IF NOT stg:HASKEY("mode") {	
			stg:ADD("mode", 1).	
		}
		
		local stg_stagingtype IS stg["staging"]["type"].
		
		IF stg_stagingtype="time" {
			SET stg["Tstage"] TO stg["Tstage"].
		} 
		ELSE IF (stg_stagingtype="depletion" OR stg_stagingtype="m_burn") {
			SET stg["Tstage"] TO (stg["m_burn"])/stg["engines"]["flow"].
		
		}
		ELSE IF stg_stagingtype="glim" {
		
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
					
					SET new_stg["Tstage"] TO glim_stg_time(new_stg).
					vehicle["stages"]:INSERT(k+1, new_stg).

					SET k TO k+1.
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
				SET stg["Tstage"] TO (stg["m_burn"])/stg["engines"]["flow"].
			}
		
		}
		
		
		
	}
	SET vehiclestate["m_burn_left"] TO vehicle["stages"][1]["m_burn"].
	
	vehicle:ADD("ign_t", 0).

	SET control["roll_angle"] TO vehicle["roll"].

	WHEN vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <= 3 THEN {STAGING().}
	
	PRINT " INITIALISATION COMPLETE" AT (0,3).
	

	
	
}






									//CONTROL FUNCTIONS




//open-loop pitch profile for pre-UPFG
FUNCTION pitch {
	PARAMETER v_.
	PARAMETER v0.
	PARAMETER scale.			 
	
	LOCAL default IS 90.

	LOCAL out IS default.
	
	IF v_>v0 {
		
		LOCAL p1 IS -0.0048.
		LOCAL p2 IS 28.8.
		LOCAL p3 IS 26300.
		LOCAL q1 IS 3.923.
		
		LOCAL x IS v_ + 400.391 - v0.
	
		SET out TO (p1*x^2 + p2*x + p3)/(x + q1).
		
		//LOCAL scale IS MIN(0.2,0.15*( (target_orbit["radius"]:MAG - BODY:RADIUS)/250000 - 1)).
		
		SET out TO out*(1 + scale*(1 - out/default)).
		
		LOCAL bias IS out - surfacestate["vdir"].
		
		SET out TO out + 0.8*bias.
		
		
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



//calculates when the g limit will be violated and the vehicle mass at that moment
FUNCTION glim_t_m {
		PARAMETER stg.
		local out is LIST(0,0).
		
		local mbreak is stg["engines"]["thrust"]/(stg["glim"]*g0).
		IF mbreak > stg["m_final"]  {
			SET out[1] TO mbreak.
			SET out[0] TO (stg["m_initial"] - mbreak)/stg["engines"]["flow"].
		}
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
	local stg_staginginfo IS stg["staging"]["type"].
	
	LOCAL x IS get_current_thrust_isp().
	
	SET vehiclestate["thr_vec"] TO x[0].
	
	vehiclestate["avg_thr"]:update(vehiclestate["thr_vec"]:MAG).
	
	LOCAL avg_thrust is vehiclestate["avg_thr"]:average().
	LOCAL avg_isp is x[1].

	
	IF NOT (vehiclestate["staging_in_progress"]) {
		
		IF NOT stg:HASKEY("tankparts") {get_stg_tanks(stg).}
		
		LOCAL m_old IS stg["m_initial"].
		
	
		SET stg["m_initial"] TO SHIP:MASS*1000.
		
		LOCAL dm IS m_old - stg["m_initial"].
		
		local res_left IS get_prop_mass(stg).

		local dmburn IS vehiclestate["m_burn_left"] - res_left.
		SET vehiclestate["m_burn_left"] to res_left.
		
		SET stg["m_final"] TO stg["m_initial"] - res_left.
		
		SET stg["m_final"] TO stg["m_final"] - ( dm - dmburn ).
								 
		IF stg_staginginfo="m_burn" {
			SET stg["m_burn"] TO stg["m_burn"] - dmburn.
		}
		ELSE IF (stg_staginginfo="time") {
		    	SET stg["Tstage"] TO stg["Tstage"] - deltat.
		}
		ELSE IF (stg_staginginfo="glim"){
			
			LOCAL nextstg IS vehicle["stages"][vehiclestate["cur_stg"]+1].
			LOCAL stg_thr IS stg.
			//SET  stg_thr["engines"]["thrust"] TO stg["engines"]["thrust"]*stg["Throttle"].
			//SET  stg_thr["engines"]["flow"] TO stg["engines"]["flow"]*stg["Throttle"].			
			
			//SET stg["m_final"] TO stg["m_initial"] - res_left.
			
			
			LOCAL y IS glim_t_m(stg_thr).
			
			SET nextstg["m_initial"] TO y[1].
			SET nextstg["m_final"] TO stg["m_final"].
			SET stg["m_final"] TO y[1].
			SET stg["m_burn"] TO stg["m_initial"] - y[1].
			SET nextstg["m_burn"] TO res_left - stg["m_burn"].
			SET stg["Tstage"] TO y[0].
			
			IF nextstg["mode"]=1 {
				SET nextstg["Tstage"] TO (nextstg["m_burn"])/nextstg["engines"]["flow"].
			}
			ELSE IF nextstg["mode"]=2 {
				SET nextstg["Tstage"] TO glim_stg_time(nextstg).
			}
		}
		ELSE IF (stg_staginginfo="depletion") {
			
			SET stg["m_burn"] TO res_left.
			
			local thrustfortime IS stg["engines"]["thrust"].
			
			IF NOT (vehiclestate["ops_mode"]=2) AND  (avg_thrust>0) {
					SET stg["engines"]["isp"] TO avg_isp/avg_thrust.
					SET thrustfortime TO avg_thrust.
			}
			SET thrustfortime TO thrustfortime*stg["Throttle"].
			
			IF thrustfortime>0 AND stg["engines"]["isp"]>0 {	
							
				local tt is 0.
				IF stg["mode"]=1 {
					SET stg["engines"]["flow"] TO  thrustfortime/(stg["engines"]["isp"]*g0).
					SET stg["Tstage"] TO (stg["m_burn"])/stg["engines"]["flow"].
				}
				ELSE IF stg["mode"]=2 {
					SET stg["Tstage"] TO glim_stg_time(stg).
				}
			}
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
	WHEN ( (vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <= 3) AND ( vehiclestate["cur_stg"]< (vehicle["stages"]:LENGTH - 1)) ) THEN {STAGING().}
	IF vehiclestate["ops_mode"]=2 {SET usc["lastthrot"] TO stg["Throttle"].	}
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
	

