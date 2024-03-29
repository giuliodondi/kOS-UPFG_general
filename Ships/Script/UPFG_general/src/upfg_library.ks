SET CONFIG:IPU TO 600.


//conic state extrapolation function / gravity integrator
//RUNPATH("0:/Libraries/cser_new").
RUNPATH("0:/Libraries/cser_sg_simple").


//global UPFG variables 

GLOBAL upfgFinalizationTime IS 5.		//	When time-to-go gets below that, keep attitude stable and simply count down time to cutoff.
GLOBAL upfgConvergenceTgo IS 0.5.	//	Maximum difference between consecutive UPFG T-go predictions that allow accepting the solution.
GLOBAL upfgConvergenceVec IS 20.	//	Maximum angle between guidance vectors calculated by UPFG between stages that allow accepting the solution.
	


GLOBAL upfgInternal IS LEXICON().
GLOBAL usc IS LEXICON().	
	
									//	UPFG HANDLING FUNCTIONS






//	Creates and initializes UPFG internal struct
FUNCTION setupUPFG {
	parameter target.
	
	LOCAL curR IS orbitstate["radius"].
	LOCAL curV IS orbitstate["velocity"].

	SET target_orbit["normal"] TO upfg_normal(target_orbit["inclination"], target_orbit["LAN"]).
	
	local cutvec is VXCL(target_orbit["normal"], vecYZ(-SHIP:ORBIT:BODY:POSITION)):NORMALIZED.
	//arbitrarily set cutoff point at 30 degrees ahead of the current position.
	set cutvec to rodrigues(cutvec, target_orbit["normal"], 30).
	
	LOCAL cutoff_r IS target_orbit["cutoff alt"]*1000 + SHIP:BODY:RADIUS.
	
	set target_orbit["radius"] TO cutvec:NORMALIZED * cutoff_r.

	LOCAL tgoV IS VCRS(-target_orbit["normal"], target_orbit["radius"]):NORMALIZED.	
	SET tgoV TO rodrigues(tgoV,target_orbit["normal"], target_orbit["fpa"]).	
	SET tgoV TO target_orbit["velocity"]* tgoV - curV.
	
	//LOCAL tgoV IS target_orbit["velocity"] - curV.
	
	local rgrav IS -SHIP:ORBIT:BODY:MU/2 * curR / curR:MAG^3.
	
	local stg is get_stage().
	
	local LEX1 IS LEXICON(
		"cser", 4,
		"rbias", V(0, 0, 0),
		"rd", target_orbit["radius"],
		"rpd", target_orbit["radius"],
		"rgrav", rgrav,
		"time", surfacestate["MET"],
		"tgo", 100,
		"v", curV,
		"vgo", tgoV,
		"lambda", V(1,0,0),
		"lambdadot", V(0,0,0),
		"t_lambda",surfacestate["MET"],
		"steering",V(1,0,0),
		"throtset",stg["Throttle"]
	).
	local LEX2 IS LEXICON(
		"iter",-2,
		"conv",-2,
		"itercount",0,
		"lastvec",V(1,0,0),
		"lastiter",surfacestate["MET"],
		"lastthrot",stg["Throttle"],
		"terminal",FALSE
	).

	RETURN LIST(LEX1,LEX2).
}	

FUNCTION upfg_framerot {
	DECLARE PARAMETER upfg_in.
	
	LOCAL newref IS SOLARPRIMEVECTOR.
	
	LOCAL rota is signed_angle(usc["lastref"],SOLARPRIMEVECTOR,-v(0,1,0),1).
	
	SET upfg_in["rd"] TO rodrigues(upfg_in["rd"],V(0,0,1),rota).
	SET upfg_in["rgrav"] TO rodrigues(upfg_in["rgrav"],V(0,0,1),rota).
	SET upfg_in["v"] TO rodrigues(upfg_in["v"],V(0,0,1),rota).
	SET upfg_in["vgo"] TO rodrigues(upfg_in["vgo"],V(0,0,1),rota).
	SET upfg_in["lambda"] TO rodrigues(upfg_in["lambda"],V(0,0,1),rota).
	SET upfg_in["lambdadot"] TO rodrigues(upfg_in["lambdadot"],V(0,0,1),rota).

	SET usc["lastref"] TO newref.
	
	RETURN upfg_in.

}

FUNCTION upfg_normal {
	PARAMETER tgtIncl.
	PARAMETER tgtLAN.

	RETURN vecYZ(targetNormal(tgtIncl, tgtLAN)). 
}

FUNCTION upfg_wrapper {

	DECLARE PARAMETER upfgInternal.
	

	LOCAL currentIterationTime IS surfacestate["MET"].
	
	clearvecdraws().
	
	LOCAL wasconv IS usc["conv"]=1.
	
	SET upfgInternal["throtset"] TO usc["lastthrot"].
	
	LOCAL out IS upfg_regular(
					currentIterationTime, 
					vehicle["stages"]:SUBLIST(vehiclestate["cur_stg"],vehicle:LENGTH-vehiclestate["cur_stg"]) , 
					target_orbit , 
					upfgInternal
	).
	
	LOCAL upfgOutput IS out[0].
	SET target_orbit TO out[1].
	
	//arrow_body(vecYZ(target_orbit["normal"]), "norm").
	//arrow_body(vecYZ(target_orbit["radius"]), "cutoff").
	
	IF NOT usc["terminal"] {
		IF usc["conv"]<1 {SET usc["itercount"] TO usc["itercount"]+1.}
		
		LOCAL iterationDeltaTime IS ABS(currentIterationTime - usc["lastiter"]).
		IF vehiclestate["staging_in_progress"] {
			SET iterationDeltaTime TO 0.
			SET upfgOutput["time"] TO upfgInternal["time"].
		}		
  
		SET usc["lastiter"] TO currentIterationTime.
		LOCAL expectedTgo IS upfgInternal["tgo"]- iterationDeltaTime.
			
		IF ABS(expectedTgo-upfgOutput["tgo"]) < upfgConvergenceTgo { //first criterion for convergence
			IF VANG(upfgOutput["steering"], upfgInternal["steering"]) < upfgConvergenceVec { //second criterion for convergence
				IF usc["conv"]<1 { //score one good hit, increment until conv is 1
					SET usc["conv"] TO usc["conv"]+1.
				}
			} ELSE { //something is wrong, reset
				IF NOT vehiclestate["staging_in_progress"] {
					SET upfgOutput TO resetUPFG(upfgOutput).
		    	}
		    }
		} ELSE {SET usc["conv"] TO usc["iter"].}	

		IF wasconv AND usc["conv"]<1 {
			//in this case we had convergence and we lost it, reset itercount
			SET usc["itercount"] TO 0.
		}
	}
	IF usc["conv"]=1 { //converged and stable, accept result
			SET usc["lastvec"] TO upfgOutput["steering"].
			SET usc["lastthrot"] TO upfgOutput["throtset"].
	}
	ELSE{
		IF wasconv{//in this case we had convergence and we lost it, reset itercount
		SET usc["itercount"] TO 0.
		}
	}

	RETURN upfgOutput.
}


FUNCTION resetUPFG {
	PARAMETER upfgOutput.
	

	addMessage("RESETTING UPFG").
	LOCAL lastvec IS usc["lastvec"].
	LOCAL x IS setupUPFG(target_orbit).
	SET upfgOutput[0] TO x[0].
	SET usc TO x[1].
	SET usc["lastvec"] TO lastvec.
	local stg is get_stage().
	SET usc["lastthrot"] TO stg["Throttle"].
	
	RETURN upfgOutput.
}



//		UPFG MAIN ROUTINE

FUNCTION upfg_regular {

	DECLARE FUNCTION compute_iF {
		PARAMETER time_.
		LOCAL out IS  lambda + lambdadot*time_.
		RETURN out:NORMALIZED.
	}

	PARAMETER t.
	PARAMETER vehicle.
	PARAMETER tgt_orb.
	PARAMETER previous.
	

	LOCAL dt IS t - previous["time"].
	LOCAL v_cur IS orbitstate["velocity"].
	LOCAL vgo IS previous["vgo"] - (v_cur - previous["v"]).
	LOCAL tgo IS previous["tgo"].
	LOCAL lambda IS previous["lambda"].
	LOCAL lambdadot IS previous["lambdadot"].
		
	LOCAL r_cur IS orbitstate["radius"].
	LOCAL cser IS previous["cser"].
	LOCAL rd IS previous["rd"].
	LOCAL rbias IS previous["rbias"].
	LOCAL rgrav IS previous["rgrav"].
	LOCAL iy IS -tgt_orb["normal"]:NORMALIZED.
	LOCAL iz IS VCRS(rd,iy):NORMALIZED.
	LOCAL m IS vehicle[0]["m_initial"].
	LOCAL Kk IS previous["throtset"].
	
	LOCAL t40flag IS tgo<40.
	
	LOCAL g0 IS 9.80665. 
	
	//	1
	LOCAL n IS vehicle:LENGTH.
	LOCAL SM IS LIST().
	LOCAL aL IS LIST().
	LOCAL md IS LIST().
	LOCAL ve IS LIST().
	LOCAL fT IS LIST().
	LOCAL aT IS LIST().
	LOCAL tu IS LIST().
	LOCAL tb IS LIST().
	LOCAL kklist IS LIST().
  
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		SM:ADD(vehicle[i]["mode"]).
		IF vehicle[i]:HASKEY("glim") {
			aL:ADD(vehicle[i]["glim"]*g0).
		} ELSE {
			aL:ADD(0).
		}
		fT:ADD(vehicle[i]["engines"]["thrust"]).
		md:ADD(vehicle[i]["engines"]["flow"]).
		kklist:ADD(vehicle[i]["Throttle"]).
		IF (i=0) {
			SET kklist[i] TO Kk.	
		}
		SET fT[i] TO fT[i]*kklist[i].
		SET md[i] TO md[i]*kklist[i].
		ve:ADD(vehicle[i]["engines"]["isp"]*g0).
		aT:ADD(fT[i] / vehicle[i]["m_initial"]).
		tu:ADD(ve[i]/aT[i]).
		tb:ADD(vehicle[i]["Tstage"]).
	}
	
	
	//	3
	IF SM[0]=1 {
		SET aT[0] TO fT[0] / m.
	} ELSE IF SM[0]=2 {
		SET aT[0] TO aL[0].
	}
	SET tu[0] TO ve[0] / aT[0].
	
	LOCAL Li IS LIST().
	LOCAL Lsum IS 0.
	FROM { LOCAL i IS 0. } UNTIL i>=n-1 STEP { SET i TO i+1. } DO {
		IF SM[i]=1 {
			Li:ADD( ve[i]*LN(tu[i]/(tu[i]-tb[i])) ).
		} ELSE IF SM[i]=2 {
			Li:ADD( aL[i]*tb[i] ).
		} ELSE Li:ADD( 0 ).
		SET Lsum TO Lsum + Li[i].
		
		IF Lsum>vgo:MAG {
			RETURN upfg_regular(
				t,
				vehicle:SUBLIST(0,vehicle:LENGTH-1),
				tgt_orb,
				previous
			).
		}
	}
	Li:ADD(vgo:MAG - Lsum).
	
	
	LOCAL tgoi IS LIST().
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		IF SM[i]=1 {
			SET tb[i] TO tu[i] * (1-CONSTANT:E^(-Li[i]/ve[i])).
		} ELSE IF SM[i]=2 {
			SET tb[i] TO Li[i] / aL[i].
		}
		IF i=0 {
			tgoi:ADD(tb[i]).
		} ELSE {
			tgoi:ADD(tgoi[i-1] + tb[i]).
		}
	}
	
	SET tgo TO tgoi[n-1].
	
	//	4
	LOCAL L_ IS 0.
	LOCAL J_ IS 0.
	LOCAL S_ IS 0.
	LOCAL Q_ IS 0.
	LOCAL H_ IS 0.
	LOCAL P_ IS 0.
	LOCAL Ji IS LIST().
	LOCAL Si IS LIST().
	LOCAL Qi IS LIST().
	LOCAL Pi IS LIST().
	LOCAL tgoi1 IS 0.
	
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		IF i>0 {
			SET tgoi1 TO tgoi[i-1].
		}
		IF SM[i]=1 {
			Ji:ADD( tu[i]*Li[i] - ve[i]*tb[i] ).
			Si:ADD( -Ji[i] + tb[i]*Li[i] ).
			Qi:ADD( Si[i]*(tu[i]+tgoi1) - 0.5*ve[i]*tb[i]^2 ).
			Pi:ADD( Qi[i]*(tu[i]+tgoi1) - 0.5*ve[i]*tb[i]^2 * (tb[i]/3+tgoi1) ).
		} ELSE IF SM[i]=2 {
			Ji:ADD( 0.5*Li[i]*tb[i] ).
			Si:ADD( Ji[i] ).
			Qi:ADD( Si[i]*(tb[i]/3+tgoi1) ).
			Pi:ADD( (1/6)*Si[i]*(tgoi[i]^2 + 2*tgoi[i]*tgoi1 + 3*tgoi1^2) ).
		}
		
		SET Ji[i] TO Ji[i] + Li[i]*tgoi1.
		SET Si[i] TO Si[i] + L_*tb[i].
		SET Qi[i] TO Qi[i] + J_*tb[i].
		SET Pi[i] TO Pi[i] + H_*tb[i].
		
		SET L_ TO L_+Li[i].
		SET J_ TO J_+Ji[i].
		SET S_ TO S_+Si[i].
		SET Q_ TO Q_+Qi[i].
		SET P_ TO P_+Pi[i].
		SET H_ TO J_*tgoi[i] - Q_.
	}
	LOCAL K_ IS J_/L_.
	
	
	//	5
	IF vgo:MAG <>0 { SET lambda TO vgo:NORMALIZED.}
	IF previous["tgo"]>0 {
		SET rgrav TO (tgo/previous["tgo"])^2 * rgrav.
	}
	
	LOCAL rgo IS rd - (r_cur + v_cur*tgo + rgrav).
	LOCAL iz IS VCRS(rd,iy):NORMALIZED.
	LOCAL rgoxy IS rgo - VDOT(iz,rgo)*iz.
	LOCAL rgoz IS (S_ - VDOT(lambda,rgoxy)) / VDOT(lambda,iz).
	SET rgo TO rgoxy + rgoz*iz + rbias.
	LOCAL lambdade IS Q_ - S_*K_.
	
	IF (NOT t40flag) {
		SET lambdadot TO (rgo - S_*lambda) / lambdade.
	}
	
	
	LOCAL iF_ IS compute_iF(-K_).
	LOCAL phi IS VANG(iF_,lambda)*CONSTANT:DEGTORAD.
	LOCAL phidot IS -phi/K_.
	LOCAL vthrust IS (L_ - 0.5*L_*phi^2 - J_*phi*phidot - 0.5*H_*phidot^2).
	SET vthrust TO vthrust*lambda - (L_*phi + J_*phidot)*lambdadot:NORMALIZED.
	LOCAL rthrust IS S_ - 0.5*S_*phi^2 - Q_*phi*phidot - 0.5*P_*phidot^2.
	SET rthrust TO rthrust*lambda - (S_*phi + Q_*phidot)*lambdadot:NORMALIZED.
	SET vbias TO vgo - vthrust.
	SET rbias TO rgo - rthrust.
	
	
	//	7
	
	
	LOCAL rc1 IS r_cur - 0.1*rthrust - (tgo/30)*vthrust.
	LOCAL vc1 IS v_cur + 1.2*rthrust/tgo - 0.1*vthrust.
	LOCAL pack IS cse(rc1, vc1, tgo, cser).
	SET cser TO pack[2].
	SET rgrav TO pack[0] - rc1 - vc1*tgo.
	LOCAL vgrav IS pack[1] - vc1.
	
	
	//	8
	LOCAL rp IS r_cur + v_cur*tgo + rgrav + rthrust.
	
	IF (NOT t40flag) {
		SET rp TO VXCL(iy,rp).
	}
	
	LOCAL vd IS v(0,0,0).
	
	LOCAL ix IS rp:NORMALIZED.
	SET iz TO VCRS(ix,iy):NORMALIZED.
	
	SET tgt_orb TO cutoff_params(tgt_orb,rd).
	
	SET rd TO tgt_orb["radius"]:MAG*ix.	
	SET vd TO rodrigues(iz,iy, tgt_orb["fpa"]):NORMALIZED*tgt_orb["velocity"].	

	SET vgo TO vd - v_cur - vgrav + vbias.
	
	//	RETURN - build new internal state instead of overwriting the old one
	LOCAL current IS LEXICON(
		"cser", cser,
		"rbias", rbias,
		"rd", rd,
		"rp", rp,
		"rgrav", rgrav,
		"time", t,
		"tgo", tgo,
		"v", v_cur,
		"vgo", vgo,
		"lambda", lambda,
		"lambdadot", lambdadot,
		"t_lambda",(t + K_),
		"steering",iF_,
		"throtset",Kk,
		"flyback_flag",false,
		"dmbo",0,
		"mbod",0,
		"Tc",0
	).
	
	
	RETURN LIST(current,tgt_orb).
}