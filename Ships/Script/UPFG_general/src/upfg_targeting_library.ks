//GLOBAL NAV VARIABLES 

GLOBAL launchpad IS SHIP:GEOPOSITION.
GLOBAL surfacestate IS  LEXICON(
								"time",0,
								"deltat", 0,
								"MET",0,
								"surfv", v(0,0,0),
								"az",0,
								"pitch",0,
								"alt",0,
								"vs",0,
								"hs",0,
								"vdir",0,
								"hdir",0,
								"q",0, 
								"maxq", 0
).

GLOBAL orbitstate IS  LEXICON(
								"radius",v(0,0,0),
								"velocity",v(0,0,0)
). 

GLOBAL debug_tli is false.



// normal vector of the current instantaneous orbital plane
//it's opposite of upfg normal vector convention 
FUNCTION currentNormal{
	RETURN VCRS(orbitstate["radius"],orbitstate["velocity"]):NORMALIZED.
}


FUNCTION cutoff_params {
	PARAMETER tgt_orb.
	PARAMETER cutoff_r.
	
	SET tgt_orb["normal"] TO upfg_normal(tgt_orb["inclination"], tgt_orb["LAN"]).
	
	set tgt_orb["radius"] to cutoff_r.
	
	local cut_alt is tgt_orb["radius"]:MAG.
	set tgt_orb["eta"] to orbit_alt_eta(cut_alt, tgt_orb["SMA"], tgt_orb["ecc"]).
	
	set tgt_orb["velocity"] to orbit_alt_vel(cut_alt, tgt_orb["SMA"]).
	
	set tgt_orb["fpa"] to orbit_eta_fpa(tgt_orb["eta"], tgt_orb["SMA"], tgt_orb["ecc"]).

	RETURN tgt_orb.
}

//upfg-compatible velocity vector
//expects normal vector as the left-handed product of position and velocity
function cutoff_velocity_vector {
	parameter cutoff_r.
	parameter normvec.
	parameter cutoff_vel.
	parameter cutoff_fpa.
	
	local ix_ is cutoff_r:normalized.
	local iy_ is normvec:normalized.
	local iz_ is VCRS(ix_, iy_):NORMALIZED.
	
	return rodrigues(iz_, iy_, cutoff_fpa):NORMALIZED * cutoff_vel.	

}



//only called if hastarget=true
//propagates the current target orbital parameters forward in time 
//given a delta-t and applies secular variations
//given by j2 perturbation
FUNCTION tgt_j2_timefor {
	PARAMETER tgt_orb.
	PARAMETER deltat.
	
	LOCAL j2LNG is -1.5*1.08262668e-3*rad2deg((BODY:RADIUS/(TARGET:ORBIT:SEMIMAJORAXIS*(1 - TARGET:ORBIT:ECCENTRICITY^2)))^2*SQRT(BODY:MU/TARGET:ORBIT:SEMIMAJORAXIS^3)*COS(TARGET:ORBIT:INCLINATION)).
	
	SET tgt_orb["LAN"] TO  fixangle(TARGET:ORBIT:LAN +  j2LNG*deltat ).
	SET tgt_orb["inclination"] TO TARGET:ORBIT:INCLINATION.	

	RETURN tgt_orb.

}		



							//PRELAUNCH TARGETING FUNCTIONS

//computes LAN of target orbit that provides a TLI opportunity
//given  a transit time
//accounts for warp time to launch and TLI waiting time

declare function moon_shot {

	RUNPATH("0:/UPFG_pdi/Moon_sites.ks").	

	//as the moon rotates along its orbit it traces a ground track on the surface of earth.
	//at any time we define the antipode vector as the antipode to the moon's 
	//instantaneous position on the surface.
	//we have a tli trajectory and we know how it's oriented with respect to the moon 
	//at ARRIVAL NOT DEPARTURE.
	//so we need to rotate the point of TLI to account for the moon's movement during transfer.
	//the point of TLI is then the moon's antipode at arrival
	//and the injection parking orbit must contain this vector.
	
	//we have a parking orbit defined as inclination sma and ecc,
	//we need a lan value to place it in space and inject onto it.
	//the orbital plane needs to be positioned with respect to the moon plane so it 
	//contains the parking vector.
	
	//but we also need to account for the warp time to launch window, the moon moves during it
	//and we don't know how long it is because we don't know the orbital plane without the right antipode,
	//and we also need to correct for the coast time between injection and tli.
	//we find it iteratively.
	
	clearvecdraws().

	//WE ARE DOING THE CALCULATIONS IN THE KSP FRAME 
	
	//target site
	local moonsite is pdi_siteslex[moon_transfer["tgt_site"]].
	local moon_sitevec is moonsite["position"]:position - moon_transfer["body"]:position.
	
	//initialise antipode vector
	LOCAL antipodevec IS (-moon_transfer["body"]:POSITION + SHIP:ORBIT:BODY:POSITION):NORMALIZED*SHIP:ORBIT:BODY:RADIUS*2.
	
	if (debug_tli) {
		arrow_body(antipodevec, "antipode1").
	}
	
	//find transfer time
	
	//find intersection point bw transfer orbit and lunar SOI given fixed patching parameters
	
	local moon_xfer_apo is 450000.		//hard-coded, km 
	LOCAL lambda1 IS 80. //hard-coded value
	LOCAL r2 IS 66100000.
	LOCAL D_em IS (moon_transfer["body"]:POSITION - SHIP:ORBIT:BODY:POSITION):MAG.
	
	LOCAL r1 IS sqrt( D_em^2 + r2^2 - 2*d_em*r2*cos(lambda1) ).
	LOCAL gamma1 IS ARCSIN(r2*SIN(lambda1)/r1).
	
	//find transfer time between periapsis and intersection point
	
	moon_transfer:ADD("SMA", orbit_appe_sma(moon_xfer_apo, target_orbit["periapsis"])).
	moon_transfer:ADD("ecc", orbit_appe_ecc(moon_xfer_apo, target_orbit["periapsis"])).
	
	//true anomaly
	LOCAL transfer_moon_eta IS orbit_alt_eta(r1, moon_transfer["SMA"], moon_transfer["ecc"]).
	
	//transfer time between periapsis and encounter, plus countdown and launchtimeadvance time shifts.
	//still missing pre-launch warp time and parking orbit coast time
	//will compute them iteratively
	LOCAL tau0 IS  eta_to_dt(transfer_moon_eta, moon_transfer["SMA"], moon_transfer["ecc"]) + vehicle["launchTimeAdvance"] + 10.1.
	
	LOCAL moon_orbit IS LEXICON (	
							"SMA",moon_transfer["body"]:ORBIT:SEMIMAJORAXIS,
							"ecc",moon_transfer["body"]:ORBIT:ECCENTRICITY,
							"inclination",moon_transfer["body"]:ORBIT:INCLINATION,													
							"LAN",moon_transfer["body"]:ORBIT:LAN,
							"normal", - body_orbital_normal_vec(moon_transfer["body"])
	) . 
	
	if (debug_tli) {
		arrow_body(moon_orbit["normal"] * SHIP:ORBIT:BODY:RADIUS, "moon_normal").
	}

	//rotate antipode the first time by at least the transfer time
	//find the MASTER antipode on which the rest is based
	LOCAL rota IS t_to_eta(moon_transfer["body"]:ORBIT:TRUEANOMALY,tau0,moon_orbit["SMA"],moon_orbit["ecc"]) - moon_transfer["body"]:ORBIT:TRUEANOMALY .
	
	if (debug_tli) {
		PRINT "transfer time " + sectotime(tau0) AT (0,44).
		PRINT "moon transfer rotation base" + rota AT (0,45).
	}
	
	SET rota TO rota + gamma1.
	SET antipodevec TO rodrigues(antipodevec, moon_orbit["normal"], -rota). 
	
	//find the tli normal vectors given target site 
	local moon_arrival_pos is (moon_transfer["body"]:POSITION - SHIP:ORBIT:BODY:POSITION).
	set moon_arrival_pos to rodrigues(moon_arrival_pos, moon_orbit["normal"], -rota). 
	set moon_sitevec to rotate_moon_posvec(moon_sitevec, tau0).
	
	//best tli relative angle
	local tli_opps is tli_planner_site(
				time,
				moon_arrival_pos,
				moon_orbit["normal"],
				moon_sitevec,
				target_orbit["inclination"]
	).
	
	local best_tli_opp is tli_opps[0].
	
	for t_opp in tli_opps {
		if (t_opp["site_angle"] < best_tli_opp["site_angle"]) {
			set best_tli_opp to t_opp.
		}
	}
	
	set moon_transfer["rel_angle_high"] to (best_tli_opp["orbit_angle"] = "high").
	
	//LAN and injection vector given inclination and "right now" launch
	
	SET target_orbit["LAN"] TO LAN_orbit_overhead(target_orbit["inclination"], (target_orbit["direction"]="south"), vehicle["launchTimeAdvance"] + vehicle_countdown + 1).
	
	LOCAL parking_orb_normal IS targetNormal(target_orbit["inclination"], target_orbit["LAN"]).
	
	LOCAL injectionvec IS VXCL(parking_orb_normal, - SHIP:ORBIT:BODY:POSITION ).
	
	//work out the tli relative angle based on proximity to the target site on the moon 
	
	//is the antipode closer to its ascending or descending node?
	//if the antipode is ascending, the moon is descending by definition
	local apveclng IS VXCL(V(0,1,0),antipodevec).
	LOCAL deltalng_lan IS signed_angle( apveclng ,  SOLARPRIMEVECTOR,v(0,1,0),1).
	set deltalng_lan TO unfixangle(deltalng_lan - moon_orbit["LAN"]).
	
	LOCAL antipode_asc IS TRUE.
	IF ABS(deltalng_lan)>90 {set antipode_asc to FALSE.} 
	
	local tli_plane_above_moon_orb is (moon_transfer["rel_angle_high"] and antipode_asc) 
									or ((NOT moon_transfer["rel_angle_high"]) and (not antipode_asc)).
	
	//during the iteration we initialise the tli vector as the ascending or descending node 
	//of the parking orbit, rotate it until we match the antipode's latitude and then compare the longitude.
	//to do this we need to determine where the antipode lies with respect to the parking orbit
	//in order to define the correct node and the rotation angle sign
	//and we assume that these don't change when we add the warp time
	
	//TLI vector rotation sign
	LOCAL tli_s IS 1.
	
	
	//is the antipode above or below the equatorial plane?
	LOCAL antipodelat IS BODY:GEOPOSITIONOF(antipodevec + SHIP:BODY:POSITION):LAT.
	LOCAL antipode_above IS TRUE.
	IF SIGN(antipodelat)<0 {set antipode_above to FALSE.}
	
	
	
	//should TLI occur close to the parking orbit's ascending or descending node?
	LOCAL tli_ldn IS FALSE.
	
	if (moon_transfer["rel_angle_high"]) {
		IF  antipode_asc {SET tli_ldn TO TRUE.}
	} else {
		IF NOT antipode_asc {SET tli_ldn TO TRUE.}
	}
	
	//now determine whether to rotate forwards or backwards
	IF tli_ldn {
		IF antipode_above {SET tli_s TO -1.}
	}
	ELSE {
		IF NOT antipode_above {SET tli_s TO -1.}
	}

	if (debug_tli) {
		PRINT "antipode above: " + antipode_above AT (0,47).
		PRINT "tli plane above moon plane: " + tli_plane_above_moon_orb AT (0,48).
		PRINT "antipode ascending: " + antipode_asc AT (0,49).
		PRINT "difference of longitude of antipode and moon lan: " + deltalng_lan AT (0,50).
		PRINT "tli lan or ldn: " + tli_ldn AT (0,51).
		PRINT "tli rotation sign: " + tli_s AT (0,52).
	}
	
	//time factors
	GLOBAL tli_wait IS 3600.
	LOCAL warp_dt IS 0.
	LOCAL pkobt_t Is 2*CONSTANT:PI*SQRT(target_orbit["SMA"]^3/(SHIP:ORBIT:BODY:MU)).
	
	LOCAL epsilon IS 0.05.
	
	lOCAL dLNG IS 0.
	LOCAL tlivec IS V(0,0,0).
	
	UNTIL FALSE {
		//adjust "time ahead"
		LOCAL tau IS tau0 + tli_wait + warp_dt.
		
		//current antipode vector
		//LOCAL antipodevec IS -moon_transfer["body"]:POSITION + SHIP:BODY:POSITION.
		SET antipodevec TO (-moon_transfer["body"]:POSITION + SHIP:ORBIT:BODY:POSITION):NORMALIZED*SHIP:ORBIT:BODY:RADIUS*2.
		
		//rotate antipode given time ahead
		SET rota TO t_to_eta(moon_transfer["body"]:ORBIT:TRUEANOMALY,tau,moon_orbit["SMA"],moon_orbit["ecc"]).
		SET rota TO rota -moon_transfer["body"]:ORBIT:TRUEANOMALY + gamma1.
		SET antipodevec TO rodrigues(antipodevec, moon_orbit["normal"], -rota). 
	
		//TLI vector initialised as ascending node and if appropriate turned into descending node
		
		SET tlivec TO rodrigues(SOLARPRIMEVECTOR, V(0,1,0), -target_orbit["LAN"]):NORMALIZED*SHIP:ORBIT:BODY:RADIUS*2..
		IF tli_ldn {SET tlivec TO -tlivec.}
	
		//rotate TLI vector given antipode's latitude
		LOCAL antipodelat IS ABS(BODY:GEOPOSITIONOF(antipodevec + SHIP:BODY:POSITION):LAT).
		SET rot_antip TO fixangle(get_c_bBB(antipodelat,target_orbit["inclination"])).

		SET tlivec TO rodrigues(tlivec, parking_orb_normal, -tli_s*rot_antip). 
		
		//set longitude of periapsis to longitude of TLI
		local pveclng IS VXCL(V(0,1,0),tlivec).
		SET target_orbit["Longitude of Periapsis"] TO signed_angle(SOLARPRIMEVECTOR, pveclng , v(0,1,0),1).		

		//determine whether the antipode lies close enough to the current orbital plane
		//by computing longitude difference bw antipode and TLI vector
		//which are at the same latitude by construction	
		SET dLNG TO signed_angle(tlivec, antipodevec , v(0,1,0),0).	
		
		//PRINT "angle bw tlivec and antipode: " + dLNG AT (0,55).
				
		//SET dLNG TO project_angle(dLNG,antipodelat).

		
		//if so exit, the orbit plane has been completely determined
		IF ABS(dLNG) <= epsilon {BREAK.}

		//if not rotate the entire plane to intersect the antipode vector
		
		SET target_orbit["LAN"] TO fixangle(target_orbit["LAN"] - 0.7 * dLNG ).
		SET injectionvec TO rodrigues(injectionvec,V(0,1,0),dLNG ).
		SET tlivec TO rodrigues(tlivec,V(0,1,0),dLNG).
		SET parking_orb_normal TO targetNormal(ABS(target_orbit["inclination"]), target_orbit["LAN"]).
		
		local tli_antip_angle is signed_angle(tlivec, antipodevec , v(0,1,0),0).	
		
		//compute warp time to this new plane
		SET warp_dt TO orbitInterceptTime(target_orbit["inclination"], target_orbit["LAN"], (target_orbit["direction"]="south")).
		
		//estimate coast time between injection and TLI
		SET coasta TO signed_angle(tlivec, injectionvec, parking_orb_normal ,1).
		//remember that we assume the periapsis is at tli.
		SET tli_wait TO eta_to_dt(coasta,target_orbit["SMA"],target_orbit["ecc"]).
		
		
		if (debug_tli) {
			print "lunar rot angle between now and arrival: " + rota  at (0,54).
			PRINT "delta_longitude: " + dLNG AT (0,55).
			PRINT "tli/antipode angle: " + tli_antip_angle AT (0,56).
			print "parking obt coast angle:" + coasta at (0,57).
		}

		//if TLI is less than half an orbit away add one full orbit
		IF coasta<180 {
			SET tli_wait TO tli_wait + pkobt_t.
		}
		
		//this yields the "time ahead"
		//adjust antipode and iterate until the antipode lies in-plane

		//clearvecdraws().
		//arrow(antipodevec, "antipode",SHIP:ORBIT:BODY:POSITION,1,0.5).
		//arrow(tlivec, "tlivec",SHIP:ORBIT:BODY:POSITION,2,0.5).
		//arrow(injectionvec, "injectionvec",SHIP:ORBIT:BODY:POSITION,2,0.5).
	}
	
	//account for J2 perturbation
		
	LOCAL j2LNG is -1.5*1.08262668e-3*rad2deg((BODY:RADIUS/(target_orbit["SMA"]*(1 - target_orbit["ecc"]^2)))^2*SQRT(BODY:MU/target_orbit["SMA"]^3)*COS(target_orbit["inclination"])).
	
	
	SET target_orbit["LAN"] TO fixangle(target_orbit["LAN"] -  j2LNG*tli_wait ).
	
	PRINT "                                                               " at (0,11).
	PRINT " TLI SCHEDULED FOR " + sectotime(tli_wait) + " AFTER INJECTION" AT (0,11).
	
	if (debug_tli) {
		arrow_body(antipodevec, "antipode2").
		arrow_body(tlivec * SHIP:ORBIT:BODY:RADIUS, "tli_vec").
		arrow_body(parking_orb_normal * SHIP:ORBIT:BODY:RADIUS, "tli_normal").
	}
	
	until (NOT debug_tli) {}.

}

//the planetary flyby routine fixes the orbital plane at given inclination that intersects a given target position after a given transfer time 
//it's basically the same calculations as the moon shot routine, only we expect the transfer time to be much larger than the parking orbit time or the warp time 
//therefore we neglect those times and do an approximate calculation moving the target position by just the transfer time 
declare function planetary_flyby {

	
	//as the moon rotates along its orbit it traces a ground track on the surface of earth.
	//at any time we define the antipode vector as the antipode to the moon's 
	//instantaneous position on the surface.
	//we have a tli trajectory and we know how it's oriented with respect to the moon 
	//at ARRIVAL NOT DEPARTURE.
	//so we need to rotate the point of TLI to account for the moon's movement during transfer.
	//the point of TLI is then the moon's antipode at arrival
	//and the injection parking orbit must contain this vector.
	
	//we have a parking orbit defined as inclination sma and ecc,
	//we need a lan value to place it in space and inject onto it.
	//the orbital plane needs to be positioned with respect to the moon plane so it 
	//contains the parking vector.
	
	//but we also need to account for the warp time to launch window, the moon moves during it
	//and we don't know how long it is because we don't know the orbital plane without the right antipode,
	//and we also need to correct for the coast time between injection and tli.
	//we find it iteratively.
	
	

	//WE ARE DOING THE CALCULATIONS IN THE KSP FRAME 
	//ALL NORMAL VECTORS NEED A VECYZ TO CONVERT THEM FROM UPFG FRAME TO KSP
	
	//initialise antipode vector
	LOCAL antipodevec IS (-flyby_transfer["body"]:POSITION + SHIP:ORBIT:BODY:POSITION).
	

	
	LOCAL tgt_planet_orbit IS LEXICON (	
							"SMA",flyby_transfer["body"]:ORBIT:SEMIMAJORAXIS,
							"ecc",flyby_transfer["body"]:ORBIT:ECCENTRICITY,
							"inclination",flyby_transfer["body"]:ORBIT:INCLINATION,													
							"LAN",flyby_transfer["body"]:ORBIT:LAN,
							"normal",V(1,0,0)
	) . 
	
	SET tgt_planet_orbit["normal"] TO targetNormal(ABS(tgt_planet_orbit["inclination"]), tgt_planet_orbit["LAN"]).
	
	
	//rotate antipode by the transfer time
	//transform the transfer time form days to seconds
	SET flyby_transfer["transfer time"] TO flyby_transfer["transfer time"]*60*60*24.
	
	
	LOCAL rota IS t_to_eta(flyby_transfer["body"]:ORBIT:TRUEANOMALY,flyby_transfer["transfer time"],tgt_planet_orbit["SMA"],tgt_planet_orbit["ecc"]).
	SET rota TO rota -flyby_transfer["body"]:ORBIT:TRUEANOMALY .
	SET antipodevec TO rodrigues(antipodevec, vecYZ(tgt_planet_orbit["normal"]), rota). 
	

	
	//SET target_orbit["LAN"] TO LAN_orbit_overhead().
	IF NOT target_orbit:HASKEY("LAN") {
		target_orbit:ADD("LAN",LAN_orbit_overhead()).
	}
	
	SET target_orbit["normal"] TO targetNormal(ABS(target_orbit["inclination"]), target_orbit["LAN"]).
	
	//the TLi vector correspponds to the antipode vector when the planes match
			
	//during the iteration we initialise the tli vector as the ascending or descending node 
	//of the parking orbit, rotate it until we match the antipode's latitude and then compare the longitude.
	//to do this we need to determine where the antipode lies with respect to the parking orbit
	//in order to define the correct node and the rotation angle sign
	//and we assume that these don't change when we add the warp time
	
	//TLI vector rotation sign
	LOCAL tli_s IS 1.
	
	//is the moon above or below the equatorial plane?
	LOCAL antipodelat IS BODY:GEOPOSITIONOF(antipodevec + SHIP:BODY:POSITION):LAT.
	LOCAL above IS TRUE.
	IF SIGN(antipodelat)<0 {set above to FALSE.}
	
	//is the antipode closer to its ascending or descending node?
	local apveclng IS VXCL(V(0,1,0),antipodevec).
	LOCAL deltalng_lan IS signed_angle( apveclng ,  SOLARPRIMEVECTOR,v(0,1,0),1).
	set deltalng_lan TO deltalng_lan - tgt_planet_orbit["LAN"].
	
	PRINT "difference of longitude of antipode and moon lan: " + deltalng_lan AT (0,48).
	
	LOCAL asc IS TRUE.
	IF ABS(deltalng_lan)>90 {set asc to FALSE.} 
	PRINT "moon ascending: " + asc AT (0,49).
	
	//should TLI occur close to the parking orbit's ascending or descending node?
	LOCAL ldn IS FALSE.
	
	IF (flyby_transfer["rel_angle"]="low") {
		IF NOT asc {SET ldn TO TRUE.}
	}
	ELSE IF (flyby_transfer["rel_angle"]="high") {
		IF  asc {SET ldn TO TRUE.}
	}
	

	
	//now determine whether to rotate forwards or backwards
	IF ldn {
		IF above {SET tli_s TO -1.}
	}
	ELSE {
		IF NOT above {SET tli_s TO -1.}
	}

	
	PRINT "lan or ldn: " + ldn AT (0,50).
	PRINT "tli rotation sign: " + tli_s AT (0,51).
	
	//TLI vector initialised as ascending node and if appropriate turned into descending node
		
	LOCAL tlivec IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), -target_orbit["LAN"]).
	IF ldn {SET tlivec TO -tlivec.}
	
	//rotate TLI vector given antipode's latitude
	LOCAL antipodelat IS ABS(BODY:GEOPOSITIONOF(antipodevec + SHIP:BODY:POSITION):LAT).
	SET rota TO get_c_bBB(antipodelat,target_orbit["inclination"]).

	SET tlivec TO rodrigues(tlivec, vecYZ(target_orbit["normal"]), tli_s*rota). 
		
	//determine whether the antipode lies close enough to the current orbital plane
	//by computing longitude difference bw antipode and TLI vector
	//which are at the same latitude by construction		
	lOCAL dLNG IS signed_angle(tlivec, antipodevec , v(0,1,0),0).	
	
	//rotate the entire plane to intersect the antipode vector
	SET target_orbit["LAN"] TO fixangle(target_orbit["LAN"] - dLNG ).
	
	

}




FUNCTION warp_window{
	parameter warp_dt.
	
	LOCAL launch_time IS TIME:SECONDS + warp_dt.
	addMessage("TIME TO WINDOW : " + sectotime(warp_dt)).

	UNTIL FALSE {
	
		LOCAL timetolaunch IS launch_time - TIME:SECONDS.
		
		warp_controller(timetolaunch, TRUE, 2).
		
		IF (timetolaunch <=0.1) {BREAK.}
		
		Wait 0.
	}
	set warp to 0.
}


// calculate target orbital parameters 
//calculate launch azimuth and window and handle warp 
FUNCTION prepare_launch {

	clearvecdraws().

	target_orbit:ADD("direction", "nearest").
	target_orbit:ADD("sma", 0).
	target_orbit:ADD("ecc", 0).
	target_orbit:ADD("eta", 0).
	target_orbit:ADD("LAN", 0).
	target_orbit:ADD("radius", 0).
	target_orbit:ADD("velocity", 0).
	target_orbit:ADD("normal", V(0,0,0)).
	target_orbit:ADD("fpa", 0).
	target_orbit:ADD("launch_az", 0).
	target_orbit:ADD("warp_dt", 0).
	target_orbit:ADD("mode", 1).
	
	//first compute in-plane orbital parameters
	
	//check altitudes
	
	IF target_orbit["periapsis"]>target_orbit["apoapsis"] {
		local ap_ is target_orbit["apoapsis"].
		set target_orbit["apoapsis"] to target_orbit["periapsis"].
		set target_orbit["periapsis"] to ap_.
	}
	
	IF NOT target_orbit:HASKEY("cutoff alt") {
		target_orbit:ADD("cutoff alt", target_orbit["periapsis"]).
	} ELSE {
		IF (target_orbit["cutoff alt"] < target_orbit["periapsis"]) {
			SET target_orbit["cutoff alt"] TO target_orbit["periapsis"].
		} ELSE IF (target_orbit["cutoff alt"] > target_orbit["apoapsis"]) {
			SET target_orbit["cutoff alt"] TO target_orbit["apoapsis"].
		}
	}
	
	SET target_orbit["sma"] TO orbit_appe_sma(target_orbit["apoapsis"], target_orbit["periapsis"]).
	SET target_orbit["ecc"] TO orbit_appe_ecc(target_orbit["apoapsis"], target_orbit["periapsis"]).
	
	LOCAL cutoff_r IS target_orbit["cutoff alt"]*1000 + SHIP:BODY:RADIUS.
	
	//compute cutoff orbital parameters
	
	SET target_orbit["eta"] TO orbit_alt_eta(cutoff_r, target_orbit["sma"], target_orbit["ecc"]).
	set target_orbit["velocity"] to orbit_alt_vel(cutoff_r, target_orbit["sma"]).
	
	set_vehicle_traj_steepness(target_orbit["cutoff alt"]).


	// now compute orbital plane

	// check inclination 
	//overridden in case of targeted launch
	//negative for southerly launches
	IF NOT target_orbit:HASKEY("inclination") {
		target_orbit:ADD("inclination", SHIP:GEOPOSITION:LAT).
	} ELSE {
		IF ABS(target_orbit["inclination"])<SHIP:GEOPOSITION:LAT {
			SET target_orbit["inclination"] TO SIGN(target_orbit["inclination"])*SHIP:GEOPOSITION:LAT.
		}
	}
	
	
	//compute lan given the various options
	
	//the second check only filters targets in earth orbit e.g. no interplanetary targets
	IF (DEFINED moon_transfer) {
		SET target_orbit["direction"] TO "north".
		moon_shot().
	} ELSE IF (HASTARGET = TRUE AND TARGET:BODY = SHIP:BODY) {
		SET target_orbit["LAN"] TO TARGET:ORBIT:LAN.
		SET target_orbit["inclination"] TO TARGET:ORBIT:INCLINATION.	
		
	} ELSE {
		IF target_orbit["inclination"] < 0 {
			SET target_orbit["direction"] TO "south".
			SET target_orbit["inclination"] TO ABS(target_orbit["inclination"]).	
		} ELSE {
			SET target_orbit["direction"] TO "north".
		}
		SET target_orbit["LAN"] TO LAN_orbit_overhead(target_orbit["inclination"], (target_orbit["direction"]="south"), vehicle["launchTimeAdvance"] + vehicle_countdown + 1).
	}
	
	//handle nearest launch window
	//compute nearest launch direction
	IF (target_orbit["direction"]="nearest") {
			
		LOCAL shiplngvec IS VXCL(V(0,1,0), -SHIP:ORBIT:BODY:POSITION):NORMALIZED.
		
		LOCAL dlng IS get_a_bBB(SHIP:GEOPOSITION:LAT, target_orbit["inclination"]).
		
		LOCAL north_launch_vec IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), -(target_orbit["LAN"] - dlng)).
		LOCAL south_launch_vec IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), -(target_orbit["LAN"] + 180 - dlng)).

		//arrow_body(north_launch_vec, "north").
		//arrow_body(south_launch_vec, "south").
		//arrow_body(shiplngvec, "ship").
		
		LOCAL north_dlan IS signed_angle(shiplngvec, north_launch_vec, V(0,1,0), 1).
		LOCAL south_dlan IS signed_angle(shiplngvec, south_launch_vec, V(0,1,0), 1).
		
		//print " north_dlan " + north_dlan at (0,61).
		//print " south_dlan " + south_dlan at (0,62).
		
		IF (south_dlan < north_dlan) {
			SET target_orbit["direction"] TO "south". 
		} ELSE {
			SET target_orbit["direction"] TO "north". 
		}
	}
	
	//time to window
	LOCAL time2window IS orbitInterceptTime(target_orbit["inclination"], target_orbit["LAN"], (target_orbit["direction"]="south")).
	
	//if launching into plane of target account for J2 nodal precession
	IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
		LOCAL t2w IS time2window.
		LOCAL j2LNG is -1.5*1.08262668e-3*rad2deg((BODY:RADIUS/(TARGET:ORBIT:SEMIMAJORAXIS*(1 - TARGET:ORBIT:ECCENTRICITY^2)))^2*SQRT(BODY:MU/TARGET:ORBIT:SEMIMAJORAXIS^3)*COS(TARGET:ORBIT:INCLINATION)).
		LOCAL lan_old IS target_orbit["LAN"].
		UNTIL FALSE {
			//print ltt_old AT (0,54).
			SET target_orbit["LAN"] TO  fixangle(lan_old +  j2LNG*t2w ).
			LOCAL t2w_new IS orbitInterceptTime(target_orbit["inclination"], target_orbit["LAN"], (target_orbit["direction"]="south")).
			
			//print ltt_new AT (0,55).
			
			IF ABS(t2w - t2w_new)<0.05 {
				BREAK.
			}
			SET t2w TO t2w_new.
			SET target_orbit["inclination"] TO TARGET:ORBIT:INCLINATION.	
		}
		SET time2window TO t2w.
	}
	
	SET time2window TO time2window - vehicle["launchTimeAdvance"].
	
	IF (time2window < vehicle_countdown) {
		SET time2window TO time2window + SHIP:BODY:ROTATIONPERIOD.
	}
	
	set target_orbit["warp_dt"] to time2window  - vehicle_countdown.
	
	//this is for message logging
	SET vehicle["ign_t"] TO TIME:SECONDS + time2window. 
	
	set target_orbit["launch_az"] to launchAzimuth(target_orbit["inclination"], target_orbit["velocity"], (target_orbit["direction"]="south")).	
	
	//print target_orbit:dump.
	//arrow_body(targetLANvec(target_orbit["LAN"]), "lan").
	//arrow_body(targetNormal(target_orbit["inclination"], target_orbit["LAN"]), "norm").
	//until false{}
		
	warp_window(target_orbit["warp_dt"]).	

}





//		NAVIGATION FUNCTIONS 



FUNCTION update_navigation {
	
	local t_prev is surfacestate["time"].
	SET surfacestate["time"] TO TIME:SECONDS.
	set surfacestate["deltat"] to surfacestate["time"] - t_prev.
	SET surfacestate["MET"] TO surfacestate["time"] - vehicle["ign_t"]. 
	
	
	//measure position and orbit parameters
	
	LOCAL progv IS v(0,0,0).
	
	IF vehiclestate["ops_mode"] >1 {set progv to SHIP:PROGRADE:VECTOR.}
	ELSE {set progv to SHIP:SRFPROGRADE:VECTOR.}
	
	SET surfacestate["hdir"] TO compass_for(progv,SHIP:GEOPOSITION ).
	SET surfacestate["vdir"] TO 90 - VANG(progv, SHIP:UP:VECTOR).
	SET surfacestate["pitch"] TO 90 - VANG(SHIP:FACING:VECTOR, SHIP:UP:VECTOR).	
	SET surfacestate["az"] TO compass_for(SHIP:FACING:VECTOR,SHIP:GEOPOSITION ).
	SET surfacestate["alt"] TO SHIP:ALTITUDE.
	SET surfacestate["vs"] TO SHIP:VERTICALSPEED.
	SET surfacestate["hs"] TO SHIP:VELOCITY:SURFACE:MAG.
	set surfacestate["q"] to SHIP:Q.
	
	SET orbitstate["velocity"] TO vecYZ(SHIP:ORBIT:VELOCITY:ORBIT).
	SET orbitstate["radius"] TO vecYZ(SHIP:ORBIT:BODY:POSITION)*-1.

}

