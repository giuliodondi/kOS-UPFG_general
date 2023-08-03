//GLOBAL NAV VARIABLES 

GLOBAL launchpad IS SHIP:GEOPOSITION.
GLOBAL surfacestate IS  LEXICON("MET",0,"az",0,"pitch",0,"alt",0,"vs",0,"hs",0,"vdir",0,"hdir",0,"q",0).
GLOBAL orbitstate IS  LEXICON("radius",0,"velocity",0). 



									//GENERAL NAVIGATION - ORBIT FUNCTIONS



FUNCTION update_navigation {
	
	SET surfacestate["MET"] TO TIME:SECONDS. 
	
	
	//measure position and orbit parameters
	
	IF vehiclestate["ops_mode"] >1 {set vel to SHIP:PROGRADE:VECTOR.}
	ELSE {set vel to SHIP:SRFPROGRADE:VECTOR.}
	
	SET surfacestate["hdir"] TO compass_for(vel,SHIP:GEOPOSITION ).
	SET surfacestate["vdir"] TO 90 - VANG(vel, SHIP:UP:VECTOR).
	SET surfacestate["pitch"] TO 90 - VANG(SHIP:FACING:VECTOR, SHIP:UP:VECTOR).	
	SET surfacestate["az"] TO compass_for(SHIP:FACING:VECTOR,SHIP:GEOPOSITION ).
	SET surfacestate["alt"] TO SHIP:ALTITUDE.
	SET surfacestate["vs"] TO SHIP:VERTICALSPEED.
	SET surfacestate["hs"] TO SHIP:VELOCITY:SURFACE:MAG.
	
	SET orbitstate["velocity"] TO vecYZ(SHIP:ORBIT:VELOCITY:ORBIT).
	SET orbitstate["radius"] TO vecYZ(SHIP:ORBIT:BODY:POSITION)*-1.

}





//	Target plane normal vector in MATLAB coordinates, UPFG compatible direction
FUNCTION targetNormal {
	DECLARE PARAMETER targetInc.	//	Expects a scalar
	DECLARE PARAMETER targetLan.	//	Expects a scalar
	
	//	First create a vector pointing to the highest point in orbit by rotating the prime vector by a right angle.
	LOCAL highPoint IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), 90-targetLan).
	//	Then create a temporary axis of rotation (short form for 90 deg rotation).
	LOCAL rotAxis IS V(-highPoint:Z, highPoint:Y, highPoint:X).
	//	Finally rotate about this axis by a right angle to produce normal vector.
	LOCAL normalVec IS rodrigues(highPoint, rotAxis, 90-targetInc).
	
	RETURN -vecYZ(normalVec).
}

//computes periapsis vector (normalised) given target orbit and longitude of periapsis
FUNCTION target_perivec {
	LOCAL peri IS v(0,0,0).
	
	IF target_orbit["mode"] = 2 {
		set peri to rodrigues(vecYZ(SOLARPRIMEVECTOR), V(0,0,1), target_orbit["LAN"]).
		set peri to rodrigues(peri, -target_orbit["normal"], target_orbit["periarg"]).
	}
	ELSE {
		SET peri TO target_orbit["radius"].
	}
	
	return peri:NORMALIZED.

}



FUNCTION cutoff_params {
	PARAMETER target.
	PARAMETER cutoff_r.
	PARAMETER etaa.
	
	LOCAL mode IS target["mode"].
	
	IF mode<>5 {
	
		local x is 1 + target["ecc"]*COS(etaa).
		
		local r_ is cutoff_r:MAG.
		
		IF mode=2{ //given sma, ecc and eta, compute r
			set r_ to target["SMA"]*(1-target["ecc"]^2)/x.	
		}
		ELSE IF mode=1 {	//given sma, ecc and r, compute eta
			IF target["ecc"]=0 {set etaa to  0.}
			ELSE {		
				set etaa to (target["SMA"]*(1-target["ecc"]^2)/r_ - 1)/target["ecc"].
				set etaa to ARCCOS(etaa).
			}
			set x to 1 + target["ecc"]*COS(etaa).
		}
		
		
		//ELSE IF mode=1 {	//given r, ecc and eta, compute sma
		//	SET target["SMA"] TO x*r_/(1-target["ecc"]^2).
		//}
	
		local v_ is SQRT(SHIP:BODY:MU * (2/r_ - 1/target["SMA"])).
			
		local phi is target["ecc"]*sin(etaa)/x.
		set phi to ARCTAN(phi).
		
		set target["radius"] to cutoff_r:NORMALIZED*r_.
		set target["velocity"] to v_.
		set target["angle"] to phi.
		set target["eta"] to etaa.
	}
	
	
	
	RETURN target.
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

	declare function cutoff_vec {
	
		PARAMETEr incl.

		
		LOCAL launchlat IS SHIP:GEOPOSITION:LAT.
		
		local k IS 367*COS(launchlat)*460.
		SEt k TO k/SHIP:BODY:RADIUS.
		
		LOCAl beta IS get_a_bBB(launchlat,incl).
		SET beta TO 90 - get_AA_aBB(beta,incl).
		
		LOCAL d IS get_b_cBB(k,beta).
		
		LOCAL a IS get_a_cBB(k,beta).
		
		local r_ IS rad2deg((2400000/SHIP:BODY:RADIUS)).
		SET a TO a + get_a_bc(d,r_).
		
		local x IS get_a_cBB(a,beta) - k.
		
		local alpha IS ARCCOS( limitarg(COS(r_)/COS(x)) ).
		SET alpha TO ARCSIN(limitarg(SIN(alpha)/SIN(r_))).
		
		
		local theta IS 90.
		IF target_orbit["direction"]="north" {
			SET theta TO theta + alpha.
		} ELSE IF target_orbit["direction"]="south" {
			SET theta TO theta - alpha.
		}
		
		LOCAL launchvec IS -SHIP:ORBIT:BODY:POSITION.
		
		LOCAL out IS VCRS(V(0,1,0),launchvec:NORMALIZED):NORMALIZED.
		SEt out TO rodrigues(out,launchvec:NORMALIZED,theta).
		
		
		SET out TO rodrigues(launchvec,out,r_).
	
		RETURN out.
	}



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
	//ALL NORMAL VECTORS NEED A VECYZ TO CONVERT THEM FROM UPFG FRAME TO KSP
	
	//initialise antipode vector
	LOCAL antipodevec IS (-moon_transfer["body"]:POSITION + SHIP:ORBIT:BODY:POSITION):NORMALIZED*SHIP:ORBIT:BODY:RADIUS*2.
	
	//arrow(antipodevec, "antipode_0",SHIP:ORBIT:BODY:POSITION,1,0.5).
	
	//find transfer time
	
	//find intersection point bw transfer orbit and lunar SOI given fixed patching parameters
	
	LOCAL lambda1 IS 58.2. //hardcoded value
	LOCAL r2 IS 66100000.
	LOCAL D_em IS (moon_transfer["body"]:POSITION - SHIP:ORBIT:BODY:POSITION):MAG.
	
	LOCAL r1 IS sqrt( D_em^2 + r2^2 - 2*d_em*r2*cos(lambda1) ).
	LOCAL gamma1 IS ARCSIN(r2*SIN(lambda1)/r1).
	
	//find transfer time between periapsis and intersection point
	local pe IS target_orbit["periapsis"]*1000 + SHIP:ORBIT:BODY:RADIUS.
	LOCAL ap IS moon_transfer["apoapsis"]*1000 + SHIP:ORBIT:BODY:RADIUS.

	LOCAL sma IS (pe+ap) / 2.
	
	LOCAL ecc IS (ap - pe)/(ap + pe).
	
	//true anomaly
	LOCAL cosnu IS limitarg((sma*(1 - ecc^2)/r1 - 1)/ecc).
	
	//transfer time between periapsis and encounter, plus countdown and launchtimeadvance time shifts.
	//still missing pre-launch warp time and parking orbit coast time
	//will compute them iteratively
	LOCAL tau0 IS  eta_to_dt(ARCCOS(cosnu),sma,ecc) + vehicle["launchTimeAdvance"] + 10.1.


	
	LOCAL moon_orbit IS LEXICON (	
							"SMA",moon_transfer["body"]:ORBIT:SEMIMAJORAXIS,
							"ecc",moon_transfer["body"]:ORBIT:ECCENTRICITY,
							"inclination",moon_transfer["body"]:ORBIT:INCLINATION,													
							"LAN",moon_transfer["body"]:ORBIT:LAN,
							"normal",V(1,0,0)
	) . 
	
	SET moon_orbit["normal"] TO targetNormal(ABS(moon_orbit["inclination"]), moon_orbit["LAN"]).
	

	//rotate antipode the first time by at least the transfer time
	//find the MASTER antipode on which the rest is based
	LOCAL rota IS t_to_eta(moon_transfer["body"]:ORBIT:TRUEANOMALY,tau0,moon_orbit["SMA"],moon_orbit["ecc"]) + moon_transfer["lead_angle"].
	SET rota TO rota -moon_transfer["body"]:ORBIT:TRUEANOMALY + moon_transfer["lead_angle"] + gamma1.
	SET antipodevec TO rodrigues(antipodevec, vecYZ(moon_orbit["normal"]), rota). 
	

	
	//LAN and injection vector given inclination and "right now" launch
	//LOCAL injectionvec IS cutoff_vec(target_orbit["inclination"]).
	LOCAL injectionvec IS VXCL( vecYZ(target_orbit["normal"]) ,- SHIP:ORBIT:BODY:POSITION ).
	
	
	
	IF NOT target_orbit:HASKEY("LAN") {
		target_orbit:ADD("LAN",LAN_orbit_overhead()).
	}
	
	SET target_orbit["normal"] TO targetNormal(ABS(target_orbit["inclination"]), target_orbit["LAN"]).
	
	IF NOT target_orbit:HASKEY("Longitude of Periapsis") {
		target_orbit:ADD("Longitude of Periapsis",0).
	}
	
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
	set deltalng_lan TO deltalng_lan - moon_orbit["LAN"].
	
	PRINT "difference of longitude of antipode and moon lan: " + deltalng_lan AT (0,48).
	
	LOCAL asc IS TRUE.
	IF ABS(deltalng_lan)>90 {set asc to FALSE.} 
	PRINT "moon ascending: " + asc AT (0,49).
	
	//should TLI occur close to the parking orbit's ascending or descending node?
	LOCAL ldn IS FALSE.
	
	IF (moon_transfer["rel_angle"]="low") {
		IF NOT asc {SET ldn TO TRUE.}
	}
	ELSE IF (moon_transfer["rel_angle"]="high") {
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
	
	
	//time factors
	GLOBAL tli_wait IS 3600.
	LOCAL warp_dt IS 0.
	LOCAL pkobt_t Is 2*CONSTANT:PI*SQRT(target_orbit["SMA"]^3/(SHIP:ORBIT:BODY:MU)).
	
	LOCAL epsilon IS 0.05.
	
	PRINT "ITERATING TO FIND TARGET LAN" AT (0,11).
	
	lOCAL dLNG IS 0.
	
	UNTIL FALSE {
		//adjust "time ahead"
		LOCAL tau IS tau0 + tli_wait + warp_dt.
		
		
		
		//current antipode vector
		//LOCAL antipodevec IS -moon_transfer["body"]:POSITION + SHIP:BODY:POSITION.
		LOCAL antipodevec IS (-moon_transfer["body"]:POSITION + SHIP:ORBIT:BODY:POSITION):NORMALIZED*SHIP:ORBIT:BODY:RADIUS*2.
		
		
		
		//rotate antipode given time ahead
		SET rota TO t_to_eta(moon_transfer["body"]:ORBIT:TRUEANOMALY,tau,moon_orbit["SMA"],moon_orbit["ecc"]).
		SET rota TO rota -moon_transfer["body"]:ORBIT:TRUEANOMALY + moon_transfer["lead_angle"] + gamma1.
		SET antipodevec TO rodrigues(antipodevec, vecYZ(moon_orbit["normal"]), rota). 
		print "lunar rot angle between now and arrival: " + rota  at (0,53).
		
		
	
		//TLI vector initialised as ascending node and if appropriate turned into descending node
		
		LOCAL tlivec IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), -target_orbit["LAN"]):NORMALIZED*SHIP:ORBIT:BODY:RADIUS*2..
		IF ldn {SET tlivec TO -tlivec.}

	
		//rotate TLI vector given antipode's latitude
		LOCAL antipodelat IS ABS(BODY:GEOPOSITIONOF(antipodevec + SHIP:BODY:POSITION):LAT).
		SET rota TO get_c_bBB(antipodelat,target_orbit["inclination"]).

		SET tlivec TO rodrigues(tlivec, vecYZ(target_orbit["normal"]), tli_s*rota). 
		
		
		
		//set longitude of periapsis to longitude of TLI
		local pveclng IS VXCL(V(0,1,0),tlivec).
		SET target_orbit["Longitude of Periapsis"] TO signed_angle(SOLARPRIMEVECTOR, pveclng , v(0,1,0),1).		

		//determine whether the antipode lies close enough to the current orbital plane
		//by computing longitude difference bw antipode and TLI vector
		//which are at the same latitude by construction	
		SET dLNG TO signed_angle(tlivec, antipodevec , v(0,1,0),0).	
		
		//PRINT "angle bw tlivec and antipode: " + dLNG AT (0,55).
				
		//SET dLNG TO project_angle(dLNG,antipodelat).

		
		PRINT "delta_longitude: " + dLNG AT (0,56).
		
		//if so exit, the orbit plane has been completely determined
		IF ABS(dLNG) <= epsilon {BREAK.}

		//if not rotate the entire plane to intersect the antipode vector
		
		SET target_orbit["LAN"] TO fixangle(target_orbit["LAN"] - dLNG ).
		SET injectionvec TO rodrigues(injectionvec,V(0,1,0),dLNG ).
		SET tlivec TO rodrigues(tlivec,V(0,1,0),dLNG).
		SET target_orbit["normal"] TO targetNormal(ABS(target_orbit["inclination"]), target_orbit["LAN"]).
		
		local xxx is signed_angle(tlivec, antipodevec , v(0,1,0),0).	
		
		PRINT xxx AT (0,57).
		
		//compute warp time to this new plane
		SET warp_dt TO orbitInterceptTime() .

		
		//estimate coast time between injection and TLI
		SET rota TO signed_angle(injectionvec,tlivec ,vecYZ(target_orbit["normal"]) ,1).
		//remember that we assume the periapsis is at tli.
		SET tli_wait TO eta_to_dt(rota,target_orbit["SMA"],target_orbit["ecc"]).
		print "parking obt coast angle:" + rota at (0,59).

		//if TLI is less than half an orbit away add one full orbit
		IF rota<180 {
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
	PRINT "TLI SCHEDULED FOR " + sectotime(tli_wait) + " AFTER INJECTION" AT (0,11).
	//until false {}.

}

//the planetary flyby routine fixes the orbital plane at given inclination that intersects a given target position after a given transfer time 
//it's basically the same calculations as the moon shot routine, only we expect the transfer time to be much larger than the parking orbit time or the warp time 
//therefore we neglect those times and do an approximate calculation moving the target position by just the transfer time 
declare function planetary_flyby {

	
	//computes time taken from periapsis to given true anomaly
	//for differences of true anomalies call twice and subtract times

	
	declare function eta_to_dt {

		parameter etaa.
		parameter sma.
		parameter ecc.

		local COS_ee IS (ecc + COS(fixangle(etaa)))/(1 + ecc*COS(fixangle(etaa))).

		LOCAL ee IS ARCCOS(limitarg(COS_ee)).			

		LOCAL mean_an IS deg2rad(ee)  - ecc*SIN(ee).
		
		IF etaa>180 { SET mean_an TO 2*CONSTANT:PI - mean_an.}
		
		LOCAL n IS SQRT(sma^3/(SHIP:ORBIT:BODY:MU)).
		

		RETURN n*mean_an.
	}

	//given true anomaly at t0 and a time interval, computes new true anomaly
	//approximation correct at ecc^3
	
	declare function t_to_eta {
		parameter etaa0.
		parameter dt.
		parameter sma.
		parameter ecc.
		
		
		local COS_ee IS (ecc + COS(fixangle(etaa0)))/(1 + ecc*COS(fixangle(etaa0))). 
		LOCAL ee IS ARCCOS(limitarg(COS_ee)).

		LOCAL mean_an IS deg2rad(ee)  - ecc*SIN(ee).
		
		IF etaa0>180 { SET mean_an TO 2*CONSTANT:PI - mean_an.}
		

		LOCAL n IS SQRT(sma^3/(SHIP:ORBIT:BODY:MU)).
		
		SET mean_an TO mean_an + dt/n.
		
		local out is mean_an.
		
		SET mean_an TO  fixangle(rad2deg(mean_an)).

		SET out TO out + ecc*(2*SIN(mean_an) + 1.25*ecc*SIN(2*mean_an)).
		
		RETURN fixangle(rad2deg(out)).

	}
	
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


FUNCTION compute_periarg {

	local periarg is 0.
	local vnorm IS -target_orbit["normal"]:NORMALIZED.
	local lanvec is rodrigues(vecYZ(SOLARPRIMEVECTOR), V(0,0,1), target_orbit["LAN"]).
	local peri is V(0,0,0).
	
	IF target_orbit["mode"] = "orbit" {
	
		LOCAL tgtlong IS convert_long(fixangle(target_orbit["Longitude of Periapsis"]),1).		

		SET peri TO rodrigues(vecYZ(SOLARPRIMEVECTOR), V(0,0,1), tgtlong):NORMALIZED.
		LOCAL nvec1 IS VCRS(peri,V(0,0,1)):NORMALIZED.
		LOCAL nvec2 IS VXCL(nvec1,vnorm):NORMALIZED.
		SET peri TO VXCL(nvec2,peri):NORMALIZED.
		
		//LOCAL nvec2 IS (vnorm - VDOT(vnorm,nvec1)*nvec1):NORMALIZED.
		//SET peri TO (peri - VDOT(peri,nvec2)*nvec2).
	
	}
	ELSE {
		SET peri TO target_orbit["radius"].
	} 
	//set periarg to signed_angle(peri,lanvec,vnorm,1).
	set periarg to signed_angle(lanvec,peri,vnorm,1).
	return periarg.
}


FUNCTION LAN_orbit_overhead {

	IF target_orbit["direction"] = "nearest" { SET target_orbit["direction"] TO "north". }
	LOCAL currentNode IS nodeVector(target_orbit["inclination"], target_orbit["direction"]).
	LOCAL currentLan IS VANG(currentNode, SOLARPRIMEVECTOR).
	IF VDOT(V(0,1,0), VCRS(currentNode, SOLARPRIMEVECTOR)) < 0 { SET currentLan TO 360 - currentLan. }
	
	LOCAL LAN_out IS currentLan + (vehicle["launchTimeAdvance"]+ 10.1)*360/SHIP:ORBIT:BODY:ROTATIONPERIOD.
	
	RETURN LAN_out.

}


//	Ascending node vector of the orbit passing right over the launch site
FUNCTION nodeVector {
	DECLARE PARAMETER inc.				//	Inclination of the desired orbit. Expects a scalar.
	DECLARE PARAMETER dir IS "north".	//	Launch direction. Expects a string, either "north" or "south".
	
	//	From right spherical triangle composed of inclination, latitude and "b",
	//	which is angular difference between the desired node vector and projection
	//	of the vector pointing at the launch site onto the equatorial plane.
	LOCAL b IS TAN(90-inc)*TAN(SHIP:GEOPOSITION:LAT).
	SET b TO ARCSIN( MIN(MAX(-1, b), 1) ).
	LOCAL longitudeVector IS VXCL(V(0,1,0), -SHIP:ORBIT:BODY:POSITION):NORMALIZED.
	IF dir = "north" {
		RETURN rodrigues(longitudeVector, V(0,1,0), b).
	} ELSE IF dir = "south" {
		//	This can be easily derived from spherical triangle if one draws a half
		//	of an orbit, from node to node. It is obvious that distance from node to
		//	peak equals 90 degrees, and from that the following results.
		RETURN rodrigues(longitudeVector, V(0,1,0), 180-b).
	} ELSE {
		RETURN nodeVector(inc, "north").
	}
}





//	Time to next launch opportunity in given direction
FUNCTION orbitInterceptTime {
	DECLARE PARAMETER launchDir IS target_orbit["direction"].	//	Passing as parameter for recursive calls.
	
	//	Expects a global variable "target_orbit" as lexicon
	LOCAL targetInc IS target_orbit["inclination"].
	LOCAL targetLan IS target_orbit["LAN"].
	
	//	For "nearest" launch opportunity:
	IF launchDir = "nearest" {
		LOCAL timeToNortherly IS orbitInterceptTime("north").
		LOCAL timeToSoutherly IS orbitInterceptTime("south").
		IF timeToSoutherly < timeToNortherly {
			SET target_orbit["direction"] TO "south".
			RETURN timeToSoutherly.
		} ELSE {
			SET target_orbit["direction"] TO "north".
			RETURN timeToNortherly.
		}
	} ELSE {
		//	Tind the ascending node vector of an orbit of the desired inclination,
		//	that passes above the launch site right now.
		SET currentNode TO nodeVector(targetInc, launchDir).
		//	Then find the ascending node vector of the target orbit.
		LOCAL targetNode IS rodrigues(SOLARPRIMEVECTOR, V(0,1,0), -targetLan).
		//	Find the angle between them, minding rotation direction, and return as time.
		LOCAL nodeDelta IS VANG(currentNode, targetNode).
		LOCAL deltaDir IS VDOT(V(0,1,0), VCRS(targetNode, currentNode)).
		IF deltaDir < 0 { SET nodeDelta TO 360 - nodeDelta. }
		LOCAL deltaTime IS SHIP:ORBIT:BODY:ROTATIONPERIOD * nodeDelta/360.
		
		RETURN deltaTime.
	}
}

//	Launch azimuth to a given orbit
FUNCTION launchAzimuth {

	//	Expects global variables "target_orbit" as lexicons
	
	LOCAL targetInc IS target_orbit["inclination"].
	
	//internal flag for retrograde launches, northerly or southerly alike
	LOCAL retro IS (targetInc > 90).
	
	//flag for southerly launches 
	LOCAL southerly IS (target_orbit["direction"]="south").
	

	LOCAL targetVel IS target_orbit["velocity"]*COS(target_orbit["angle"]).				//	But we already have our desired velocity, however we must correct for the flight path angle (only the tangential component matters here)
	
	LOCAL siteLat IS SHIP:GEOPOSITION:LAT.
	
	//calculate preliminary inertial azimuth 
	LOCAL equatorial_angle IS targetInc.
	IF retro {
		SET equatorial_angle TO 180 - equatorial_angle.
	}
	
	LOCAL Binertial IS ABS(COS(equatorial_angle)/COS(siteLat)).
	SET Binertial TO ARCSIN(limitarg(Binertial)).
	
	//mirror the angle w.r.t. the local north direction for retrograde launches
	IF retro {
		SET Binertial TO - Binertial.
	}
	
	//mirror the angle w.r.t the local east direction for southerly launches
	IF southerly {
		SET Binertial TO 180 - Binertial.
	}
	
	SET Binertial TO fixangle(Binertial).	//get the inertial launch hazimuth
	
	
	
	//get launch azimuth angle wrt due east=0
	LOCAL Vbody IS (2*CONSTANT:PI*SHIP:BODY:RADIUS/SHIP:BODY:ROTATIONPERIOD)*COS(siteLat).
	LOCAL VrotX IS targetVel*SIN(Binertial)-Vbody.
	LOCAL VrotY IS targetVel*COS(Binertial).
	LOCAL azimuth IS ARCTAN2(VrotY, VrotX).
	//azimuth is the angle wrt the due east direction
	//transform it into an azimuth wrt the north direction
	//this will subtract from 90° if it's a positive angle, due north, and add to 90° if it's due south. wrap around 360°
	
	LOCAL out IS fixangle(90-azimuth).
	
	//implement range azimuth limitation
	LOCAL site_azrange IS LEXICON(
						"KSC",LEXICON(
								"position",LATLNG(28.61938,-80.70092),
								"min_az",35,
								"max_az",120
						),
						"Vandenberg",LEXICON(
								"position",LATLNG(34.67974,-120.53102),
								"min_az",147,
								"max_az",220	//250
						)
	
	).
	LOCAL shippos IS SHIP:GEOPOSITION.
	FOR s IN site_azrange:VALUES{
		LOCAL sitepos IS s["position"].
		
		//if the launchsite is within 50km of a known site
		//apply its range restrictions
		IF downrangedist(sitepos,shippos) < 50 {
			SET out TO CLAMP(out,s["min_az"],s["max_az"]).
			BREAK.
		}
	
	}
	
	RETURN out.
}

FUNCTION warp_window{

	parameter liftofftime.
	
	LOCAL timetolaunch IS liftofftime - TIME:SECONDS.
	

	UNTIL timetolaunch <=0.1 {
	
		SET timetolaunch TO liftofftime - TIME:SECONDS.
		
		IF timetolaunch > 3600 or timetolaunch < 0    {set warp to 4.}
		ELSE IF timetolaunch > 400     {set warp to 3.}
		ELSE IF timetolaunch > 60  {set warp to 2.}
		ELSE IF timetolaunch > 1  {set warp to 1.}
		ELSE 							{set warp to 0.}
		
		PRINT "                                                               " at (1,23).
		PRINT "	TIME TO WINDOW : " + sectotime(timetolaunch) at (1,23).
		Wait 0.
	}
	set warp to 0.

}



FUNCTION prepare_launch {
	
	PRINT " PREPARING TO LAUNCH " AT (0,5).
	
	target_orbit:ADD("radius", 0) .
	target_orbit:ADD("velocity", 0) .
	target_orbit:ADD("normal", V(0,0,0)) .
	target_orbit:ADD("angle", 0) .
	target_orbit:ADD("periarg", 0) .
	target_orbit:ADD("mode", 0) .
	
	
	PRINT " COMPUTING IN-PLANE TARGET ORBITAL PARAMETERS" AT (0,7).
	
	IF target_orbit["periapsis"]>target_orbit["apoapsis"] {
		local x is target_orbit["apoapsis"].
		set target_orbit["apoapsis"] to target_orbit["periapsis"].
		set target_orbit["periapsis"] to x.
	}
	
	LOCAL pe IS target_orbit["periapsis"]*1000 + SHIP:BODY:RADIUS.
	LOCAL ap IS target_orbit["apoapsis"]*1000 + SHIP:BODY:RADIUS.
	target_orbit:ADD("SMA", (pe+ap) / 2) .
	
	target_orbit:ADD("ecc", (ap - pe)/(ap + pe)).
	
	
	
	PRINT " COMPUTING TARGET ORBITAL PLANE" AT (0,9).
	
	
	IF NOT target_orbit:HASKEY("inclination") {
		target_orbit:ADD("inclination", ABS(SHIP:GEOPOSITION:LAT)).
	}
	
	
	//	Set default launch direction
	IF NOT target_orbit:HASKEY("direction") {
		target_orbit:ADD("direction", "nearest").
	}
	

	
	//IF NOT target_orbit:HASKEY("LAN") {
	//	target_orbit:ADD("LAN",0).
	//}
	//ELSE {
	//	SET target_orbit["LAN"] TO fixangle(target_orbit["LAN"]).
	//}
	
	
	IF DEFINED moon_transfer {	moon_shot().}
	ELSE IF DEFINED flyby_transfer {	planetary_flyby(). }
	ELSE {

	
		PRINT " COMPUTING TARGET LAN" AT (0,11).
	
		//the second check only filters targets in earth orbit e.g. no interplanetary targets
		IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
			IF NOT target_orbit:HASKEY("LAN") {
				target_orbit:ADD("LAN",0).
			}
			SET target_orbit["LAN"] TO TARGET:ORBIT:LAN.
			SET target_orbit["inclination"] TO TARGET:ORBIT:INCLINATION.	
			
		}
		ELSE {
				IF ABS(target_orbit["inclination"])<SHIP:GEOPOSITION:LAT {
					SET target_orbit["inclination"] TO SIGN(target_orbit["inclination"])*SHIP:GEOPOSITION:LAT.
				}
				
				IF target_orbit["inclination"] >= 0 {
						SET target_orbit["direction"] TO "north".
				}
				ELSE IF target_orbit["inclination"] < 0 {
					SET target_orbit["direction"] TO "south".
					SET target_orbit["inclination"] TO 	ABS(target_orbit["inclination"]).	
				}
				
			//SET target_orbit["LAN"] TO .
			IF NOT target_orbit:HASKEY("LAN") {
				target_orbit:ADD("LAN",LAN_orbit_overhead()).
			}
			ELSE {
				SET target_orbit["LAN"] TO convert_long(fixangle(target_orbit["LAN"]),1).
			}
			
		}
		SET target_orbit["normal"] TO targetNormal(ABS(target_orbit["inclination"]), target_orbit["LAN"]).
	}
	

	
	local vnorm is -target_orbit["normal"]:NORMALIZED.
	local cutvec is (vecYZ(SHIP:BODY:POSITION)*-1):NORMALIZED.
	set cutvec to (cutvec - VDOT(cutvec,vnorm)*vnorm):NORMALIZED.
	
	//arbitrarily set cutoff point at 30 degrees ahead of the launch position.
	set cutvec to rodrigues(cutvec,vnorm, 30):NORMALIZED.
	
	set target_orbit["radius"] TO cutvec.
	
	PRINT " COMPUTING TARGET ARGUMENT OF PERIAPSIS" AT (0,13).
	
	
	IF target_orbit:HASKEY("Longitude of Periapsis") {
			SET target_orbit["mode"] TO 2.
			SET target_orbit["periarg"] TO compute_periarg().
			target_orbit:REMOVE("Longitude of Periapsis").
	}
	ELSE {
		SET target_orbit["mode"] TO 1.
		LOCAL cut_alt IS target_orbit["periapsis"].
		IF target_orbit:HASKEY("Cutoff Altitude") {
			SET cut_alt TO target_orbit["Cutoff Altitude"].
			target_orbit:REMOVE("Cutoff Altitude").
		}
		IF cut_alt<target_orbit["periapsis"] {
			SET cut_alt TO target_orbit["periapsis"].
		}
		SET cut_alt TO (cut_alt*1000 + SHIP:BODY:RADIUS).
		set target_orbit["radius"] TO cutvec:NORMALIZED*cut_alt.
	}
	
	
	

	
	PRINT " COMPUTING PERIAPSIS VECTOR" AT (0,15).
	target_orbit:ADD("perivec", target_perivec()) .
	
	PRINT " ESTIMATE CUTOFF CONDITIONS" AT (0,17).
	local etaa is 0.

	IF target_orbit["mode"] = 2 {
		set etaa to signed_angle(target_orbit["perivec"],cutvec,vnorm,1).
	}
	IF target_orbit["mode"] = 1 {
		IF NOT target_orbit["ecc"]=0 {
			set etaa to (target_orbit["SMA"]*(1-target_orbit["ecc"]^2)/target_orbit["radius"]:MAG - 1)/target_orbit["ecc"].
			set etaa to ARCCOS(etaa).		
		}
	}
	target_orbit:ADD("eta", etaa) .
	SET target_orbit TO cutoff_params(target_orbit,target_orbit["radius"],etaa).
	
	PRINT " CALCULATING TIME TO LAUNCH " AT (0,19).	
	
	//	Calculate time to launch
	LOCAL timeToOrbitIntercept IS orbitInterceptTime().
	LOCAL liftoffTime IS TIME:SECONDS + timeToOrbitIntercept - vehicle["launchTimeAdvance"].
	
	
	//if launching into plane of target account for J2 nodal precession
	IF HASTARGET = TRUE AND (TARGET:BODY = SHIP:BODY) {
		LOCAL ltt_old IS liftoffTime.
		LOCAL j2LNG is -1.5*1.08262668e-3*rad2deg((BODY:RADIUS/(TARGET:ORBIT:SEMIMAJORAXIS*(1 - TARGET:ORBIT:ECCENTRICITY^2)))^2*SQRT(BODY:MU/TARGET:ORBIT:SEMIMAJORAXIS^3)*COS(TARGET:ORBIT:INCLINATION)).
		LOCAL lan_old IS target_orbit["LAN"].
		UNTIL FALSE {
			print ltt_old AT (0,54).
			SET target_orbit["LAN"] TO  fixangle(lan_old +  j2LNG*ltt_old ).
			LOCAL ltt_new IS orbitInterceptTime().
			print ltt_new AT (0,55).
			IF ABS(ltt_old - ltt_new)<0.05 {
				SET ltt_old TO ltt_new.
				BREAK.
			}
			SET ltt_old TO ltt_new.
			SET target_orbit["inclination"] TO TARGET:ORBIT:INCLINATION.	
		}
		SET liftoffTime TO TIME:SECONDS + ltt_old - vehicle["launchTimeAdvance"].
	}
	IF timeToOrbitIntercept < vehicle["launchTimeAdvance"] { SET liftoffTime TO liftoffTime + SHIP:BODY:ROTATIONPERIOD. }
	PRINT " CALCULATING LAUNCH AZIMUTH" AT (0,21).		
	set control["launch_az"] to launchAzimuth().	
		
	warp_window(liftoffTime).
	
		
	PRINT " COMPLETE. STARTING COUNTDOWN." AT (0,25).	
	
}	

	
	
	
	







