GLOBAL terminalwidth IS 65.
GLOBAL terminalheight IS 59.

SET TERMINAL:WIDTH TO terminalwidth.
SET TERMINAL:HEIGHT TO terminalheight.

FUNCTION addMessage {
	DECLARE PARAMETER msg.
	LOCAL tt IS TIME:SECONDS.
	LOCAL ttl IS 4.

	IF NOT( DEFINED P_vizMsg ) {
		GLOBAL P_vizMsg IS LIST().
	}
	local rem_list IS LIST().
	FROM {LOCAL k IS 0.} UNTIL k = P_vizMsg:LENGTH  STEP { SET k TO k+1.} DO{
		IF tt >= P_vizMsg[k]["ttl"] {
			rem_list:ADD(k).
		}
	}
	FROM {LOCAL k IS rem_list:LENGTH-1.} UNTIL k <0  STEP { SET k TO k-1.} DO{
		P_vizMsg:REMOVE(rem_list[k]).
	}
	
	P_vizMsg:INSERT(
					0,
					LEXICON(
							"msg","T+" + sectotime(TIME:SECONDS - vehicle["ign_t"],"") + ": " + msg,
							"ttl",tt + ttl
					)
	) .
}

FUNCTION message_Viz {
	LOCAL  msglist IS P_vizMsg:SUBLIST(0,P_vizMsg:LENGTH).
	LOCAl k IS 0.
	UNTIL FALSE {
		IF k>= P_vizMsg:LENGTH {BREAK.}
		PRINT "                                                             "AT (1,msgloc + k).
		IF TIME:SECONDS < P_vizMsg[k]["ttl"] {
			PRINTPLACE(P_vizMsg[k]["msg"],61,1,msgloc + k).
		}
		SEt k TO k+1.
	}
}


FUNCTION drawUI {

	SET TERMINAL:WIDTH TO terminalwidth.
	SET TERMINAL:HEIGHT TO terminalheight.

	IF NOT (DEFINED surfloc) {GLOBAL surfloc IS 17.}
	IF NOT (DEFINED orbloc) {GLOBAL orbloc IS 25.}
	IF NOT (DEFINED vehloc) {GLOBAL vehloc IS 36.}
	IF NOT (DEFINED msgloc) {GLOBAL msgloc IS 46.}
	

	CLEARSCREEN.
	
	PRINT "|=============================================================|"  AT (0,1).
	PRINT "|         UNIFIED POWERED FLIGHT GUIDANCE ALGORITHM           |"  AT (0,2).
	PRINT "|                                                             |"  AT (0,3).
	PRINT "|                                                             |"  AT (0,4).
	PRINT "|                                                             |"  AT (0,5).
	PRINT "|                                                             |"  AT (0,6).
	PRINT "|                                                             |"  AT (0,7).
	PRINT "|=============================================================|"  AT (0,8).
	PRINT "|                                                             |"  AT (0,9).
	PRINT "|                    M.E.T.      :                            |"  AT (0,10).
	PRINT "|                                                             |"  AT (0,11).
	PRINT "|                                                             |"  AT (0,12).
	PRINT "|                                                             |"  AT (0,13).
	PRINT "|=============================================================|"  AT (0,14).
	PRINT "|                        SURFACE DATA                         |" AT (0,15).
	PRINT "|                                                             |" AT (0,16).
	PRINT "|  SURFACE ALT    :               DOWNRANGE DST  :            |" AT (0,17).
	PRINT "|  VERTICAL SPD   :               HORIZ SPD      :            |" AT (0,18).
	PRINT "|  SURF PITCH     :               INERT AZIMUTH  :            |" AT (0,19).
	PRINT "|  VERTICAL AOA   :               HORIZONTAL AOA :            |" AT (0,20).
	PRINT "|                                                             |" AT (0,21).
	PRINT "|=============================================================|" AT (0,22).
	PRINT "|      CURRENT ORBIT DATA      |       TARGET ORBIT DATA      |" AT (0,23).
	PRINT "|                              |                              |" AT (0,24).
	PRINT "|    APOAPSIS     :            |    APOAPSIS     :            |" AT (0,25).
	PRINT "|    PERIAPSIS    :            |    PERIAPSIS    :            |" AT (0,26).
	PRINT "|   ORB VELOCITY  :            |   ORB VELOCITY  :            |" AT (0,27).
	PRINT "|   INCLINATION   :            |   INCLINATION   :            |" AT (0,28).
	PRINT "|       LAN       :            |       LAN       :            |" AT (0,29).
	PRINT "|   TRUE ANOMALY  :            |   TRUE ANOMALY  :            |" AT (0,30).
	PRINT "|   RELATIVE ANGLE:            |    CUTOFF ALT   :            |" AT (0,31).
	PRINT "|                              |                              |" AT (0,32).
	PRINT "|=============================================================|" AT (0,33).
	PRINT "|         VEHICLE DATA         |     UPFG GUIDANCE DATA       |" AT (0,34).
	PRINT "|                              |                              |" AT (0,35).
	PRINT "|   TOT BURN TIME :            |                              |" AT (0,36).
	PRINT "|   CURRENT TWR   :            |                              |" AT (0,37).
	PRINT "|   CMD THROTTLE  :            |                              |" AT (0,38).
	PRINT "|   CURRENT STG   :            |                              |" AT (0,39).
	PRINT "|   STG TYPE      :            |                              |" AT (0,40).
	PRINT "|   STG REM TIME  :            |                              |" AT (0,41).
	PRINT "|                              |                              |" AT (0,42).
	PRINT "|=============================================================|" AT (0,43).
	PRINT "|                         MESSAGE BOX:                        |" AT (0,44).
	PRINT "|                                                             |" AT (0,45).
	PRINT "|                                                             |" AT (0,46).
	PRINT "|                                                             |" AT (0,47).
	PRINT "|                                                             |" AT (0,48).
	PRINT "|                                                             |" AT (0,49).
	PRINT "|                                                             |" AT (0,50).
	PRINT "|                                                             |" AT (0,51).
	PRINT "|                                                             |" AT (0,52).
	PRINT "|                                                             |" AT (0,53).	
	PRINT "|                                                             |" AT (0,54).	
	PRINT "|                                                             |" AT (0,55).	
	PRINT "|                                                             |" AT (0,56).	
	PRINT "|=============================================================|" AT (0,57).	
	//PRINT "|                                                             |" AT (0,58).	
	//PRINT "|                                                             |" AT (0,59).	
	//PRINT "|                                                             |" AT (0,60).	
	//PRINT "|                                                             |" AT (0,61).	
	//PRINT "|=============================================================|" AT (0,62).	
	//PRINT "" AT (0,63).
	
	

	
	IF (vehiclestate["ops_mode"] =2) OR (vehiclestate["ops_mode"] =3){	
	
		PRINT "    S_MODE     : "	AT (32,vehloc).
		PRINT "    STATUS     : "	AT (32,vehloc+1).
		//PRINT "   UPFG STG    : "	AT (32,vehloc+2).
		PRINT "   STG MODE    : "	AT (32,vehloc+3).
		PRINT "     T_GO      : " AT (32,vehloc+4).	
		PRINT "     V_GO      : " AT (32,vehloc+5).
		
		
	}
	ELSE {
		PRINT "DATA  NOT" AT (42,vehloc+1).
		PRINT "AVAILABLE" AT (42,vehloc+3).
	}
	
	IF vehiclestate["ops_mode"] = 4 {
		
		PRINTPLACE("TIME TAKEN: " + sectotime(TIME:SECONDS - vehicle["ign_t"]),61,1,msgloc).
		PRINTPLACE("FINAL ORBITAL PARAMETERS:",61,1,msgloc + 2).
		PRINTPLACE("APOAPSIS: " + ROUND(APOAPSIS/1000,1) + " km  |  PERIAPSIS: " + ROUND(PERIAPSIS/1000,1) + " km",61,1,msgloc + 4).
		PRINTPLACE("INCLINATION:  " + ROUND(ORBIT:INCLINATION,3) + " deg  |  LAN: " + ROUND(ORBIT:LAN,3) + " deg  |  TRUE ANOM.: " + ROUND(ORBIT:TRUEANOMALY,2) + " deg",61,1,msgloc + 6).
		
		local str is "Ap err: " + ROUND(abs((1 - APOAPSIS/(target_orbit["Apoapsis"]*1000))*100),3) + "% ".
		set str to str + "| Pe err: " + ROUND(abs((1 - PERIAPSIS/(target_orbit["Periapsis"]*1000))*100),3) + "%".
		
		PRINTPLACE(str,61,1,msgloc + 8).
		
		set str to "Incl err: " + ROUND(abs((1 - ORBIT:INCLINATION/target_orbit["Inclination"])*100),3) + "% ".
		set str to str + "| LAN err: " + ROUND(abs((1 - ORBIT:LAN/target_orbit["LAN"])*100),3) + "% ".
		set str to str + "| True Anomaly err: " + ROUND(abs(ORBIT:TRUEANOMALY - target_orbit["eta"] ),2) + "deg ".

		PRINTPLACE(str,61,1,msgloc + 10).
		PRINTPLACE("PRESS AG9 TO END THE PROGRAM.",61,1,64).
	
	}
	
}




//				outputs all flight information to the screen
//				requires flight sequence given by P_seq defined as GLOBAL and of type LIST
//				capable of displaying custom messages, set up by addMessage as a GLOBAL STRING with a FLOAT time to live
FUNCTION dataViz {
	if (vehiclestate["ops_mode"] =0) {return.}
	
	log_telemetry().
	
	//PRINTING VARIABLES IN THE CORRECT PLACE
	
	PRINTPLACE("VEHICLE : " + vehicle["name"],61,1,4).	
	IF HASTARGET = TRUE {       
		PRINTPLACE("TARGET BODY/VESSEL : " + TARGET:NAME,61,1,6).
	} 
	
	//MET
	PRINTPLACE(sectotime(TIME:SECONDS - vehicle["ign_t"]),12,34,10).
	
	//Status
	LOCAL vehstatus IS "CURRENT STATUS : ".
	IF vehiclestate["ops_mode"] =0 { SET vehstatus TO vehstatus + "INITIAL CLIMB".}
	ELSE IF vehiclestate["ops_mode"] =1 { SET vehstatus TO vehstatus + "OPEN LOOP ASCENT".}
	ELSE IF vehiclestate["ops_mode"] =2 { SET vehstatus TO vehstatus + "CLOSED LOOP GUIDANCE".}
	ELSE IF vehiclestate["ops_mode"] =3 { SET vehstatus TO vehstatus + "TERMINAL GUIDANCE".}
	ELSE IF vehiclestate["ops_mode"] =4 { SET vehstatus TO vehstatus + "GUIDANCE TERMINATED". }
	PRINTPLACE(vehstatus ,61,1,12).

	//surface data
	PRINTPLACE(ROUND(SHIP:ALTITUDE/1000 , 2) + " km",12,19,surfloc).
	PRINTPLACE(ROUND(SHIP:VERTICALSPEED,1) + " m/s",12,19,surfloc+1).
	
	PRINTPLACE(ROUND(downrangedist(launchpad,SHIP:GEOPOSITION ),2) + " km",12,50,surfloc).
	PRINTPLACE(ROUND(SHIP:GROUNDSPEED,1) + " m/s",12,50,surfloc+1).
	
	local vel is 0.
	
	IF vehiclestate["ops_mode"] >1 {set vel to SHIP:PROGRADE:VECTOR.}
	ELSE {set vel to SHIP:SRFPROGRADE:VECTOR.}

	PRINTPLACE(ROUND(90 - VANG(SHIP:FACING:VECTOR, SHIP:UP:VECTOR),2) + " deg",12,19,surfloc+2).
	PRINTPLACE(ROUND(compass_for(SHIP:FACING:VECTOR,SHIP:GEOPOSITION ),2) + " deg",12,50,surfloc+2).
	PRINTPLACE(ROUND(VANG(vel, SHIP:UP:VECTOR) - VANG(SHIP:FACING:VECTOR, SHIP:UP:VECTOR),3) + " deg",12,19,surfloc+3).
	PRINTPLACE(ROUND(compass_for(vel,SHIP:GEOPOSITION ) - compass_for(SHIP:FACING:VECTOR,SHIP:GEOPOSITION ),3) + " deg",12,50,surfloc+3).
	
	
	//orbital data
	PRINTPLACE(ROUND(APOAPSIS/1000,1) + " km",12,19,orbloc).
	PRINTPLACE(ROUND(PERIAPSIS/1000,1)+ " km",12,19,orbloc + 1).
	PRINTPLACE(ROUND(SHIP:VELOCITY:ORBIT:MAG,1) + " m/s",12,19,orbloc + 2).	
	PRINTPLACE(ROUND(ORBIT:INCLINATION,3) + " deg",12,19,orbloc + 3).
	PRINTPLACE(ROUND(ORBIT:LAN,2) + " deg",12,19,orbloc + 4).
	PRINTPLACE(ROUND(ORBIT:TRUEANOMALY,2) + " deg",12,19,orbloc + 5).
	

	
	LOCAL currentOrbitNormal IS targetNormal(SHIP:ORBIT:INCLINATION, SHIP:ORBIT:LAN).
	LOCAL tgtnormal IS targetNormal(target_orbit["inclination"], target_orbit["LAN"]).
	PRINTPLACE(ROUND(VANG(currentOrbitNormal, currentOrbitNormal),3) + " deg",12,19,orbloc + 6).
	
	
	//TARGET ORBIT DATA 
	
	PRINTPLACE(ROUND(target_orbit["Apoapsis"],2)+ " km",12,50,orbloc).
	PRINTPLACE(ROUND(target_orbit["Periapsis"],2)+ " km",12,50,orbloc + 1).
	PRINTPLACE(ROUND(target_orbit["velocity"],1) + " m/s",12,50,orbloc + 2).	
	PRINTPLACE(ROUND(target_orbit["inclination"],4) + " deg",12,50,orbloc + 3).
	PRINTPLACE(ROUND(target_orbit["LAN"],2) + " deg",12,50,orbloc + 4).
	PRINTPLACE(ROUND(target_orbit["eta"],2) + " deg",12,50,orbloc + 5).
	PRINTPLACE(ROUND(target_orbit["cutoff alt"],2) + " km",12,50,orbloc + 6).
	
	
	
	
	
	
	//vehicle data
	
	LOCAL cur_stg_idx IS vehiclestate["cur_stg"].
	
	LOCAL total_stg_time IS 0.
	FOR s IN vehicle["stages"]:SUBLIST(cur_stg_idx,vehicle["stages"]:LENGTH - cur_stg_idx) {
		SET total_stg_time TO total_stg_time + s["Tstage"].
	}
	
	PRINTPLACE(sectotime(total_stg_time),12,19,vehloc).
	PRINTPLACE(ROUND(get_TWR(),2) + " ",12,19,vehloc+1).
	PRINTPLACE(ROUND(THROTTLE*100,1) + " %",12,19,vehloc+2).
	
	PRINTPLACE(" " + cur_stg_idx + " ",12,19,vehloc + 3).
	
	LOCAL stg IS get_stage().
	
	PRINTPLACE(stg["staging"]["type"],12,19,vehloc+4).
	PRINTPLACE(sectotime(stg["Tstage"]),12,19,vehloc+5).

	
	
	//upfg data
	IF (vehiclestate["ops_mode"] =2) OR (vehiclestate["ops_mode"] =3) {
		
		PRINTPLACE(target_orbit["mode"],12,50,vehloc).
	
		IF usc["conv"]=1 { PRINTPLACE("CONVERGED",12,50,vehloc+1). }
		ELSE IF usc["conv"]>-4 { PRINTPLACE("CONVERGING",12,50,vehloc+1). }
		ELSE { PRINTPLACE("NOT CONVERGED",12,50,vehloc+1). }
		
		//PRINTPLACE(upfg_stg,12,50,vehloc+2).
		
		IF stg["mode"]=1 {
			PRINTPLACE("CONST T",12,50,vehloc+3).
		}
		ELSE IF stg["mode"]=2 {
			PRINTPLACE("CONST G",12,50,vehloc+3).
		}
		IF DEFINED (upfgInternal) {
			PRINTPLACE(sectotime(upfgInternal["Tgo"]),12,50,vehloc+4). 
			PRINTPLACE(ROUND(upfgInternal["vgo"]:MAG,0),12,50,vehloc+5). 
		}
		
		

	}
	
	//messages
	message_Viz().

}



FUNCTION prepare_telemetry {
	if logdata=TRUE {	
		GLOBAL loglex IS LEXICON(
										"Time",0,
										"Lat",0,
										"Lng",0,
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
										"Vspeed",0,
										"Incl",0,
										"Ecctr",0
		).
		log_data(loglex,"./UPFG_general/LOGS/" + vehicle["name"] + "_log", TRUE).
	}
}

FUNCTION log_telemetry {
	if logdata=TRUE {
		LOCAL stg IS get_stage().
		
		//prepare list of values to log.
		
		SET loglex["Time"] TO TIME:SECONDS - vehicle["ign_t"].
		SET loglex["Lat"] TO SHIP:GEOPOSITION:LAT.
		SET loglex["Lng"] TO SHIP:GEOPOSITION:LNG.
		SET loglex["Altitude"] TO SHIP:ALTITUDE/1000.
		SET loglex["Dwnrg Dst"] TO downrangedist(launchpad,SHIP:GEOPOSITION ).
		SET loglex["Stage"] TO vehiclestate["cur_stg"].
		SET loglex["Mass"] TO stg["m_initial"].
		SET loglex["TWR"] TO get_TWR().
		SET loglex["Throt"] TO stg["Throttle"]*100.
		SET loglex["AZ(cmd)"] TO surfacestate["hdir"].
		SET loglex["HAOA"] TO get_yaw_prograde().
		SET loglex["Pitch"] TO surfacestate["vdir"].
		SET loglex["VAOA"] TO get_pitch_prograde().
		SET loglex["Surfvel"] TO SHIP:VELOCITY:SURFACE:MAG.
		SET loglex["Orbvel"] TO SHIP:VELOCITY:ORBIT:MAG.
		SET loglex["Vspeed"] TO SHIP:VERTICALSPEED.
		SET loglex["Incl"] TO ORBIT:INCLINATION.
		SET loglex["Ecctr"] TO ORBIT:ECCENTRICITY.

		log_data(loglex).
	}
}
 