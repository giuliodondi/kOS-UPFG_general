//multi-rev Lambert Req Vel Det Routine

FUNCTION mrlrvdr {
	


	//Lambert Transfer Time Interval Subroutine
	
	FUNCTION ltti {
		PARAMETER Gamma0.
		PARAMETER p_1.
		PARAMETER p_2.
		PARAMETER sinth.
		PARAMETER costh.
		PARAMETER r0.
		PARAMETER nrev.
		
		LOCAL p_N IS p_1/(Gamma0*sinth - p_2) .
		LOCAL alpha_N IS 2 - p_N*(1+Gamma0^2).
		
		LOCAL pack IS meir(sinth,costh,Gamma0,r0,alpha_N,p_N).
		LOCAL xx IS pack[0].
		LOCAL xi IS pack[1].
		LOCAL c1 IS pack[2].
		LOCAL c2 IS pack[3].
		pack:CLEAR().
		
		LOCAL pack IS ukes(c1,c2,xx,xi,r0).
		LOCAL dtc IS pack[0].
		LOCAL s_xi IS pack[1].
		LOCAL c_xi IS pack[2].
		pack:CLEAR().
		
		IF nrev<>0 {
		
			SET dtc TO dtc + nrev*2*Constant:PI*SQRT(ABS(r0/alpha_N)^3/bodymu);
		
		}
		
		RETURN LIST(dtc,alpha_N,p_N,xx,xi,s_xi,c_xi).
	}
	
	//Universal Kepler Equation Subroutine
	
	FUNCTION ukes {
		PARAMETER c1.
		PARAMETER c2.
		PARAMETER xx.
		PARAMETER xi.
		PARAMETER r0.
		
		
		LOCAL s_xi IS ((xi/42 - 1)*xi/20 + 1)/6.
		LOCAL c_xi IS ((xi/30 - 1)*xi/12 + 1)/2.
		
		LOCAL a IS xx^2.
		
		LOCAL dtc IS ( c1*c_xi*a + xx*(c2*s_xi*a + r0) )/SQRT(bodymu).
		
	RETURN LIST(dtc,s_xi,c_xi).
	}
	
	//marscher equation inversion subroutine
	
	FUNCTION meir 	{
	
		PARAMETER sinth.
		PARAMETER costh.
		PARAMETER Gamma0.
		PARAMETER r0.
		PARAMETER alpha_N.
		PARAMETER p_N.
		
		LOCAL w1 IS SQRT(p_N)*( sinth/(1 - costh) - Gamma0 ).
		
		IF (alpha_N <=0) {
			IF (w1<=0) OR ((w1<0) AND (w1^2 + alpha_N <=0))
				print "marscher has no solution" at (1,61).
				RETURN 1/0.
		}
		
		LOCAL b IS 0.
		LOCAL wN IS w1.
		IF (ABS(wN)<=1) {
			FROM {LOCAL n IS 1.} UNTIL n = 4 STEP{SET n TO n+1.} DO{	
				SET wN TO SQRT(wN^2 + alpha_N) + ABS(wN).
			}
			SET b TO 1/wN.
		}
		ELSE {
			LOCAL a IS ABS(1/wN).
			LOCAL vN IS 1.
			FROM {LOCAL n IS 1.} UNTIL n = 4 STEP{SET n TO n+1.} DO{
				SET vN TO SQRT(vN^2 + alpha_N*a) + vN.
			}
			SET b TO a/vN.
		}
		
		LOCAL xN IS 0.
		
		FROM {LOCAL j IS 0.} UNTIL j = 10 STEP{SET j TO j+1.} DO{
			SET xN TO xN + ((-alpha_N*(b^2))^j)/(2*j + 1)
		
		}
		
		SET xN TO 16*b*xN.
		
		IF (w1<=0) {
			SET xN TO 2*Constant:PI/SQRT(alpha_N) - xN.
		}
		
		RETURN LIST(alpha_N*xN^2 , SQRT(r0)*xN , SQRT(r0*p_N)*Gamma0 , 1-alpha_N).
	}

	//Secant Iterator Routine
	
	FUNCTION secant_iter {
	
		PARAMETER s.
		PARAMETER dtc.
		PARAMETER dtc_p.
		PARAMETER terr.
		PARAMETER dGamma0.
		PARAMETER Gamma0.
		PARAMETER Gammamin.
		PARAMETER Gammamax.
		PARAMETER k.
		PARAMETER minflag.
		
		IF (s=1) {
			SET s TO 0.
			SET dGamma0 TO SIGN(terr)*k*(Gammamax - Gammamin). 
		}
		ELSE IF (s=0) {
			SET dGamma0 TO dGamma0*terr/(dtc - dtc_p).
			IF minflag {
				SET dGamma0 TO SIGN(terr)*dGamma0.
			}
		}
		
		
		IF (dGamma0>0) {
			IF NOT minflag {
				SET Gammamin TO gamma0.
			}
			IF (Gamma0 + dGamma0)>Gammamax {	
				SET dGamma0 TO 0.9*(Gammamax - Gamma0).
			}
		
		}
		ELSE {
			IF NOT minflag {
				SET Gammamax TO gamma0.
			}
			IF (Gamma0 + dGamma0)<Gammamin {	
				SET dGamma0 TO 0.9*(Gammamin - Gamma0).
			}
		}
		
		RETURN LIST(s,dGamma0,Gammamin,Gammamax).
	}
	
	
	
	
	//END OF LOCAL FUNCTIONS

	
	
	
	
	PARAMETER r0.
	PARAMETER r1.
	PARAMETER deltaT.
	PARAMETER nrev.
	PARAMETER s_soln.
	PARAMETER s_guess.
	PARAMETER Gamma_guess.
	PARAMETER eps_cone.
	PARAMETER s_proj.
	PARAMETER i_N.
	
	
	LOCAL bodymu is SHIP:ORBIT:BODY:MU.
	LOCAL eps_t IS deltaT*0.01.
	LOCAL eps_t_p IS 0.5*eps_t.
	LOCAL eps_gamma IS 0.05.
	LOCAL eps_bigt IS eps_t_p.
	LOCAL eps_lambda IS 0.1.
	LOCAL imax IS 10.
	LOCAL ksm IS 1/10000.
	LOCAL kbg IS 0.25.
	
	LOCAL abssinth IS 0.
	LOCAL i_r0 IS r0:NORMALIZED.
	LOCAL i_r1 IS r1:NORMALIZED.
	//LOCAL r0mag IS r0:MAG.
	//LOCAL r1mag IS r1:MAG.
	LOCAL Nvec IS V(1,0,0).
	LOCAL s_cone IS 0.
	LOCAL s_180 IS 0.
	
	IF (s_proj=0) {
		SET Nvec TO VCRS(i_r0,i_r1).
		SET abssinth to Nvec:MAG.
		IF (abssinth<eps_cone) {
			SET s_cone TO 1.
			SET s_proj TO 1.
		}
	}
	
	IF (s_proj<>0) {
		SET r0 TO VXCL(i_N,r0).
		SET r1 TO VXCL(i_N,r1).
		SET i_r0 TO r0:NORMALIZED.
		SET i_r1 TO r1:NORMALIZED.
		SET abssinth TO ABS(VCRS(i_r0,i_r1)).
	}
	ELSE {			
		SET i_N TO Nvec/sinth.	
	}
	
	LOCAL s_180 IS VDOT(i_r1,VCRS(i_N,i_r0)).
	SET s_180 TO SIGN(s_180).
	SET sinth TO s_180*abssinth.
	SET costh TO VDOT(i_r0,i_r1).
	
	LOCAL lambda IS r0:MAG/r1:MAG.
	LOCAL p_1 IS 1 - costh.
	LOCAL p_2 IS costh - lambda.
	LOCAL Gammamax IS sinth/p_1 + SQRT(2*lambda/p_1).
	LOCAL Gammamin IS 0.
	
	IF s_180>0 {
		SET Gammamin TO p_2/sinth.
	} ELSE IF s_180<0 {
		SET Gammamin TO -10^20.
	}
	
	LOCAL k IS 0.
	LOCAL Gamma0 IS 0.
	LOCAL dtc IS 0.
	
	IF nrev<0 {
		print "negative revolutions makes no sense" at (1,61).
	    RETURN 1/0.
	} ELSE IF nrev>0 {
	
		//multi-rev bounds adjust coding seq
	
		SET k TO ksm.
		LOCAL i_cmin IS (r0 - r1):NORMALIZED.
		LOCAL Gamma_me IS VCRS(i_r0,i_cmin):NORMALIZED*s_180/(1 + VDOT(i_r0,i_cmin)).
		LOCAL Gamma_parab IS sinth/p_1 - SQRT(2*lambda/p_1).
		SET Gamma0 TO Gamma_me.
		LOCAL lambda_cap IS Gamma0.
		LOCAL lambda_cap_p IS 0.
		LOCAL dlambda_cap IS 2*k*(Gammamax - Gammamin).
		LOCAL s IS 1.
		LOCAL i IS 0.
		
		LOCAL m IS 0.
		LOCAL m_p IS 0.
		LOCAL m_err IS 0.
		LOCAL Gamma0_p IS 0.
		LOCAL dtc_p IS 0.
		
		UNTIL 0 {
			LOCAL pack IS ltti(Gamma0,p_1,p_2,sinth,costh,r0,nrev).
		    SET dtc TO pack[0].
			pack:CLEAR().
			
			IF i>0 {
				SET mp TO m.
				SET m TO (dtc - dtc_p)/(Gamma0 - Gamma0_p).
				SET lambda_cap_p TO lambda_cap.
				SET lambda_cap TO 0.5*(Gamma0 + Gamma0_p).			
			}
			IF (ABS(dtc - dtc_p)<eps_bigt) {BREAK.}
			IF i>1{
				SET m_err TO -m.
				SET dlambda_cap TO lambda_cap - lambda_cap_p.
				LOCAL pack IS secant_iter(s,m,m_p,m_err,dlambda_cap,lambda_cap,Gammamin,Gammamax,k,TRUE).
				SET s TO pack[0].
				SET dlambda_cap TO pack[1].
				pack:CLEAR().
				IF (lambda_cap + dlambda_cap - Gamma0)<eps_lambda {
					SET dlambda_cap TO dlambda_cap/2.
				}
			}
			
			SET Gamma0 TO lambda_cap + dlambda_cap.
			SET dtc_p TO dtc.
			SET Gamma0_p TO Gamma0.
			SET i TO i+1.
		}
		
		SET Gamma0 TO lambda_cap.
		LOCAL pack IS ltti(Gamma0,p_1,p_2,sinth,costh,r0,nrev).
		SET dtc TO pack[0].
		pack:CLEAR().
		
		IF (deltaT<dtc) {
		print "lambert problem has no solution" at (1,61).
	    RETURN 1/0.		
		}
		
		
		
		IF (s_180*s_soln)>0 {
			SET Gammamax TO lambda_cap.
			SET Gammamin TO MAX(Gammamin,Gamma_parab).
			SET kbg TO -kbg.
			SET ksm TO -ksm.
		}
		ELSE IF (s_180*s_soln)<0 {
			SEt Gammamin TO lambda_cap.
		}
	}
	
	
	
	IF (s_guess=0) {
		SET Gamma0 TO 0.5*(Gammamax + Gammamin).
		SET k TO kbg.
	} ELSE {
		SET Gamma0 TO Gamma_guess.
		SET k TO ksm.
	}
	
	LOCAL s IS 1.
	LOCAL i IS 0.
	
	
	LOCAL alpha_N IS 0.
	LOCAL p_N IS 0.
	LOCAL xx IS 0.
	LOCAL xi IS 0.
	LOCAL s_xi IS 0.
	LOCAL c_xi IS 0.
	
	UNTIL 0 {
	
		LOCAL pack IS ltti(Gamma0,p_1,p_2,sinth,costh,r0,nrev).
		SET dtc TO pack[0].
		SET alpha_N TO pack[1].
		SET p_N TO pack[2].
		SET xx TO pack[3].
		SET xi TO pack[4].
		SET s_xi TO pack[5].
		SET c_xi TO pack[6].
		pack:CLEAR().
		
		SET terr TO deltaT - dtc.
		
		IF (ABS(terr)<ABS(eps_t*deltaT)) OR (s=0 AND ABS(dtc - dtc_p)<eps_t_p) OR (s<>0 AND i=imax) {BREAK.}

		
		LOCAL pack IS secant_iter(s,dtc,dtc_p,terr,dGamma0,Gamma0,Gammamin,Gammamax,k,FALSE).
		SET s TO pack[0].
		SET dGamma0 TO pack[1].
		SET Gammamin TO pack[2].
		SET Gammamax TO pack[3].
		pack:CLEAR().
		
		IF (ABS(dGamma0)<eps_gamma) {BREAK.}
		
		SET Gamma0 TO Gamma0 + dGamma0.
		SET dtc_p TO dtc.
		SET i TO i+1.
	}
	
	LOCAL v0 IS SQRT(p_N*bodymu/r0:MAG)*(Gamma0*i_r0 + VCRS(i_N,i_r0)).
	LOCAL v1 IS (SQRT(bodymu)*xx/r1:MAG)*(xi*s_xi - 1)*i_r0 + (1 - xx^2 * c_xi/r1:MAG)*v0.
	
	RETURN LIST(v0,v1,Gamma0,s_cone,r1,sinth,costh,alpha_N,p_N,xx,xi,s_xi,c_xi).
}