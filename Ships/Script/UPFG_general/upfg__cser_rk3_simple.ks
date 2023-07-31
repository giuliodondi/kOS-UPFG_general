//only spherical + j2 terms
FUNCTION agrav {

	//ought to be in metres
	PARAMETER r.

	LOCAL r_inv IS 1/r:MAG.
	
	//central body potential
	LOCAL G_c IS -BODY:MU*r*(r_inv^3).
	
	//LOCAL u_r IS r:NORMALIZED.
	
	LOCAL G IS V(0,0,0).
	
	//unnormalised, dimensionless coefficients
	//LOCAL J IS LIST(-0.10826360229840e-02,0.25324353457544e-05).
	
	//unnormalised, dimensionful coefficients
	LOCAL J2 IS -1.7555283e+25	.
	//LOCAL J3 IS 2.6191329e+29
	
	
	
	LOCAL xx IS r:X.
	LOCAL yy IS r:Y.
	LOCAL zz IS r:Z.
	
	LOCAL r7 IS r_inv^2.
	LOCAL r5 IS r7^2*r_inv.
	SET r7 TO r5*r7.
	
	LOCAL xlam IS 1.5*J2.
	LOCAL xltb IS xlam*(r5 - 5*r7*zz^2).
	
	SET G:X TO -xltb*xx.
	SET G:Y TO -xltb*yy.
	SET G:Z TO -xltb*zz - 2*xlam*zz*r5.
	
	
	
	//local aux1 IS J2*(r_inv^7).
	//LOCAL aux2 IS zz^2.
	//LOCAL aux3 IS xx^2 + yy^2.
	//LOCAL aux4 IS 3*aux3.
	
	
	//SET G:X TO G:X + aux1*3*(2*aux2 - 0.5*aux3)*xx.
	//SET G:Y TO G:Y + aux1*3*(2*aux2 - 0.5*aux3)*yy.
	//SET G:Z TO G:Z + aux1*3*(aux2 - 0.5*aux4)*zz.
	
	//SET aux1 TO J3*(r_inv^9).
	
	//SET G:X TO G:X + aux1*5*(2*aux2 - 0.5*aux4)*xx*zz.
	//SET G:Y TO G:Y + aux1*5*(2*aux2 - 0.5*aux4)*yy*zz.
	//SET G:Z TO G:Z + aux1*(4*aux2*(aux2 - aux4) + 0.5*aux4^2)
	
	RETURN G_c + G.
}

//RK3 integration
FUNCTION cse {
	DECLARE PARAMETER r0, v0, tgo,dT.
	
	
	LOCAL rf IS r0.
	LOCAL vf IS v0.
	
	//rotate to ecef frame here
	//LOCAL ref IS vecYZ(SOLARPRIMEVECTOR).
	
	//SET ref TO rodrigues(ref, V(0,0,1), -BODY:ROTATIONANGLE).
	
	//LOCAL rotangle IS signed_angle(v(1,0,0),ref,v(0,0,1),0).
	
	//SET rf TO rodrigues(rf,V(0,0,1),rotangle).
	//SET vf TO rodrigues(vf,V(0,0,1),rotangle).
	
	
	
	
	//LOCAL T IS 0.
		
	LOCAL nstep IS 7.
	LOCAL dT Is tgo/nstep.
	
	LOCAL Kv1 IS 0.
	LOCAL Kv2 IS 0.
	LOCAL Kv3 IS 0.
	LOCAL Kr1 IS 0.
	LOCAL Kr2 IS 0.
	LOCAL Kr3 IS 0.
	
	FROM { LOCAL i IS 1. } UNTIL i>nstep STEP { SET i TO i+1. } DO {
		SET Kv1 TO agrav(rf).
		SET Kr1 TO vf.
		
		SET Kv2 TO agrav(rf + 0.5*dT*Kr1).
		SET Kr2 TO vf + 0.5*dT*Kv1.
		
		SET Kv3 TO agrav(rf + dT*(2*Kr2 - Kr1)).
		SET Kr3 TO vf + dT*(2*Kv2 - Kv1).
		
		SET vf TO vf + dT*(Kv1 + 4*Kv2 + Kv3)/6.
		SET rf TO rf + dT*(Kr1 + 4*Kr2 + Kr3)/6.
	}
	
	//rotate back to eci frame here
	
	//SET rf TO rodrigues(rf,V(0,0,1),-rotangle).
	//SET vf TO rodrigues(vf,V(0,0,1),-rotangle).
	
	RETURN LISt(rf,vf,dT).

}