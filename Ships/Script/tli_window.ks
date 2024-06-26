@LAZYGLOBAL OFF.
CLEARSCREEN.
clearvecdraws().

local landing_site is "Apollo 15".
local tli_earth_inclination is 30.

RUNPATH("0:/Libraries/misc_library").	
RUNPATH("0:/Libraries/maths_library").	
RUNPATH("0:/Libraries/navigation_library").

RUNPATH("0:/UPFG_pdi/Moon_sites.ks").	

local tgtsite is pdi_siteslex[landing_site].

print "Specified site: " + tgtsite["name"].


local cur_time is TIME.
local day_sec is 86400.	//synodic day length

local earth_body is BODY("earth").
local moon_body is BODY("moon").
local sun_body is BODY("sun").

local moonlex is lexicon(
					"body", moon_body,
					"position", moon_body:position - earth_body:position,
					"normal", body_orbital_normal_vec(moon_body),
					"polevec", body_orbital_normal_vec(earth_body),	//approximate with ecliptic
					"orb_ang_v", body_orbital_angular_vel(moon_body),
					"tgt_site_vec", tgtsite["position"]:position - moon_body:position
).


local sunlex is lexicon(
					"body", sun_body,
					"position", sun_body:position - earth_body:position,
					"normal", body_orbital_normal_vec(earth_body),
					"orb_ang_v", body_orbital_angular_vel(earth_body)
).

//orbit_moon_sun(8 * day_sec).

local moon_antipode is -moonlex["position"].
local earth_polevec is v(0,1,0).

local antipode_proj is vxcl(earth_polevec, moon_antipode).
local antipode_norm is vcrs(antipode_proj, earth_polevec):normalized.

local antipode_lat is signed_angle(antipode_proj, moon_antipode, antipode_norm, 0).
local tli_norm_rota is 90 - get_a_bBB(antipode_lat, tli_earth_inclination).
local moon_normal_rota is signed_angle(antipode_proj, vxcl(earth_polevec,moonlex["normal"]), earth_polevec, 0). 

local tli_norm_0 is rodrigues(v(0,1,0), antipode_norm, sign(antipode_lat) * tli_earth_inclination).

local tli_norm_high is rodrigues(tli_norm_0, earth_polevec, -sign(moon_normal_rota) * tli_norm_rota).
local tli_norm_low is rodrigues(tli_norm_0, earth_polevec, sign(moon_normal_rota) * tli_norm_rota).


local moon_antipode_normv is vcrs(moon_antipode, moonlex["normal"]):normalized.
local tli_norm_high_lunarsoi is tli_norm_high - 2*vdot(tli_norm_high, moon_antipode_normv)*moon_antipode_normv.
local tli_norm_low_lunarsoi is tli_norm_low - 2*vdot(tli_norm_low, moon_antipode_normv)*moon_antipode_normv.


print vang(moonlex["polevec"], sunlex["normal"]).

//local site_proj_high is vxcl(, tgtsite["position"]:position

//until false {
//clearvecdraws().
//arrow_body(-moonlex["normal"], "norm").
//arrow_body(moonlex["position"], "moon").
//arrow_body(moon_antipode, "antipode").
//arrow_body(antipode_proj, "antipode_proj").
//arrow_body(tli_norm_high, "tli_norm_high").
//arrow_body(tli_norm_low, "tli_norm_low").
//
////arrow_foreignbody(moon_body, - moonlex["normal"], "normal").
////arrow_foreignbody(moon_body, tli_norm_high_lunarsoi, "high").
////arrow_foreignbody(moon_body, tli_norm_low_lunarsoi, "low").
//arrow_foreignbody(moon_body, moonlex["tgt_site_vec"], "site").
////arrow_foreignbody(moon_body, moonlex["polevec"], "pole").
//arrow_foreignbody(moon_body, sunlex["normal"], "ecliptic").
//wait 0.3.
//}



//cycle through a calendar year
//local tli_time_bias is 4 * day_sec.
//orbit_moon_sun(tli_time_bias).
//set cur_time to cur_time + timespan(tli_time_bias).

//cycle a month to test the thing
FROM {local d is 0.} UNTIL (d >= 30) STEP {set d to d+0.1.} DO {
	CLEARSCREEN.
	clearvecdraws().
	
	print cur_time:CALENDAR at (0,1).
	
	arrow_foreignbody(moon_body, - moonlex["position"], "moon").
	arrow_foreignbody(moon_body, sunlex["position"], "sun").
	arrow_foreignbody(moon_body, - moonlex["normal"] , "normal").
	arrow_foreignbody(moon_body, - moonlex["polevec"] , "polevec").
	arrow_foreignbody(moon_body, moonlex["tgt_site_vec"] , "site").
	
	
	//advance the day
	
	orbit_moon_sun(0.1*day_sec).
	set cur_time to cur_time + timespan(day_sec).
	
	
	wait 0.1.
}




function orbit_moon_sun {
	parameter delta_t.
	
	set sunlex["position"] to rodrigues(sunlex["position"], sunlex["normal"], sunlex["orb_ang_v"]*delta_t).
	set moonlex["position"] to rodrigues(moonlex["position"], moonlex["normal"],  moonlex["orb_ang_v"]*delta_t).
	//set moonlex["polevec"] to rodrigues(moonlex["polevec"], moonlex["normal"],  moonlex["orb_ang_v"]*delta_t).
	
	set moonlex["tgt_site_vec"] to rodrigues(moonlex["tgt_site_vec"], moonlex["polevec"], moonlex["orb_ang_v"]*delta_t).

}