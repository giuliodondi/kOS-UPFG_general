@LAZYGLOBAL OFF.
CLEARSCREEN.
clearvecdraws().

local landing_site is "Apollo 11".
local tli_earth_inclination is 30.
local min_sun_angle is 6.
local max_sun_angle is 18.

RUNPATH("0:/Libraries/misc_library").	
RUNPATH("0:/Libraries/maths_library").	
RUNPATH("0:/Libraries/navigation_library").

RUNPATH("0:/UPFG_pdi/Moon_sites.ks").	

local tgtsite is pdi_siteslex[landing_site].

print "Specified site: " + tgtsite["name"].


local start_time is TIME.
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


//local tli_opportunities is tli_planner_site(
//			moonlex["position"],
//			moonlex["normal"],
//			moonlex["tgt_site_vec"],
//			tli_earth_inclination
//).
//
//print "high incl. rel angle : " + tli_opportunities["high_tli"]["site_angle"].
//print "low incl. rel angle : " + tli_opportunities["low_tli"]["site_angle"].
//
//
//until false {
//clearvecdraws().
//arrow_body(-moonlex["normal"], "norm").
//arrow_body(moonlex["position"], "moon").
//arrow_body(tli_opportunities["high_tli"]["normal"], "tli_norm_high").
//arrow_body(tli_opportunities["low_tli"]["normal"], "tli_norm_low").
//
//arrow_foreignbody(moon_body, - moonlex["normal"], "normal").
//arrow_foreignbody(moon_body, moonlex["tgt_site_vec"], "site").
//
////arrow_foreignbody(moon_body, moonlex["polevec"], "pole").
////arrow_foreignbody(moon_body, sunlex["normal"], "ecliptic").
//wait 0.3.
//}



//cycle through a calendar year
local tli_time_bias is 4 * day_sec.
orbit_moon_sun(tli_time_bias).
set cur_time to cur_time + timespan(tli_time_bias).

local tli_opportunities is list().

FROM {local d is 0.} UNTIL (d >= 365) STEP {set d to d+1.} DO {
	CLEARSCREEN.
	clearvecdraws().
	
	print format_calendar(cur_time) at (0,1).
	
	arrow_foreignbody(moon_body, - moonlex["position"], "moon").
	arrow_foreignbody(moon_body, sunlex["position"], "sun").
	arrow_foreignbody(moon_body, - moonlex["normal"] , "normal").
	arrow_foreignbody(moon_body, - moonlex["polevec"] , "polevec").
	arrow_foreignbody(moon_body, moonlex["tgt_site_vec"] , "site").
	
	local site_sun_angle is 90 - signed_angle(moonlex["tgt_site_vec"], sunlex["position"], moonlex["polevec"], 0).
	
	if (site_sun_angle >= min_sun_angle) and (site_sun_angle <= max_sun_angle) {
		for t_opp in tli_planner_site(
						cur_time - timespan(tli_time_bias),
						moonlex["position"],
						moonlex["normal"],
						moonlex["tgt_site_vec"],
						tli_earth_inclination
			) {
			tli_opportunities:add(t_opp).	
		}
		
	}
	
	//advance the day
	
	orbit_moon_sun(day_sec).
	set cur_time to cur_time + timespan(day_sec).
	
	
	wait 0.
}

local first_tli_opp is tli_opportunities[0].
local best_tli_opp is tli_opportunities[0].

for t_opp in tli_opportunities {
	
	if (t_opp["time"] < first_tli_opp["time"]) {
		if (t_opp["site_angle"] < first_tli_opp["site_angle"]) {
			set first_tli_opp to t_opp.
		}
	}
	
	if (t_opp["site_angle"] < best_tli_opp["site_angle"]) {
		set best_tli_opp to t_opp.
	}
}

print "         current date : " + format_calendar(start_time) at (0,3).


print "first tli opportunity : " + format_calendar(first_tli_opp["time"]) at (0,5).
print "	 site relative angle : " + first_tli_opp["site_angle"] at (0,6).
print "	   tli orbital angle : " + first_tli_opp["orbit_angle"] at (0,7).

print "best tli opportunity : " + format_calendar(best_tli_opp["time"]) at (0,9).
print "	 site relative angle : " + best_tli_opp["site_angle"] at (0,10).
print "	   tli orbital angle : " + best_tli_opp["orbit_angle"] at (0,11).


clearvecdraws().

function orbit_moon_sun {
	parameter delta_t.
	
	set sunlex["position"] to rodrigues(sunlex["position"], sunlex["normal"], sunlex["orb_ang_v"]*delta_t).
	set moonlex["position"] to rodrigues(moonlex["position"], moonlex["normal"],  moonlex["orb_ang_v"]*delta_t).
	//set moonlex["polevec"] to rodrigues(moonlex["polevec"], moonlex["normal"],  moonlex["orb_ang_v"]*delta_t).
	
	set moonlex["tgt_site_vec"] to rodrigues(moonlex["tgt_site_vec"], moonlex["polevec"], moonlex["orb_ang_v"]*delta_t).

}