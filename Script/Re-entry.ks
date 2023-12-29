parameter offset_d is 5.

function isEngineActive{
    list ENGINES in eng.
    for ij in eng{
        if (ij:ignition)
            return true.
    }    
    return false.
}

function possibleThrust{
    parameter pressure is 0.
    declare local thrust to 0.
    list ENGINES in engine.
    for i in engine{
        if i:ignition {
            set thrust to thrust + i:possiblethrustat(pressure).
        }
    }
    return thrust.
}

function terrainHeight{
    set y to ship:orbit:body:geoPositionOf(ship:position):terrainHeight.
    if y<=0
        return 0.
    else
        return y.
}

function stoppingDistance{
    parameter offset is 2.
    return ship:altitude-terrainHeight()-offset.
}

function distanceToGround{
    parameter sf is ship:orbit:body:mu/(ship:orbit:body:radius^2).
    local maxDec is (ship:availablethrust/ship:mass)-sf.
    return (ship:verticalspeed^2+ship:groundspeed^2)/(2*maxDec).
}

function calculateExhaustSpeedg{
    list ENGINES in ind.
    local ariumitis is possibleThrust(constant:kpatoatm*ship:sensors:pres).
    local paranomastis is 0.
    for i in ind{
        set paranomastis to paranomastis+i:possiblethrustat(constant:kpatoatm*ship:sensors:pres)/i:ispat(constant:kpatoatm*ship:sensors:pres).
    }
    return g(0)*ariumitis/paranomastis.
}

function thrustRatio{
    parameter dis is 1.
    declare local dv is ship:verticalspeed-3.
    declare local vex is calculateExhaustSpeedg(). 
    declare local thruster is (ship:mass/(2*dis))*(ship:verticalspeed^2-(9*constant:e^(-dv/vex))).
    declare local ratio is thruster/possibleThrust(constant:kpatoatm*ship:sensors:pres).
    if ratio>1
        return 1.
    else if ratio<=0
        return 0.
    else
        return ratio.
}

function distanceToGroundAtmo{
    parameter sf is ship:orbit:body:mu/(ship:orbit:body:radius^2).
    local maxDec is (ship:availableThrustat(ship:sensors:pres)/ship:mass)-sf.
    return (ship:verticalspeed^2+ship:groundspeed^2)/(2*maxDec).
}

function calculateSF{
    declare local h1 is ship:altitude.
    declare local sv1 is ship:verticalspeed^2.
    wait until (h1-ship:altitude)>10.
    return (sv1-ship:verticalspeed^2)/(2*(h1-ship:altitude)).
}

function planetReentry{
    parameter offset.
     if (isEngineActive()){
        print "Engine active".
    }else{
        wait until stage:ready.
        stage.//engine start
        wait 1.
    }
    if (bodyAtmosphere(ship:orbit:body:name):exists) {
        print "Atmosperic re-entry".
        atmosphericDescent(offset).
    }else{
        print "Non - atmospheric re-entry".
        vacuumDescent(offset).
    }
}

function g{
    parameter h is 0.
    return ship:orbit:body:mu/((ship:orbit:body:radius+h)^2).
}

function shipSpeed{
    return sqrt(ship:verticalSpeed^2+ship:groundSpeed^2).
}

function speedDecreasing{
    declare local vel is abs(ship:verticalspeed)+abs(ship:groundspeed).
    wait 0.1.
    if (vel>(abs(ship:verticalspeed)+abs(ship:groundspeed))){
        return true.
    }else
        return false.
}

function atmosphericDescent{
parameter lander_offset.
   set sas to false.
    set RCS to true.
    lock steering to srfRetrograde.
    wait until vDot(ship:facing:forevector,srfRetrograde:vector)>0.999.
    print "Retro go".
    wait until speedDecreasing() or ship:altitude<0.8*ship:orbit:body:atm:height.
    set RCS to false.
    when stoppingDistance(lander_offset)<350 then {
      set gear to true.
    }
    wait until stoppingDistance(lander_offset)<19900 and shipSpeed()<=930 .
    print "Deploy drag chutes.".
    wait until stage:ready.
    stage.
    wait 2.
    unlock steering.
    wait until stoppingDistance(lander_offset)<=3500.
    wait until stage:ready.
    stage. //main parachutes
    print "Deploy main sutes".
    wait 3.
    wait until stage:ready.
    stage. //eject down thermoshield
    wait 3.5.
    print "Eject thermoshield.".
    if not(ship:partsnamed("radialDrogue"):empty){
        set drogue to ship:partsnamed("radialDrogue").
        for chute in drogue{
            chute:getmodule("RealChuteModule"):doaction("cut chute",true).
        }   
    print "Cutting drogue chute's".
    }
    wait until stoppingDistance(lander_offset)<=50.
    lock steering to srfRetrograde.
    wait until stage:ready.
    stage. //main engines
    print "Start main engine".
    lock throttle to thrustRatio(stoppingDistance(lander_offset)).
    until ship:verticalspeed>=-3.5 {
        print "Throttle:"+throttle.
    }
    lock throttle to 0.
    unlock steering.
    set sas to true.
    wait 5.
    set ag1 to true.
    print "deploy electrics and coms.".
    wait 1.
    set ag2 to true.
    print "deploy ladders.".   
}

function vacuumDescent{
    parameter lander_offset.
    set SAS to FALSE.
    set RCS to TRUE.
    lock steering to srfRetrograde.
    wait until vDot(ship:facing:forevector,srfRetrograde:VECTOR)>0.999.
    lock throttle to 1.
    set RCS to FALSE.
    wait until ship:periapsis<0 or ship:maxthrust=0.
    lock throttle to 0.
    wait 0.1.
    when stoppingDistance(lander_offset)<250 then {
        set gear to true.
    }
    lock pct to distanceToGround()/stoppingDistance(lander_offset).
    until pct>1 and stoppingDistance(lander_offset)<3000{
         print "pct ratio:"+pct.
    }
    lock throttle to pct.   
    until ship:verticalSpeed>-3.5{
         print "pct ratio:"+pct+",vel"+ship:verticalspeed.
    }
    lock throttle to 0.
    unlock steering.
    set sas to true.  
    wait 2.
    set ag1 to true.
    print "Deploy ladders".
    set ag2 to true.
    print "Deploy Electrics".
}

function readKeyboard{
    set value to "".
    set ch to "".
    print "Enter now to start decent:".
    until ch=terminal:input:enter {
        set ch to terminal:input:getchar().
        if not(ch=terminal:input:enter){
        set value to value+ch.
        print value.
        }
    }
    if value="now" or value="NOW"
        return true.
    else
        return false.
}

function main{
    print "=== Script active ===".
    if(readKeyboard())
        planetReentry(offset_d).
    print "=== Program ended ===".
}

main().