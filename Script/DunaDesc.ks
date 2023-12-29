//author C P Konstantinidis.
//To be used with Duna Lander II craft.

parameter offset_d is 9.

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

function isEngineActive{
    list ENGINES in eng.
    for ij in eng{
        if (ij:ignition)
            return true.
    }    
    return false.
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

function g{
    parameter h is 0.
    return ship:orbit:body:mu/((ship:orbit:body:radius+h)^2).
}

function shipSpeed{
    return sqrt(ship:verticalSpeed^2+ship:groundSpeed^2).
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

function speedDecreasing{
    declare local vel is abs(ship:verticalspeed)+abs(ship:groundspeed).
    wait 0.1.
    if (vel>(abs(ship:verticalspeed)+abs(ship:groundspeed))){
        return true.
    }else
        return false.
}

function descent{
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


function main{
    print "=== Eve Descent Script active ===".
        descent(offset_d).
    print "=== Program ended ===".
}

main().