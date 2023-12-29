parameter offset_d is 5.

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

function distanceToGround{
    parameter sf is ship:orbit:body:mu/(ship:orbit:body:radius^2).
    local maxDec is (ship:availablethrust/ship:mass)-sf.
    return (ship:verticalspeed^2+ship:groundspeed^2)/(2*maxDec).
}

function thrustRatio{
    parameter sf is ship:orbit:body:mu/(ship:orbit:body:radius^2).
    declare local thruster is ship:mass*(sf+((ship:verticalspeed^2+ship:groundspeed^2)/(2*stoppingDistance(offset_d)))).
    declare local ratio is thruster/ship:availablethrustat(0.00980966720709589*ship:sensors:pres).
    if ratio>1 or ratio<0
        return 1.
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

function atmosphericDescent{
parameter lander_offset.
    set sas to false.
    lock steering to srfRetrograde.
    //set RCS to TRUE.
    wait until vDot(ship:facing:forevector,srfRetrograde:vector)>0.999.
    //declare local s_time to TIME:SECONDS + ship:orbit:ETA:apoapsis.
    //wait until time:seconds>=s_time.
    lock throttle to 1.
    set RCS to FAlSE.
    wait until ship:periapsis<=(ship:orbit:body:atm:height*0.6).
    lock throttle to 0.
    wait 0.2.
   // wait until stage:ready.
   // stage.
    wait until ship:altitude<ship:orbit:body:atm:height.
   // set ag2 to true.//Inflate thermoshields
    when stoppingDistance(lander_offset)<250 then {
    //set gear to true.
     set ag8 to true.//Gear extracted
    }
    wait until stoppingDistance(lander_offset)<=9000.
    if ship:groundspeed>700{
        lock throttle to 1.
        wait until ship:groundspeed<=640.
        lock throttle to 0.
    }
   // set ag3  to true. //eject top thermoshield
    wait until stage:ready.
    stage. //Drag parachutes
    print "Deploy drag sutes".
    wait until stoppingDistance(lander_offset)<=7000.
    wait until stage:ready.
    stage. //main parachutes
    print "Deploy main sutes".
    wait 0.2.
    //set ag4 to true. //eject down thermoshield
    wait until stoppingDistance(lander_offset)<=1020.
    declare local sf is calculateSF().
    print "Sa="+sf.
    lock pct to thrustRatio(sf).
    wait until stoppingDistance(lander_offset)<=1000 or pct>1.
    lock throttle to pct.
    until ship:verticalspeed>-2.1 {
        print "Throttle:"+throttle.
    }
    lock throttle to 0.
    unlock steering.
    set sas to true.
    wait 5.
    set ag3 to true.
    print "deploy ladders".
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
    until ship:verticalSpeed>-3.1 and ship:groundspeed<0.4{
         print "pct ratio:"+pct+",vel"+ship:verticalspeed.
    }
    lock throttle to 0.
    unlock steering.
    set sas to true.  
    wait 2.
    set ag1 to true.
    print "Deploy ladders".
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