//author C. P. Konstantinidis

parameter range_d is 1000.
parameter di is 90.
parameter lander_offset is 5.

function stepUp{
    declare local h1 is ship:altitude.
    wait 0.2.
    declare local h2 is ship:altitude.
    if(h1-h2<=0)
        return true.
    else
        return false. 
}

function checkFuel{
    set tanks to ship:partsnamed("fuelTank.long").
    for tank in tanks{
        if tank:mass<=tank:drymass{
            print "Tank: "+tank:name+" decoupled".
            tank:decoupler:getModule("ModuleAnchoredDecoupler"):doEvent("decouple").
        }
    }    
}

function stoppingDistance{
    parameter offset is 2.
    return ship:altitude-ship:orbit:body:geoPositionOf(ship:position):terrainHeight-offset.
}

function distanceToGround{
    parameter sf is ship:orbit:body:mu/(ship:orbit:body:radius^2).
    local maxDec is (ship:availablethrust/ship:mass)-sf.
    return (ship:verticalspeed^2+ship:groundspeed^2)/(2*maxDec).
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
        if i:ignition
            set paranomastis to paranomastis+i:possiblethrustat(constant:kpatoatm*ship:sensors:pres)/i:ispat(constant:kpatoatm*ship:sensors:pres).
    }
    return g(0)*ariumitis/paranomastis.
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

function ascension{
    set ag1 to false.//retract ladders
    set ag2 to false.//retract electrics and comms
    print "Countdown:".
    from{local count is 10.} until count=0 step {set count to count -1.}do{
        print "..."+count.
        wait 1.
    }
    if (ship:sensors:pres>0){
        print "Atmosheric hop".
        atmosphericAscension().
    }else{
        print "Vaccum hop".
        vacuumAscension().
    }
}

function atmosphericAscension{
set sas to false.
declare local bias is ship:orbit:semimajoraxis.
if(range_d<5000){
    print "Unable to comply".
}else{   
 lock steering to heading(di,90).
 lock throttle to 1.
 wait 2.
 set gear to false.
 set ag8 to false.
 wait until ship:verticalSpeed>3.
 set gear to false.
 until ship:verticalSpeed>50{
    checkFuel().
 }
 lock steering to heading(di,45).
 until ship:orbit:semimajoraxis>=bias+1.5*range_d{
    if (ship:altitude>10000)
        lock steering to heading(di,30).
    print "Diff"+(ship:orbit:semimajoraxis-bias).
    checkFuel().
 }
 lock throttle to 0.
 lock steering to srfPrograde.
 print "t_apoaps:"+ETA:apoapsis.
 wait until stepUp()=false.
  print "Apoapsis reached".
 if(ship:orbit:semimajoraxis<bias+1.5*range_d){
    lock throttle to 1.
    until ship:orbit:semimajoraxis>=bias+1.5*range_d{
        checkFuel().
    }
    lock throttle to 0.
 }
 lock steering to srfRetrograde.
 wait 10.
 when stoppingDistance(lander_offset)<250 then {
         if not(ship:partsnamed("KRE-ShepardLeg-S"):empty) and not(ship:partsnamed("fuelTank.long"):empty)
            set ag8 to true.
        else
            set gear to true.
}
 wait until stoppingDistance(lander_offset)<10000 and shipSpeed()<940.
        wait until stage:ready.
        stage.//drag sutes
        print "Drag sutes deployed".
        wait 1.
 wait until stoppingDistance(lander_offset)<3000.
        wait until stage:ready.
        stage.//main sutes
        print "Main sutes deployed".
        wait 1.
 wait until stoppingDistance(lander_offset)<50.
    lock throttle to thrustRatio(stoppingDistance(lander_offset)).
    until ship:verticalspeed>-3.5{
        print "Throttle:"+throttle.
    }
    lock throttle to 0.
    unlock steering.
    set sas to true.
    wait 5.
    set ag1 to true.
    print "deploy ladders". 
    set ag2 to true.
    print "deploy electrics and coms".
}
}

function vacuumAscension{
declare local bias is ship:orbit:semimajoraxis.
     if(range_d<1000){
    print "Unable to comply".
 }else{   
 set SAS to false.   
 lock steering to up.
 lock throttle to 1.
 wait 2.
 set gear to false.
 wait until ship:verticalSpeed>50.
 lock steering to heading(di,10).
 wait until ship:orbit:semimajoraxis>=bias+2*range_d.
 lock throttle to 0.
 lock steering to srfRetrograde.
 wait until stepUp()=false.
 print "Apoapsis reached".
 when stoppingDistance(lander_offset)<250 then {
         if not(ship:partsnamed("KRE-ShepardLeg-S"):empty) and not(ship:partsnamed("fuelTank.long"):empty)
            set ag8 to true.
        else
            set gear to true.
}
 wait until stoppingDistance(lander_offset)<=2000.
 lock throttle to distanceToGround()/stoppingDistance(lander_offset).
 until ship:verticalspeed>-3.1 and ship:groundspeed<0.5{
        print "Throttle:"+throttle.
 }
 lock throttle to 0.
 unlock steering.
 set sas to true.
 wait 5.
 set ag1 to true.
 print "deploy ladders". 
}
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
    ascension().        
    print "=== Script end ===".
}

main().