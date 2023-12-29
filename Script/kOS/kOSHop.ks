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
    parameter sf is ship:orbit:body:mu/(ship:orbit:body:radius^2).
    declare local thruster is ship:mass*(sf+((ship:verticalspeed^2+ship:groundspeed^2)/(2*stoppingDistance(lander_offset)))).
    declare local ratio is thruster/ship:availablethrustat(ship:sensors:pres).
    if ratio>1 or ratio<0
        return 1.
    else 
        return ratio.
}

function calculateSF{
    declare local h1 is ship:altitude.
    declare local sv1 is ship:verticalspeed^2.
    wait until (h1-ship:altitude)>10.
    return (sv1-ship:verticalspeed^2)/(2*(h1-ship:altitude)).
}

function ascension{
    set ag1 to false.//retract ladders
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
 when stoppingDistance(lander_offset)<=250 then {
    if ship:partsnamed("KRE-ShepardLeg-S"):empty
        set gear to true.
    else
        set ag8 to true.
}
 wait until stoppingDistance(lander_offset)<10000.
        wait until stage:ready.
        stage.//drag sutes
        print "Drag sutes deployed".
        wait 1.
 wait until stoppingDistance(lander_offset)<7000.
        wait until stage:ready.
        stage.//main sutes
        print "Main sutes deployed".
        wait 1.
 wait until stoppingDistance(lander_offset)<1200.
 declare local sf is calculateSF().
//declare local sm is ship:mass.
 //   wait until stage:ready.
 //   stage.//engine start
    //lock throttle to distanceToGroundAtmo(sf)/stoppingDistance(lander_offset).
    wait until stoppingDistance(lander_offset)<=1000.
    lock throttle to thrustRatio(sf).
    until ship:verticalspeed>-3.1{
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
         if ship:partsnamed("KRE-ShepardLeg-S"):empty
        set gear to true.
    else
        set ag8 to true.
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