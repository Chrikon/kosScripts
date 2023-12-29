//author C P Konstantinidis.
//to be used with Eve Explorer VII craft.

parameter direction_d is 90,altitude_d is 95000,thrust_d is 1.

function checkEngineStatus{
   LIST ENGINES in eng.
    for engi in eng{
        if (engi:flameout){
            print "Engine flameout purge".
            //engi:decoupler:getModule("ModuleAnchoredDecoupler"):doEvent("decouple").
            wait until stage:ready.
            stage. 
            return true.
        }
    }
    return false.
}

function acheiveTWR{
    parameter var.
    parameter engi.
    parameter rat is 2.
    declare local thr to calculateNowThrust(engi).
    local twr is ((thr)*(ship:orbit:body:radius+ship:altitude)^2)/(ship:MASS*ship:orbit:body:mu).
    local ratio is 1-((twr)/rat).//2 for the drag forces
    local factor is var+(0.3/ship:sensors:acc:mag)*ratio.
    print "TWR:"+twr+", Coeff:"+ratio+", Fac:"+factor.
    if factor>=1
        return 1.
    else if factor<=0
        return 0.
    else
        return factor.
}


function findActiveEngine{
    LIST ENGINES in eng.
    local s to list().
    for engi in eng{
        if (engi:ignition and engi:maxthrust>0)
            s:add(engi).
    }
    return s.
}

function g{
    parameter h is 0.
    return ship:orbit:body:mu/((ship:orbit:body:radius+h)^2).
}

function calculateExhaustSpeedg{
    parameter ind.
    for i in ind{
        print i:name.
    }
    local ariumitis is calculatePossibleThrust(ind).
    local paranomastis is 0.
    for i in ind{
        if(constant:kpatoatm*ship:sensors:pres>0.001)
            set paranomastis to paranomastis+i:possiblethrustat(constant:kpatoatm*ship:sensors:pres)/i:ispat(constant:kpatoatm*ship:sensors:pres).
        else
            set paranomastis to paranomastis+i:possiblethrust/i:vacuumisp.
    }
    return g(0)*ariumitis/paranomastis.
}

function calculatePossibleThrust{
    parameter ind is findActiveEngine().
    local thr is 0.
    for i in ind{
        if(constant:kpatoatm*ship:sensors:pres>0.01)
            set thr to thr+i:possiblethrustat(constant:kpatoatm*ship:sensors:pres).
        else
            set thr to thr+i:possiblethrust.
    }
    return thr.
}

function calculateNowThrust{
    parameter ind is findActiveEngine().
    local thr is 0.
    for i in ind{
        set thr to thr+i:thrust.
    }
    return thr.
}

function calculateBurnTime{
  parameter x,ind.
    local ve is calculateExhaustSpeedg(ind).
    local thrust is calculatePossibleThrust(ind).
    local dv is x:burnvector:mag.
    local a is dv/ve.
    return round(((ve*ship:mass/thrust)*(1-(1/constant:e^a))),1).
}


function calculateDVForCirculize{
    LOCAL m_time IS TIME:SECONDS + ETA:APOAPSIS.
    // what will the magnitude of our velocity be at that time
    LOCAL v0 IS VELOCITYAT(SHIP, m_time):ORBIT:MAG.
    PRINT "V0="+v0.
    //Kerbin standard graviatational parameter
    //local m TO 3.7316E+12.
    LOCAL V1 IS sqrt(ship:orbit:body:mu/(ship:orbit:body:radius+ship:apoapsis)).
    PRINT "V1="+V1.
    declare local DV is V1-v0.
    PRINT "dV="+DV.
    return dv.
}

function createManuverNode{
    parameter exectime is eta:apoapsis.
    parameter dv is calculateDVForCirculize().
    local a is node(TIME:seconds+exectime,0,0,dv).
    return a.
}

function vectorDif{
    parameter x.
    parameter y.
    parameter lim is 2.
    local s is vang(x,y).
    if s>=lim{
        print "TRUE: "+s.
        return true.
    }else{
       // print "FALSE"+s.
        return false.
    }
}

function executeManuver{
    parameter x,radius1,lim is 0.1.
    set SAS to false.
    lock steering to x.
    //print radius1.
    print "DeltaV of stage:"+(SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT).
    wait 5.
    declare local ind is findActiveEngine().
    print ind.
    
    if(SHIP:DELTAV:CURRENT>=x:deltav:MAG)
        simpleExecution(x,calculateBurnTime(x,ind),radius1,lim).    
    else{
        unlock steering.
        print "Manuver not feasible".
        wait 1.
        set SAS to TRUE.
        wait 2.
        set SASMODE to "PROGRADE".
    }
}

function simpleExecution{
    parameter x,burnTime,radious,lim is 1.
    print "Burntime:"+burnTime.
    local a is x:time-(burnTime/2).
    local b is x:time+(burnTime/2).
    wait until time:seconds>=a.
    declare local originalVector to x:burnvector.
    print "Throttle 100%".
    lock throttle to 1.
    until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:deltav:mag<0.1  or (ship:apoapsis>0 and ship:apoapsis-ship:periapsis<=radious){
        set originalVector to x:burnvector.
        print x:burnvector:mag.
    }
    lock throttle to 0.
    print "Time sec:"+time:seconds+" ,b:"+b.
    unlock steering.
}

function isMAnueverFeasible{
    parameter mnv.
    local t_burn is 0.
    ADD mnv.
    if(SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT>=mnv:deltav:MAG)
        set t_burn to calculateBurnTime(mnv).
    REMOVE mnv.
    if(time:seconds-1<mnv:time+t_burn/2)
        return true.
    else
        return false.
}

function circulizeInitialOrbit{
    local manuverNode is createManuverNode().
    add manuverNode.
    executeManuver(manuverNode,1000,1).
    REMOVE manuverNode.
}

function doAscent{ 
parameter ship_altitude,ship_throttle,direction.
print "Countdown:".
from{local count is 10.} until count=0 step {set count to count -1.}do{
    print "..."+count.
    wait 1.
}
print "Launch.".
declare local maxQ is ship:q. 
declare local di is direction.
declare local pi is 90.
declare local flag is false.
set sas to false.
wait 0.2.
lock steering to heading(di,pi).
print "Heading :90,"+direction.
declare local vsteer is ship_throttle.
lock throttle to vsteer.
//stage.
wait until ship:verticalspeed>3.
set gear to false.
declare local engi is findActiveEngine().
for i in engi{
    print i:name.
}
until ship:apoapsis>=ship_altitude{
    if ship:verticalSpeed>=100{
        set flag to true.
        if (ship:altitude>15000){
            set pi to round(90*(1-(ship:apoapsis/altitude_d))^1,6).
            print "(Heading,Pitch) :"+di+","+pi+","+eta:apoapsis+" sec".
            if ship:altitude>=0.8*ship:orbit:body:atm:height{
                set vsteer to 1.
                set flag to false.
            //  print flag.
            }
        }
    }
    set maxQ to ship:q.
    wait 0.1. 
    if (ship:maxThrust=0){
        set vsteer to 1.
        print "Main engine cut-off".
        wait until stage:ready.
        stage.
        wait until stage:ready.
        stage.
        wait 0.1.
        set engi to findActiveEngine(). 
    }else if checkEngineStatus(){
        wait 0.1.
        set engi to findActiveEngine(). 
    }
    if(flag){
        //print "=====ENGINE SCRIPT ACTIVE=====".
        //print "THROTTLE:"+vsteer.
        if (maxQ<ship:q)
            set vsteer to acheiveTWR(vsteer,engi,1.7).
        else{
            print "maxQ reached.".
            set vsteer to acheiveTWR(vsteer,engi,1.35).
        }
    }
}
lock throttle to 0.
print "Atmos height:"+ship:orbit:body:atm:height.
unlock steering.
wait 1.
set SAS to true.
set RCS to true.
// SOS Μεταξύ της εκκινησης του SAS και του SASMODE πρέπει να υπάρχει
//χρονική καθυστέρηση
wait 1.
set SASMODE to "PROGRADE".
//SOS το SASMODE πρεπει να γραφετέ κεφαλαία
wait until ship:altitude>=ship:orbit:body:atm:height+500.
print "Ascent complete".
}

function main{
     set ag1 to false.//Close electric's and antenna 
     set ag2 to false.//Close ladders
     print "=== Eve Lifter Script Active ===".
    doAscent(altitude_d,thrust_d,direction_d).
    print "Circulization burn iminent.".
    circulizeInitialOrbit().
    print "=== Program Ended ===".
}

main().