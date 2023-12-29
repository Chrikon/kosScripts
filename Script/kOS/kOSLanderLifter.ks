//author : C P Konstantinidis

parameter altitude_d is 100000,thrust_d is 0.2,direction is 90.

function checkFuelTanks{
    set tanks to ship:partsnamed("fuelTank.long").
    for tank in tanks{
        if tank:mass<=tank:drymass{
            print "Tank: "+tank:name+" decoupled".
            tank:decoupler:getModule("ModuleAnchoredDecoupler"):doEvent("decouple").
        }
    }    
}

function acheiveTWR{
    parameter var.
    parameter engi.
    declare local thr to calculateNowThrust(engi).
    local twr is ((thr)*(ship:orbit:body:radius+ship:altitude)^2)/(ship:MASS*ship:orbit:body:mu).
    local ratio is 1-((twr)/3).//3 for the drag forces
    local factor is var+(0.3/ship:sensors:acc:mag)*ratio.
    print "TWR:"+twr+", Coeff:"+ratio+", Fac:"+factor.
    if factor>=1
        return 1.
    else if factor<=0
        return 0.
    else
        return factor.
}

function g{
    parameter h is 0.
    return ship:orbit:body:mu/((ship:orbit:body:radius+h)^2).
}

function acheiveTWRVacuum{
    parameter var.
    parameter engi.
    declare local thr to calculateNowThrust(engi).
    local twr is ((thr)*(ship:orbit:body:radius+ship:altitude)^2)/(ship:MASS*ship:orbit:body:mu).
    local ratio is 1-((twr)/3).
    //local factor is var+(0.3/ship:sensors:acc:mag)*ratio.
    local factor is var+0.1*ratio.
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

function improveBurnTime{
    parameter x,ind.
    local part1 is SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT.
    local part2 is x:deltav:mag-part1.
    local ve_part1 is calculateExhaustSpeedg(ind).
    local thrust_part1 is calculatePossibleThrust(ind).
    local part1_a is part1/ve_part1.
    local t1 is round((ve_part1*ship:mass/thrust_part1)*(1-(1/constant:e^part1_a)),1).
    print "t1:"+t1.
    list parts in p.
    local mas to 0.
        for pa in p{
            IF PA:STAGE=SHIP:stagenum OR PA:STAGE=ship:stagenum-1{
            SET mas to mas+pa:mass.    
            print pa:name+",mass:"+pa:mass.
        }
    }
    print "Total mas of ship: "+ship:mass. 
    print "Total mass of stage:"+mas.
    set remainingmass to ship:mass-mas.
    print "Remaining mass of stage:"+remainingmass.
    list engines in eng.
    declare local rem_eng is list().
    for i in eng{
        if i:stage=ship:stagenum-2
            rem_eng:add(i).
    }
    local ve_part2 is calculateExhaustSpeedg(rem_eng).
    local thrust_part2 is calculatePossibleThrust(rem_eng).
    local part2_a is part2/ve_part1.
    local t2 is round(((ve_part2*remainingmass)/thrust_part2)*(1-(1/constant:e^part2_a)),1).
    local t to list(t1,t2).
    print "t1:"+t1+",t2:"+t2+",tol:"+(t1+t2).
    return t.
}

function doVacuumAscent{
print "===Lifter Script Active===".
//print "Countdown:".
//from{local count is 10.} until count=0 step {set count to count -1.}do{
    //print "..."+count.
  //  wait 1.
//}
declare local di is direction.
declare local pi is 90.
declare local engi is findActiveEngine().
set sas to false.
wait 0.2.
lock steering to heading(di,pi).
print "Heading :"+direction+","+pi.
declare local vsteer to thrust_d.
set thr to vsteer.
lock throttle to thr.
wait until ship:verticalspeed>2.
set gear to false.
for i in engi{
    print i:name.
}
until ship:apoapsis>=altitude_d{
    set thr to acheiveTWRVacuum(thr,engi).
    if ship:verticalSpeed>=50{
        set pi to 15.
        print "Heading :90,15".
    }
    //set vsteer to acheiveTWR(vsteer,engi).
    checkFuelTanks().
    wait 0.2.
 }
 lock throttle to 0.
unlock steering.
wait 1.
set SAS to true.
// SOS Μεταξύ της εκκινησης του SAS και του SASMODE πρέπει να υπάρχει
//χρονική καθυστέρηση
wait 1.
//SOS το SASMODE πρεπει να γραφετέ κεφαλαία
set SASMODE to "PROGRADE".

}

function doAscent{ 
parameter ship_altitude,ship_throttle.
print "===Lifter Script Active===".
//print "Countdown:".
//from{local count is 10.} until count=0 step {set count to count -1.}do{
    //print "..."+count.
  //  wait 1.
//}
declare local di is direction.
declare local pi is 90.
declare local flag is false.
declare local engi is findActiveEngine().
set sas to false.
wait 0.2.
lock steering to heading(di,pi).
print "Heading :90,90".
declare local vsteer is ship_throttle.
lock throttle to vsteer.
wait until ship:verticalspeed>2.
set gear to false.

for i in engi{
    print i:name.
}
until ship:apoapsis>=ship_altitude{
    if ship:verticalSpeed>=100{
        set flag to true.
        set pi to round(90*(1-(ship:apoapsis/altitude_d))^2,2).
        print "(Heading,Pitch) :"+di+","+pi+","+eta:apoapsis+" sec".
        if ship:altitude>=2*ship:orbit:body:atm:height/3{
            set vsteer to 1.
          //  print flag.
        }
    } 
    checkFuelTanks().
    if(flag){
        //print "=====ENGINE SCRIPT ACTIVE=====".
        lock throttle to vsteer.
        //print "THROTTLE:"+vsteer.
        set vsteer to acheiveTWR(vsteer,engi).
    }
    wait 0.2.
}
lock throttle to 0.
print "Atmos height:"+ship:orbit:body:atm:height.
unlock steering.
wait 1.
set SAS to true.
// SOS Μεταξύ της εκκινησης του SAS και του SASMODE πρέπει να υπάρχει
//χρονική καθυστέρηση
wait 2.
//SOS το SASMODE πρεπει να γραφετέ κεφαλαία
set SASMODE to "PROGRADE".
until (ship:altitude>=ship:orbit:body:atm:height){
    wait 5.
    if ship:apoapsis<ship_altitude{
        lock throttle to 0.2.
        wait until ship:apoapsis>=ship_altitude.
        lock throttle to 0.
    }
}
if ship:apoapsis<ship_altitude{
        lock throttle to 0.2.
        wait until ship:apoapsis>=ship_altitude.
        lock throttle to 0.
}
wait until ship:altitude>ship:orbit:body:atm:height+500.
print "Ascent complete".
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
    parameter x,radius1,lim is 0.1,opt is 1.
    if(SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT<=90){
        wait until stage:ready.
        stage.
        wait until stage:ready.
        stage.
    }
    set SAS to false.
    lock steering to x.
    //print radius1.
    print "DeltaV of stage:"+(SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT).
    wait 10.
    declare local ind is findActiveEngine().
    print ind.
    if(SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT>=x:deltav:MAG)
        simpleExecution(x,calculateBurnTime(x,ind),radius1,lim,opt).    
    else
        twoStageExecution(x,improveBurnTime(x,ind),radius1,lim,opt).
    set SAS to TRUE.
    wait 2.
    set SASMODE to "PROGRADE".
}

function twoStageExecution{
   parameter x,burnTime,radious,lim is 0.1,opt is 1.
    print "two stage execution".
    print burnTime.
    local a is x:time-(burnTime[0]+burnTime[1])/2.
    print "t1:"+burntime[0].
    wait until time:seconds>=a.
    lock throttle to 1.
    wait until ship:maxthrust=0. 
    wait until stage:ready.
    stage.
    lock throttle to 0.
    wait until stage:ready.
    stage.
    local b is time:seconds+calculateBurnTime(x,findActiveEngine()).
    lock throttle to 1.
    print "t2:"+b.
    print "Option"+opt.
    //wait 0.01.
    declare local originalVector to x:burnvector.
   if opt=1{
                    until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:deltav:mag<0.1  or (ship:apoapsis>0 and ship:apoapsis-ship:periapsis<=radious){
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }else if opt=2{
                    until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:deltav:mag<0.1  or ship:apoapsis>=radious{
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }else if opt=3{
                    until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:burnvector:mag<0.1 or ship:periapsis>=radious{
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }else if opt=4{
                    until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:burnvector:mag<0.1 or ship:periapsis<=radious{
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }else if opt=5{
                    until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:burnvector:mag<0.1 or ship:apoapsis<=radious{
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }
    lock throttle to 0.
     unlock steering.
}

function simpleExecution{
    parameter x,burnTime,radious,lim is 1,opt is 1.
    print "Burntime:"+burnTime.
    local a is x:time-(burnTime/2).
    local b is x:time+(burnTime/2).
    print "Option"+opt.
    wait until time:seconds>=a.
    declare local originalVector to x:burnvector.
            if burnTime>=2{
                print "Throttle 100%".
                lock throttle to 1.
               if opt=1{
                    until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:deltav:mag<0.1  or (ship:apoapsis>0 and ship:apoapsis-ship:periapsis<=radious){
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }else if opt=2{
                    until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:deltav:mag<0.1  or ship:apoapsis>=radious{
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }else if opt=3{
                    until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:burnvector:mag<0.1 or ship:periapsis>=radious{
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }else if opt=4{
                    until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:burnvector:mag<0.1 or ship:periapsis<=radious{
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }else if opt=5{
                    until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:burnvector:mag<0.1 or ship:apoapsis<=radious{
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }
            }else{
                print "Throttle 10%".
                lock throttle to 0.1.
                if opt=1{
                    until vectorDif(x:burnvector,originalVector,lim)  or x:deltav:mag<0.1  or (ship:apoapsis>0 and ship:apoapsis-ship:periapsis<=radious){
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }else if opt=2{
                    until vectorDif(x:burnvector,originalVector,lim) or x:deltav:mag<0.1  or ship:apoapsis>=radious{
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }else if opt=3{
                    until vectorDif(x:burnvector,originalVector,lim) or x:burnvector:mag<0.1 or ship:periapsis>=radious{
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }else if opt=4{
                    until vectorDif(x:burnvector,originalVector,lim) or x:burnvector:mag<0.1 or ship:periapsis<=radious{
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }else if opt=5{
                    until vectorDif(x:burnvector,originalVector,lim) or x:burnvector:mag<0.1 or ship:apoapsis<=radious{
                        set originalVector to x:burnvector.
                        print x:burnvector:mag.
                    }
                }
            }
    lock throttle to 0.
    unlock steering.
}

function isMAnueverFeasible{
    parameter mnv.
    local t_burn is 0.
    ADD mnv.
    if(SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT>=mnv:deltav:MAG)
        set t_burn to calculateBurnTime(mnv).
    else
        set t_burn to improveBurnTime(mnv).
    REMOVE mnv.
    if(time:seconds-1<mnv:time+t_burn/2)
        return true.
    else
        return false.
}

function circulizeInitialOrbit{
    local manuverNode is createManuverNode().
    add manuverNode.
    executeManuver(manuverNode,1000,1,1).
    REMOVE manuverNode.
}



function main{
     set ag1 to false.//Close electrics+antenna
     set ag2 to false.// and ladders
     print "Countdown:".
from{local count is 10.} until count=0 step {set count to count -1.}do{
    print "..."+count.
    wait 1.
}
print "Launch.".
 if(ship:sensors:pres>0.1){
    doAscent(altitude_d,thrust_d).
 }else{
    doVacuumAscent().
 }
    circulizeInitialOrbit().
}

main().