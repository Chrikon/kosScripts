//author : C P Konstantinidis
//Lift with booster's

parameter altitude_d is 100000,thrust_d is 1,direction_d is 90,t_spool is 1.06.

function acheiveTWR{
    parameter var.
    parameter engi.
    declare local thr to calculateNowThrust(engi).
    local twr is ((thr)*(ship:orbit:body:radius+ship:altitude)^2)/(ship:MASS*ship:orbit:body:mu).
    local ratio is 1-(twr)/2.
    print "TWR:"+twr+", Coeff:"+ratio.
    local factor is var+0.5*ratio.
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
        if(0.00986923267*ship:sensors:pres>0.001)
            set paranomastis to paranomastis+i:possiblethrustat(0.00986923267*ship:sensors:pres)/i:ispat(0.00986923267*ship:sensors:pres).
        else
            set paranomastis to paranomastis+i:possiblethrust/i:vacuumisp.
    }
    return 9.81*ariumitis/paranomastis.
}

function calculatePossibleThrust{
    parameter ind.
    local thr is 0.
    for i in ind{
        if(0.00986923267*ship:sensors:pres>0.001)
            set thr to thr+i:possiblethrustat(0.00986923267*ship:sensors:pres).
        else
            set thr to thr+i:possiblethrust.
    }
    return thr.
}

function calculateNowThrust{
    parameter ind.
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

function doAscent{ 
parameter ship_apoapsis,ship_throttle,ship_direction.
print "===Lifter Script Active===".
//print "Countdown:".
//from{local count is 10.} until count=0 step {set count to count -1.}do{
    //print "..."+count.
  //  wait 1.
//}
declare local di is ship_direction.
declare local pi is 90.
declare local flag is false.
lock steering to heading(di,pi).
print "(Heading,Pitch) : "+di+","+pi.
declare local vsteer is ship_throttle.
lock throttle to vsteer.
wait until stage:ready.
stage.
wait t_spool.
declare local engi is findActiveEngine().
for i in engi{
    print i:name.
}
until ship:apoapsis>ship_apoapsis{
    if ship:verticalSpeed>=100 {
        set flag to true.
        set pi to round(90*(1-(ship:apoapsis/altitude_d))^2,2).
        print "(Heading,Pitch) :"+di+","+pi+","+eta:apoapsis+" sec".
        if ship:altitude>=2*ship:orbit:body:atm:height/3{
            set vsteer to 1.
          //  print flag.
        }
    } 
    wait 0.1.
    if (ship:maxThrust=0 and flag){
        set vsteer to 1.
        print "Main engine cut-off".
        wait until stage:ready.
        stage.
        wait until stage:ready.
        stage.
        wait t_spool.
       set engi to findActiveEngine(). 
    }
    if(flag){
        //print "=====ENGINE SCRIPT ACTIVE=====".
        lock throttle to vsteer.
        //print "THROTTLE:"+vsteer.
        set vsteer to acheiveTWR(vsteer,engi).
    }
}
lock throttle to 0.
unlock steering.
wait 1.
set SAS to true.
// SOS Μεταξύ της εκκινησης του SAS και του SASMODE πρέπει να υπάρχει
//χρονική καθυστέρηση
wait 2.
//SOS το SASMODE πρεπει να γραφετέ κεφαλαία
//set SASMODE to "PROGRADE".
wait until ship:altitude>ship:orbit:body:atm:height+500.
if not(ship:partsnamed("fairingSize1p5"):empty){
    set fairing to ship:partsnamed("fairingSize1p5").
    fairing[0]:getModule("ModuleProceduralFairing"):doevent("deploy").
    print "Debunk".
}else if not(ship:partsnamed("fairingSize2"):empty){
    set fairing to ship:partsnamed("fairingSize2").
    fairing[0]:getModule("ModuleProceduralFairing"):doevent("deploy").
    print "Debunk".
}else if not(ship:partsnamed("fairingSize3"):empty){
    set fairing to ship:partsnamed("fairingSize3").
    fairing[0]:getModule("ModuleProceduralFairing"):doevent("deploy").
    print "Debunk".
}else if not(ship:partsnamed("fairingSize3Classic"):empty){
    set fairing to ship:partsnamed("fairingSize3Classic").
    fairing[0]:getModule("ModuleProceduralFairing"):doevent("deploy").
    print "Debunk".
}else if not(ship:partsnamed("nflv-fairing-5-1"):empty){
    set fairing to ship:partsnamed("nflv-fairing-5-1").
    fairing[0]:getModule("ModuleProceduralFairing"):doevent("deploy").
    print "Debunk".
}
wait 1.
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
   // if(SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT<=90){
     //   wait until stage:ready.
       // stage.
  //      wait until stage:ready.
    //    stage.
   // }
    set SAS to false.
    lock steering to x.
    //print radius1.
    print "DeltaV of stage:"+(SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT).
    wait 10.
    declare local ind is findActiveEngine().
    print ind.
    if (SHIP:DELTAV:CURRENT>=x:deltav:MAG){
        if(SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT>=x:deltav:MAG)
            simpleExecution(x,calculateBurnTime(x,ind),radius1,lim,opt).    
        else
            twoStageExecution(x,improveBurnTime(x,ind),radius1,lim,opt).
    }else
        print "Manuver not feasible".
    set SAS to TRUE.
    wait 2.
    set SASMODE to "PROGRADE".
}

function twoStageExecution{
   parameter x,burnTime,radious,lim is 0.1,opt is 1.
    print "two stage execution".
    print burnTime.
    //1.06 spool time of main engines
    local a is x:time-((burnTime[0]+burnTime[1])/2)-t_spool-0.1.
    print "t1:"+burntime[0].
    wait until time:seconds>=a.
    wait until stage:ready.
    stage.
    wait 0.1.
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
    local a is x:time-(burnTime/2)-t_spool-0.1.
    local b is x:time+(burnTime/2).
    print "Option"+opt.
    wait until time:seconds>=a.
    declare local originalVector to x:burnvector.
            if burnTime>=2{
                print "Throttle 100%".
                 wait until stage:ready.
                 stage.
                 wait 0.1.   
                lock throttle to 1.
               if opt=1{
                    until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:deltav:mag<0.1  or (ship:apoapsis>0 and abs(ship:apoapsis-ship:periapsis)<=radious){
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

 
function circulizeInitialOrbit{
    local manuverNode is createManuverNode().
    add manuverNode.
    executeManuver(manuverNode,1000,1,1).
    REMOVE manuverNode.
}

function main{
    if(ship:sensors:pres>0.1 and altitude_d>ship:orbit:body:atm:height){
       // set ag1 to true.
        doAscent(altitude_d,thrust_d,direction_d).
        //Αρχική manuver
        circulizeInitialOrbit().
    }else
        print "Unable to comply".
}

main().