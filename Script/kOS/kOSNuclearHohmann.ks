//author : C P Konstantinidis

parameter newRad.

function acheiveTWR{
    parameter var.
    parameter engi.
    declare local thr to calculateNowThrust(engi).
    local twr is ((thr)*(ship:orbit:body:radius+ship:altitude)^2)/(ship:MASS*ship:orbit:body:mu).
    local ratio is 1-(twr)/1.5.
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
            set paranomastis to paranomastis+i:possiblethrust/i:vacuumisp.
    }
    return 9.81*ariumitis/paranomastis.
}

function calculatePossibleThrust{
    parameter ind is findActiveEngine().
    local thr is 0.
    for i in ind{
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
  parameter x,ind is findActiveEngine().
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


function createManuverNode{
    parameter exectime.
    parameter dv.
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
                    until vectorDif(x:burnvector,originalVector,lim) or x:deltav:mag<0.1  or (ship:apoapsis>0 and ship:apoapsis-ship:periapsis<=radious){
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
    lock throttle to 0.
     unlock steering.
}

function simpleExecution{
    parameter x,burnTime,radious,lim is 1,opt is 1.
    print "Burntime:"+burnTime.
    local a is x:time-(burnTime/2).
    //local b is x:time+(burnTime/2).
    print "Option"+opt.
    wait until time:seconds>=a-4.
    declare local originalVector to x:burnvector.
            if burnTime>=2{
                print "Throttle 100%".
                lock throttle to 1.
               if opt=1{
                    until vectorDif(x:burnvector,originalVector,lim) or x:deltav:mag<0.1  or (ship:apoapsis>0 and ship:apoapsis-ship:periapsis<=radious){
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

function calculateDVofneworbit{
    parameter r2,r1 is ship:periapsis.
    declare local dv1 is sqrt(ship:orbit:body:mu/(ship:orbit:body:radius+r1))*(sqrt((2*(ship:orbit:body:radius+r2))/(2*ship:orbit:body:radius+r1+r2))-1).
    return dv1.
}

function calculateCirculizationDVofneworbit{
    parameter r2,r1 is ship:periapsis.
    declare local dv2 is sqrt(ship:orbit:body:mu/(ship:orbit:body:radius+r2))*(1-sqrt((2*(ship:orbit:body:radius+r1))/(2*ship:orbit:body:radius+r1+r2))).
    return dv2.
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

 function executeHohmannTransfer{
     wait 2.
    parameter r2,r1 is ship:periapsis.
    if r2>r1{
        declare local nodetime1 is eta:periapsis.
        declare local nodedv1 is calculateDVofneworbit(r2). 
        declare local mnv1 to createManuverNode(nodetime1,nodedv1).
        if (isMAnueverFeasible(mnv1)){
            ADD mnv1.
            executeManuver(mnv1,r2,1,2).
            REMOVE mnv1.
        }else{
            set mnv1 to createManuverNode(eta:periapsis+ship:orbit:period,nodedv1).
            ADD mnv1.
            executeManuver(mnv1,r2,1,2).
            REMOVE mnv1.
        }
        declare local nodetime2 is eta:apoapsis.
        declare local nodedv2 is calculateCirculizationDVofneworbit(r2). 
        declare local mnv2 to createManuverNode(nodetime2,nodedv2).
        ADD mnv2.
        executeManuver(mnv2,r2,1,3).
        REMOVE mnv2. 
    }else if r2=r1
        print "Same radius,nothing to execute".
    else if r2<r1{
        declare local nodedv2 is calculateDVofneworbit(r2,r1).
        declare local nodetime2 is eta:apoapsis.
        declare local mnv2 to createManuverNode(nodetime2,nodedv2).
        if (isMAnueverFeasible(mnv2)){
            ADD mnv2.
            executeManuver(mnv2,r2,1,4).
            REMOVE mnv2.
        }else{
            set mnv2 to createManuverNode(eta:apoapsis+ship:orbit:period,nodedv2).
            ADD mnv2.
            executeManuver(mnv2,r2,1,4).
            REMOVE mnv2.
        }
        declare local nodetime3 is eta:periapsis.
        declare local nodedv3 is calculateCirculizationDVofneworbit(r2,r1). 
        declare local mnv3 to createManuverNode(nodetime3,nodedv3).
        ADD mnv3.
        executeManuver(mnv3,r2,1,5).
        REMOVE mnv3. 
    }
 }

 function circulizeOrbit{
    declare local r2 is ship:periapsis.
    declare local r1 is ship:apoapsis.
    if(r1-r2>1000){
    declare local nodetime3 is eta:periapsis.
    declare local dv2 is sqrt(ship:orbit:body:mu/(ship:orbit:body:radius+r2))*(1-sqrt((2*(ship:orbit:body:radius+r1))/(2*ship:orbit:body:radius+r1+r2))).
    declare local mnv3 to createManuverNode(nodetime3,dv2).
    ADD mnv3.
    executeManuver(mnv3,r2,1,5).
    REMOVE mnv3. 
    }else
        print "Marginal distance r2-r1".
 }

function main{
   circulizeOrbit().
   executeHohmannTransfer(newRad).
}

main().