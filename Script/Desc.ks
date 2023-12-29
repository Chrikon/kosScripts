//author : C P Konstantinidis

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
        if(0.00980966720709589*ship:sensors:pres>0.001)
            set paranomastis to paranomastis+i:possiblethrustat(0.00980966720709589*ship:sensors:pres)/i:ispat(0.00980966720709589*vess:sensors:pres).
        else
            set paranomastis to paranomastis+i:possiblethrust/i:vacuumisp.
    }
    return 9.81*ariumitis/paranomastis.
}

function calculatePossibleThrust{
    parameter ind is findActiveEngine().
    local thr is 0.
    for i in ind{
        if(0.00980966720709589*ship:sensors:pres>0.001)
           set thr to thr+i:possiblethrustat(0.00980966720709589*ship:sensors:pres).
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
    //print radius1.
    if(SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT<50){
        set sasmode to "NORMAL".
        wait 10.
        stage.
        wait until stage:ready.
        stage.
    }
    set SAS to false.
    lock steering to x.
    print "DeltaV of stage:"+(SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT).
    if(SHIP:STAGEDELTAV(SHIP:STAGENUM):CURRENT>=x:deltav:MAG){
        print "Simple Excution".
        simpleExecution(x,calculateBurnTime(x,findActiveEngine()),radius1,lim,opt).  }  
    else{
        print "Two stage Excution".
        twoStageExecution(x,improveBurnTime(x,findActiveEngine()),radius1,lim,opt).}
    set SAS to TRUE.
    wait 2.
    set SASMODE to "PROGRADE".
}

function twoStageExecution{
   parameter x,burnTime,radious,lim is 0.1,opt is 1.
   LIST ENGINES in eng.
    print "two stage execution".
    print burnTime.
    local a is x:time-(burnTime[0]+burnTime[1])/2.
    print "t1:"+burntime[0].
    wait until time:seconds>=a.
    lock throttle to 1.
    wait until ship:maxthrust=0. 
    stage.
    lock throttle to 0.
    wait 1.
    stage.
    wait until stage:ready.
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
                    print "Executing option 1".
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



function desaccelaration{
   // parameter planet is "Duna".
   // wait until ship:orbit:body:name=planet.
   // LOCAL m_time IS TIME:SECONDS + ship:orbit:ETA:periapsis-3600.
    //print timestamp(m_time):clock.
   // wait until time:seconds>m_time.
    set m_time to TIME:SECONDS + ship:orbit:ETA:periapsis.
    // what will the magnitude of our velocity be at that time
    LOCAL v0 IS VELOCITYAT(SHIP, m_time):orbit:MAG.
    PRINT "V0="+v0.
    //Planet standard graviatational parameter
    LOCAL V1 IS sqrt(ship:orbit:body:mu/(ship:orbit:body:radius+ship:orbit:periapsis)).
    PRINT "V1="+V1.
    SET DV TO v1-v0.
    PRINT "dV="+DV.
    set mnv to node(m_time,0,0,DV).
    ADD mnv.
    executeManuver(mnv,500,1,1).
    REMOVE mnv.
}


function main{
    print "=== Script active ===".
    desaccelaration().
    print "=== Program ended ===".
}

main().