//author : C P Konstantinidis

parameter limi is 1.

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
        if(ship:sensors:pres>0.001)
            set paranomastis to paranomastis+i:possiblethrustat(ship:sensors:pres)/i:ispat(0.00980966720709589*vess:sensors:pres).
        else
            set paranomastis to paranomastis+i:possiblethrust/i:vacuumisp.
    }
    return 9.81*ariumitis/paranomastis.
}

function calculatePossibleThrust{
    parameter ind is findActiveEngine().
    local thr is 0.
    for i in ind{
        if(ship:sensors:pres>0.001)
           set thr to thr+i:possiblethrustat(ship:sensors:pres).
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
    parameter x,lim is 0.1.
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
        simpleExecution(x,calculateBurnTime(x,findActiveEngine()),lim).  }  
    else{
        print "Two stage Excution".
        twoStageExecution(x,improveBurnTime(x,findActiveEngine()),lim).}
    set SAS to TRUE.
    wait 2.
    set SASMODE to "PROGRADE".
}

function twoStageExecution{
   parameter x,burnTime,lim is 0.1.
   LIST ENGINES in eng.
    print "two stage execution".
    print burnTime.
    local a is x:time-(burnTime[0]+burnTime[1])/2.
    print "t1:"+burntime[0].
    wait until time:seconds>=a.
    lock throttle to 1.
    wait until ship:maxthrust=0. 
    lock throttle to 0.
    wait until stage:ready.
    stage.
    wait until stage:ready.
    stage.
    declare local burnT to calculateBurnTime(x,findActiveEngine()).
    local b is time:seconds+burnT.
    //wait 0.01.
    declare local originalVector to 0.
     if (burnT>1){
        set originalVector to x:burnvector.
        lock throttle to 1.
         until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:deltav:mag<0.1{
            set originalVector to x:burnvector.
            print x:burnvector:mag.
        }
    }else{
        set originalVector to x:burnvector.
        lock throttle to 0.2.
         until vectorDif(x:burnvector,originalVector,lim) or x:deltav:mag<0.1{
            set originalVector to x:burnvector.
            print x:burnvector:mag.
        }
    }
    lock throttle to 0.
    unlock steering.
}

function simpleExecution{
    parameter x,burnTime,lim is 1.
    declare local originalVector to 0.
    print "Burntime:"+burnTime.
    local a is x:time-(burnTime/2).
    local b is x:time+(burnTime/2).
    print "Limit "+lim.
    if (burnTime>1){
        wait until time:seconds>=a.
        set originalVector to x:burnvector.
        lock throttle to 1.
        until vectorDif(x:burnvector,originalVector,lim) or time:seconds>b or x:deltav:mag<0.1{
            set originalVector to x:burnvector.
            print x:burnvector:mag.
        }
    }else{
        wait until time:seconds>=a.
        set originalVector to x:burnvector.
        lock throttle to 0.2.
        until vectorDif(x:burnvector,originalVector,lim) or x:deltav:mag<0.1{
            set originalVector to x:burnvector.
            print x:burnvector:mag.
        }
    }
    
    lock throttle to 0.
    unlock steering.
}



function manuver{
    if(hasNode)
        executeManuver(nextNode,limi).
    else
        print "Nothing to execute".
}


function main{
    print "=== Script active ===".
    manuver().
    print "=== Program ended ===".
}

main().