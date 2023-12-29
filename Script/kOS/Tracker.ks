function main{
   until false{
    local inc is vAng(minmus:north:vector,kerbin:north:vector).
    local posAng is vAng(ship:position-kerbin:position,(minmus:position-kerbin:position)).
    local upPos is vAng(kerbin:up:vector,ship:position-kerbin:position).
    local zeta is kerbin:position:y-minmus:position:y.
    if(inc>5.8 and posAng>89 and upPos<45 and zeta<0)
      PRINT "Ascending point 96 deg ejection angle".
    else if(inc>5.8 and posAng>89 and upPos>135 and zeta>0)
    PRINT "Descending point 84 deg ejection angle".
    else{
      print "NORTHVECTORS: "+vAng(minmus:north:vector,kerbin:north:vector).
    print "Position: "+vAng(ship:position-kerbin:position,(minmus:position-kerbin:position)).
    print "up pos"+vAng(kerbin:up:vector,ship:position-kerbin:position).
    print "Zeta"+zeta.
    }
      wait 60.
    //CLEARVECDRAWS().
   }
}

main().