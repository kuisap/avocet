epsilon = 0.001;
$fn = 100;
debug_shaft=false;

module halfCylinder(height, r) {
    mirror([0, 1, 0])
    difference() {
        cylinder(height, r, r, $fn=$fn);
        translate([-r, 0, -epsilon])
            cube(size=[r*2+epsilon, r*2+2*epsilon, height+epsilon*2]);
    }
}

module halfShaft(height, outerR, centerR, wireR, centerFn=6) {
    difference(){
        union() {
            difference() {
                halfCylinder(height, outerR);
                translate([-wireR/2, (outerR + centerR)/2, -epsilon]) 
                    cube([wireR, height, (height+epsilon)*2]);
            }
            difference() {
                marginR = 2;
                r = outerR+marginR;
                translate([0, 0, 5])
                    cylinder(10, r, r, center=true);
                mirror ([0, 1, 0])
                    translate([-r, 0, -epsilon])
                        cube(size=[r*2+epsilon, r*2+2*epsilon, height+epsilon*2]);
            }
        }
        cylinder((height+epsilon)*2, centerR, centerR, center=true, $fn=centerFn);
    }
}

if (debug_shaft) {
    color("red") 
        union() {
            halfShaft(height=100, outerR=15, centerR=2.3, wireR=3.5);
            mirror([0, 1, 0])
                halfShaft(height=100, outerR=15, centerR=2.3, wireR=3.5);
        }
}