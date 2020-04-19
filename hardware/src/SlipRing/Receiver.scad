include <Shaft.scad>;
include <Util.scad>;

debug_receiver = true;

module receiver (width, height, depth, pitch, r) {
    T = width/4;
    difference() {
        translate([-width/2, 0, 0]) 
            cube([width, height, depth]);
        translate([T/2, -epsilon, T*2/3])
            cube([width+epsilon*2, height+epsilon*2, depth]);
        mirror([1, 0, 0])
            translate([T/2, -epsilon, T*2/3])
                cube([width+epsilon*2, height+epsilon*2, depth]);
        translate([width/2-T, -epsilon, T])
            cube([width, height, depth]);
        mirror([1, 0, 0])
            translate([width/2-T, -epsilon, T])
                cube([width, height, depth]);
        for (i = [0:4]) {
            margin=5;
            translate([0, height/2, T+margin+i*pitch])
                rotate([0, 90, 0]) 
                    cylinder(width+epsilon*2, r, r, center=true);
        }
    }
}

if (debug_receiver) {
    union() {
        color("green") 
            halfShaft(height=90, outerR=15, centerR=2.3, wireR=8);
        color("green") 
            mirror([0, 1, 0])
                halfShaft(height=90, outerR=15, centerR=2.3, wireR=8);
        for (i = [0:4]){
            h=10;
            margin=2;
            color("gold") translate([0, 0, 10-epsilon+(h+margin)*i ]) ring(h, 16, 15);
            color("gray") translate([0, 0, 10-epsilon+h*(i+1)+margin*(i)]) ring(margin, 17, 15);
        }
    }

    color("red")
        translate([0, 18, -2.5])
            receiver(50, 30, 80, 12, 4);
}

