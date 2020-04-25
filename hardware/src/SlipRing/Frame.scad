include <Util.scad>;
include <Receiver.scad>;

module frame(width, height, depth, centerR, t=5) {
    _round = 0.1;
        difference() {
            translate([-width/2, 0, 0])
                cube([width, height, depth]);
            margin = 1;
            translate([0, 0, -epsilon])
                cylinder(depth+epsilon*2, centerR+margin, centerR+margin, $fn=100);
            translate([0, 0, -depth/2-epsilon])
                cylinder(depth+epsilon*2, centerR+margin+1, centerR+margin+2, $fn=100);
            translate([-(width-t*2)/2, -t, t])
                cube([width-t*2, height-t*2, depth-t*2]);
        }
    }
}

debug_frame = true;

if (debug_frame) {
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
        color("red")
            translate([0, 18, -2.5])
                receiver(50, 30, 80, 12, 4);
        color("red")
            mirror([0, 1, 0])
                translate([0, 18, -2.5])
                    receiver(50, 30, 80, 12, 4);
    }
    frame(70, 60, 90, 15);
}

