epsilon = 0.01;

module ring(height, outerR, innerR, fn=100) {
    difference() {
        cylinder(height, outerR, outerR, $fn=fn);
        translate([0, 0, -epsilon]) cylinder(height+epsilon*2, innerR, innerR, $fn=fn);
    }
}
