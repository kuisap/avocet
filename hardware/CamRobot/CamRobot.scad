$fa = 1;
$fs = 0.4;

module wheel() {
rotate([90, 0, 0])
    translate([4.5, 1.8, -0.9])
    cylinder(h=1.8, r=1.8);

rotate([90, 0, 0])    
    translate([-4.5,1.8,-0.9])
    cylinder(h=1.8, r=1.8);
    
rotate([90, 0, 0])    
    translate([0,5,-0.9])
    cylinder(h=1.8, r=1.8);
    
path_pts = [
    [4.5, 0.9],
    [-4.5, 0.9],
    [-5.5, 4.2],
    [-1, 7.4],
    [1, 7.4],
    [5.5, 4.2]
];

translate([0,0.9,-0.9])
    rotate([90, 0, 0])
    linear_extrude(height=1.8)
    polygon(path_pts);
}


module body(){
translate([0, 5, 0])
    wheel();
translate([0, -5, 0])
    wheel();
translate([0,0,6.9])
cube([11,8.5,9], center=true);
}


module hand(){
translate([0, 0.5, 0])
rotate([90,0,0])
    cylinder(h=0.5, r=0.8);

translate([0, 0, -0.8])
    cube([3,0.5,1.6]);

translate([3, 0.5, 0])
    rotate([90,0,0])
    cylinder(h=0.5, r=0.8);

translate([3.3, 0, -0.73])
    rotate([0,-30,0])
    cube([3,0.5,1.6]);

translate([5.5, 0.5, 1.45])
    rotate([90,0,0])
    cylinder(h=0.5, r=0.8);
}

body();

translate([0, 4.2 ,9])
    hand();

translate([0, -4.7 ,9])
    hand();
