$fa = 1;
$fs = 0.01;

camera_hole_r = 0.1;
camera_hole_pitch_w=2.1;
camera_hole_pitch_h=1.25;
upper_hole_h = 1;

module half(){
    
// Base
difference(){
    cube([3.25, 8.7, 0.2]);   
   translate([2.45,0.6,-0.05]) 
        cylinder(h=0.3, r=0.1);
   translate([2.45,6.4,-0.05]) 
        cylinder(h=0.3, r=0.1);
}
// Side Stopper
translate([3.05, 0, 0.2])
cube([0.2, 7, 0.3]);
    
// Front Stopper
translate([2.85, 0, 0.2])
    cube([0.4, 0.2, 0.3]);

//Back Stopper
translate([2.85, 7, 0.2])
    cube([0.4, 0.2, 0.3]);

// Camera Mount
difference(){
    translate([0, 0, 0.2])
        cube([1.3, 0.2, 2.4]);
    
    // Upper hole
    translate([camera_hole_pitch_w/2, 0.5, upper_hole_h])
        rotate([90, 0, 0])
        cylinder(h=1, r=camera_hole_r);

    // Lower hole
    translate([camera_hole_pitch_w/2, 0.5, upper_hole_h + camera_hole_pitch_h])
        rotate([90, 0, 0])
        cylinder(h=1, r=camera_hole_r);

}

}

half();
mirror([1, 0, 0]){
half();
}