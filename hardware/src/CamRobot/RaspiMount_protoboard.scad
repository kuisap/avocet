$fa = 1;
$fs = 0.1;

//args
// stereo_camera: true:stereo camera mount, false:single camera mount
// camera_hole_r: radius of camera hole
// camera _hole_pitch_w: horizontal pitch of camera holes
// camera_hole_pitch_h: vertical pitch of camera holes
// camera_distance: distance between hole of 2 cameras. It is used  only for stereo camera mount
// upper_hole_h: height of upper camera holes
module RaspiCameraMount(stereo_camera=false, camera_hole_r=1, camera_hole_pitch_w=21, camera_hole_pitch_h=12.5, camera_distance=10,  upper_hole_h=10, trim=false){
    cm_half(stereo_camera, camera_hole_r, camera_hole_pitch_w, camera_hole_pitch_h, camera_distance, upper_hole_h, trim);
    mirror([1, 0, 0]){
    cm_half(stereo_camera, camera_hole_r, camera_hole_pitch_w, camera_hole_pitch_h, camera_distance, upper_hole_h, trim);
    }
}

module cm_half(stereo_camera, camera_hole_r, camera_hole_pitch_w, camera_hole_pitch_h, camera_distance, upper_hole_h, trim){
    
    // Base of Raspi Mount
    difference(){
        cube([37.5, 87, 2]);   
       translate([24.5,6,-0.5]) 
            cylinder(h=3, r=1);
       // tapped hole
       translate([24.5,64,-0.5]) 
            cylinder(h=3, r=1);
        
        // Make holes on Raspi mount
        if(trim){
            trim_holes();
        }
    }
    
    // Front-side stopper
    side_stopper();

    // Back-side Stopper    
    translate([0 , 62, 0])
        side_stopper();
        
    // Front Stopper
    front_stopper();

    //Back Stopper
    translate([0, 70, 0])
        front_stopper();
    
    if(stereo_camera==false){
        camera_mount(camera_hole_r, camera_hole_pitch_w, camera_hole_pitch_h, upper_hole_h);
    } else {
        translate([camera_hole_pitch_w/2 +  camera_distance/2, 0, 0]){
            camera_mount(camera_hole_r, camera_hole_pitch_w, camera_hole_pitch_h, upper_hole_h);
            mirror([1, 0, 0]){            
                camera_mount(camera_hole_r, camera_hole_pitch_w, camera_hole_pitch_h, upper_hole_h);
            }
        }   
    }
}

module trim_hole(){
     minkowski(){
        translate([2.5, 5, -0.5])
            cube([1, 5, 3]);
            cylinder(r=1, h=3);   
    }
}

module trim_hole_line(){
    trim_hole();
    translate([7, 0, 0])
        trim_hole();
    translate([12, 0, 0])
        trim_hole();
    translate([18, 0, 0])
        trim_hole();
    translate([25, 0, 0])
        trim_hole();
}

module trim_holes(){
    trim_hole_line();
    translate([0, 10, 0])    
        trim_hole_line();
    translate([0, 20, 0])    
        trim_hole_line();
    translate([0, 30, 0])    
        trim_hole_line();
    translate([0, 40, 0])    
        trim_hole_line();
    translate([0, 50, 0])    
        trim_hole_line();
    translate([0, 60, 0])    
        trim_hole_line();
    translate([0, 70, 0])    
        trim_hole_line();
}



module front_stopper(){
    minkowski(){
        translate([34, 1, 2])
            cube([1, 1, 1]);
        rotate([90, 0, 0])
            cylinder(r=2, h=1);
    }
}

module side_stopper(){
    difference(){
    translate([35.5, 0, 2])
        minkowski()
        {
            translate([0, 2, 0])
                cube([1, 6, 6]);
            rotate([0, 90, 0])
                cylinder(r=2,h=1);
        }
    // tapped hole
    translate([35,5 ,7]) 
        rotate([0, 90, 0])
        cylinder(h=3, r=1);
    }    
}


module camera_mount(camera_hole_r, camera_hole_pitch_w, camera_hole_pitch_h, upper_hole_h){
    // Camera Mount
    difference(){
        
        translate([0, 2, 2])
            rotate([90, 0, 0])
            minkowski()
            {
                cube([(camera_hole_pitch_w )/2+1, camera_hole_pitch_h+upper_hole_h , 1]);
                cylinder(r=2,h=1);
            }
        
        
        // Upper hole
        translate([camera_hole_pitch_w/2, 5, upper_hole_h])
            rotate([90, 0, 0])
            cylinder(h=10, r=camera_hole_r);


        // Lower hole
        translate([camera_hole_pitch_w/2, 5, upper_hole_h + camera_hole_pitch_h])
            rotate([90, 0, 0])
            cylinder(h=10, r=camera_hole_r);

    }
}    
    



// Mount of Prototyping board
module pm_half(protoboard_pitch_w, protoboard_pitch_h, protoboard_h, protoboard_w){
    
    translate([0, protoboard_pitch_h/2 + protoboard_w, 0]){
        front_pb_mount(protoboard_pitch_w, protoboard_pitch_h, protoboard_h);
        mirror([0, 1, 0]){
            front_pb_mount(protoboard_pitch_w, protoboard_pitch_h, protoboard_h);
        }
    }
}

module front_pb_mount(protoboard_pitch_w, protoboard_pitch_h, protoboard_h){
    // set of front pole and mount
    translate([protoboard_pitch_w/2, - protoboard_pitch_h/2, 0]){
            translate([0, 0, protoboard_h]){
                difference(){

                    translate([-3, -1, 0]){
                        minkowski()
                        {
                            cube([4, 4, , 1]);
                            cylinder(r=2,h=1);
                        }
                    }
                        //cube([protoboard_pitch_w/2+0.5, protoboard_pitch_h + 1, 0.2]);
                        // front hole
                       translate([0,0,-0.5]) 
                            cylinder(h=3, r=1);
                     }
             }
         // front poll
        translate([ -3, 3, 0]) 
            cylinder(h=protoboard_h + 1, r=2);
     }
}

//args
//  protoboard_pitch_w, protoboard_pitch_h: pitch of prototyping board holes
//  protoboard_h: length between the raspi mount and the protoboard mount
// protoboard_w: length between the camera mount and the protoboard mount
module ProtoboardMount(protoboard_pitch_w = 41, protoboard_pitch_h = 66, protoboard_h=10, protoboard_w=10){
    pm_half(protoboard_pitch_w, protoboard_pitch_h, protoboard_h, protoboard_w);
    mirror([1, 0, 0]){
        pm_half(protoboard_pitch_w, protoboard_pitch_h, protoboard_h, protoboard_w);
    }
}


// Generate

//args: stereo_camera=false, camera_hole_r=0.1, camera_hole_pitch_w=2.1, camera_hole_pitch_h=1.25, camera_distance=1,  upper_hole_h=1, trim=true
RaspiCameraMount();

// args: protoboard_pitch_w = 3, protoboard_pitch_h = 5, protoboard_h=2, protoboard_w=1
ProtoboardMount();

