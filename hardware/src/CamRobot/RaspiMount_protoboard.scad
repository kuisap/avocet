$fa = 1;
$fs = 0.01;

//args
// stereo_camera: true:stereo camera mount, false:single camera mount
// camera_hole_r: radius of camera hole
// camera _hole_pitch_w: horizontal pitch of camera holes
// camera_hole_pitch_h: vertical pitch of camera holes
// camera_distance: distance between hole of 2 cameras. It is used  only for stereo camera mount
// upper_hole_h: height of upper camera holes
module RaspiCameraMount(stereo_camera=true, camera_hole_r=0.1, camera_hole_pitch_w=2.1, camera_hole_pitch_h=1.25, camera_distance=1,  upper_hole_h=1){
    cm_half(stereo_camera, camera_hole_r, camera_hole_pitch_w, camera_hole_pitch_h, camera_distance, upper_hole_h);
    mirror([1, 0, 0]){
    cm_half(stereo_camera, camera_hole_r, camera_hole_pitch_w, camera_hole_pitch_h, camera_distance, upper_hole_h);
    }
}


module cm_half(stereo_camera, camera_hole_r, camera_hole_pitch_w, camera_hole_pitch_h, camera_distance, upper_hole_h){
    
    // Base of Raspi Mount
    difference(){
        cube([3.75, 8.7, 0.2]);   
       translate([2.45,0.6,-0.05]) 
            cylinder(h=0.3, r=0.1);
       translate([2.45,6.4,-0.05]) 
            cylinder(h=0.3, r=0.1);
        
        // Make holes on Raspi mount
        trim_holes();
        
    }
    
    // Front-side stopper
    side_stopper();

    // Back-side Stopper    
    translate([0 , 6.2, 0])
        side_stopper();
        
    // Front Stopper
    front_stopper();

    //Back Stopper
    translate([0, 7, 0])
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
        translate([0.25, 0.5, -0.05])
            cube([0.1, 0.5, 0.3]);
            cylinder(r=0.1, h=0.3);   
    }
}

module trim_hole_line(){
    trim_hole();
    translate([0.7, 0, 0])
        trim_hole();
    translate([1.2, 0, 0])
        trim_hole();
    translate([1.8, 0, 0])
        trim_hole();
    translate([2.5, 0, 0])
        trim_hole();
}

module trim_holes(){
    trim_hole_line();
    translate([0, 1, 0])    
        trim_hole_line();
    translate([0, 2, 0])    
        trim_hole_line();
    translate([0, 3, 0])    
        trim_hole_line();
    translate([0, 4, 0])    
        trim_hole_line();
    translate([0, 5, 0])    
        trim_hole_line();
    translate([0, 6, 0])    
        trim_hole_line();
    translate([0, 7, 0])    
        trim_hole_line();
}

module front_stopper(){
    minkowski(){
        translate([3.4, 0.1, 0.2])
            cube([0.1, 0.1, 0.1]);
        rotate([90, 0, 0])
            cylinder(r=0.2, h=0.1);
    }
}

module side_stopper(){
    difference(){
    translate([3.55, 0, 0.2])
        minkowski()
        {
            translate([0, 0.2, 0])
                cube([0.1, 0.6, 0.6]);
            rotate([0, 90, 0])
                cylinder(r=0.2,h=0.1);
        }
    translate([3,0.5 ,0.7]) 
        rotate([0, 90, 0])
        cylinder(h=0.3, r=0.1);
    }    
}

module camera_mount(camera_hole_r, camera_hole_pitch_w, camera_hole_pitch_h, upper_hole_h){
    // Camera Mount
    difference(){
        
        translate([0, 0.2, 0.2])
            rotate([90, 0, 0])
            minkowski()
            {
                cube([(camera_hole_pitch_w )/2+0.1, camera_hole_pitch_h+upper_hole_h , 0.1]);
                cylinder(r=0.2,h=0.1);
            }
        
        
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

//args
//  protoboard_pitch_w, protoboard_pitch_h: pitch of prototyping board holes
//  protoboard_h: length between the raspi mount and the protoboard mount
// protoboard_w: length between the camera mount and the protoboard mount
module ProtoboardMount(protoboard_pitch_w = 3, protoboard_pitch_h = 5, protoboard_h=2, protoboard_w=1){
    pm_half(protoboard_pitch_w, protoboard_pitch_h, protoboard_h, protoboard_w);
    mirror([1, 0, 0]){
        pm_half(protoboard_pitch_w, protoboard_pitch_h, protoboard_h, protoboard_w);
    }
}

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

                    translate([-0.3, -0.1, 0]){
                        minkowski()
                        {
                            cube([0.4, 0.4, , 0.1]);
                            cylinder(r=0.2,h=0.1);
                        }
                    }
                        //cube([protoboard_pitch_w/2+0.5, protoboard_pitch_h + 1, 0.2]);
                        // front hole
                       translate([0,0,-0.05]) 
                            cylinder(h=0.3, r=0.1);
                     }
             }
         // front poll
        translate([ -0.3, 0.3, 0]) 
            cylinder(h=protoboard_h + 0.1, r=0.2);
     }
}



// Generate

//args: stereo_camera=true, camera_hole_r=0.1, camera_hole_pitch_w=2.1, camera_hole_pitch_h=1.25, camera_distance=1,  upper_hole_h=1
RaspiCameraMount();

// args: protoboard_pitch_w = 3, protoboard_pitch_h = 5, protoboard_h=2, protoboard_w=1
ProtoboardMount();

