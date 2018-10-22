
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 
    for (x in robot.links) {
        robot.links[x].xform = generate_identity(4);
    }    

    for (x in robot.joints) {
        robot.joints[x].xform = generate_identity(4);
    }
    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }
    // console.log("FDkine");
    

    kineval.buildFKTransforms();
    // STENCIL: implement kineval.buildFKTransforms();

}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
kineval.buildFKTransforms=function buildFKTransforms(link){
    // console.log("time");
    traverseFKBase();
    traverseFKLink(robot.base); 
    return
}

function traverseFKBase(){
    tx = robot.origin.xyz[0];
    ty = robot.origin.xyz[1];
    tz = robot.origin.xyz[2];
    x = robot.origin.rpy[0];
    y = robot.origin.rpy[1];
    z = robot.origin.rpy[2];
    D = generate_translation_matrix(tx,ty,tz);
    Rx = generate_rotation_matrix_X(x);
    Ry = generate_rotation_matrix_Y(y);
    Rz = generate_rotation_matrix_Z(z);
    var DR = [];
    
        DR = matrix_multiply(D,Rz);
        DR = matrix_multiply(DR,Ry);
        DR = matrix_multiply(DR,Rx);
    
    
    // robot.links[robot.base].xform= DR;
    //mstack.push(matrix_multiply(mstack[mstack.length-1],DR));
    robot.links[robot.base].xform=matrix_multiply(mstack[mstack.length-1],DR);
    robot_heading = matrix_multiply(robot.links[robot.base].xform, [[0],[0],[1],[1]]);
    robot_lateral = matrix_multiply(robot.links[robot.base].xform, [[1],[0],[0],[1]]);
    //robot.origin.xform=mstack[mstack.length-1];
    //robot_heading = matrix_multiply(robot.links[robot.base].xform, [[0],[0],[1],[1]]);
    //robot_lateral = matrix_multiply(robot.links[robot.base].xform, [[1],[0],[0],[1]]);
    // console.log("base");
    // console.log(robot.links[robot.base].xform);
    if (robot.links_geom_imported) {

        var conver_mat = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
        robot.links[robot.base].xform = matrix_multiply(robot.links[robot.base].xform, conver_mat);

        // console.log("imported");
        // console.log(robot.links[robot.base].xform);
    }
    
    mstack.push(robot.links[robot.base].xform);
    // console.log("base");
    // console.log(mstack);
}

function traverseFKLink(link){
    var i,j;
    //i=number of joint connected to one link
    // console.log(link);
    // if (robot.links[link].children.length){
        for (i=0;i<robot.links[link].children.length;i++){
            joint = robot.links[link].children[i];
            tx = robot.joints[joint].origin.xyz[0];
            ty = robot.joints[joint].origin.xyz[1];
            tz = robot.joints[joint].origin.xyz[2];
            x = robot.joints[joint].origin.rpy[0];
            y = robot.joints[joint].origin.rpy[1];
            z = robot.joints[joint].origin.rpy[2];
            D = generate_translation_matrix(tx,ty,tz);
            Rx = generate_rotation_matrix_X(x);
            Ry = generate_rotation_matrix_Y(y);
            Rz = generate_rotation_matrix_Z(z);
            var DR = [];
           
                DR = matrix_multiply(D,Rz);
                DR = matrix_multiply(DR,Ry);
                DR = matrix_multiply(DR,Rx);
      
            mstack.push(matrix_multiply(mstack[mstack.length-1],DR));
            
            // console.log("add");
            // console.log(mstack);
            traverseFKJoint(joint);
            
        }   
        if (mstack.length>2){
            robot.links[link].xform = mstack[mstack.length-1];
            
            joint=robot.links[link].parent;
            for (j=0;j<joint.length;j++){
                joint2=joint[j];
                robot.joints[joint2].xform = robot.links[link].xform;
            }
            // console.log(joint);
            // console.log(robot.joints[joint].xform);
        }
        mstack.pop();
        // console.log("pop");
        // console.log(link);
        // console.log(robot.links[link].xform);
        // console.log(mstack);
        
    
        return
        
    // }

    // else{
    //     robot.links[link].xform = mstack.pop();
    //     joint=robot.links[link].parent;
    //     robot.joints[joint].xform = robot.links[link].xform;
    //     console.log("3");
    //     console.log(link);
    //     console.log(joint);
    //     return
    // }   
}

function traverseFKJoint(joint){
    traverseFKLink(robot.joints[joint].child);
    return
}