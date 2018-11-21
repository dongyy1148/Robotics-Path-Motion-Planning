
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

   // update time from start of trial
   cur_time = new Date();
   kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

   // get endeffector Cartesian position in the world
   endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

   // compute distance of endeffector to target
   kineval.params.trial_ik_random.distance_current = Math.sqrt(
           Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
           + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
           + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

   // if target reached, increment scoring and generate new target location
   // KE 2 : convert hardcoded constants into proper parameters
   if (kineval.params.trial_ik_random.distance_current < 0.01) {
       kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
       kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
       kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
       kineval.params.trial_ik_random.targets += 1;
       textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
   }
    // STENCIL: see instructor for random time trial code
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {
    endeffector_world = matrix_multiply_vector(robot.joints[endeffector_joint].xform, endeffector_position_local);
    //angle = robot.joints[endeffector_joint].angle;
    axis = robot.joints[endeffector_joint].axis;
    axis = vector_normalize(axis);
    axis = [axis[0], axis[1], axis[2], 1];
    if(robot.links_geom_imported) {
        type = robot.joints[endeffector_joint].type;
    }
    else {
        type = 'continuous';
    }
    var JT=[]; J0=[]; J1=[]; J2=[]; v1=[]; v2=[]; v3 = [];
    if (type === 'prismatic') {
        v1 = matrix_multiply_vector(robot.joints[endeffector_joint].xform,axis);
        v2 = matrix_multiply_vector(robot.joints[endeffector_joint].xform,[0,0,0,1]);
        J0 = vector_sub(v1,v2);
        J0 = vector_normalize(J0);
        J0 = [J0[0], J0[1], J0[2],0,0,0];
        JT.push(J0) ;
    }
    else if (type === 'continuous' || type === 'revolute') {
        v1 = matrix_multiply_vector(robot.joints[endeffector_joint].xform,axis);
        v2 = matrix_multiply_vector(robot.joints[endeffector_joint].xform,[0,0,0,1]);
        v3 = endeffector_world;
        J1 = vector_sub(v1,v2);
        J1 = vector_normalize(J1);
        J2 = vector_sub(v3,v2);
        J0 = vector_cross(J1,J2); 
        J0 = [J0[0], J0[1], J0[2], J1[0], J1[1], J1[2] ];
        JT.push(J0) ;
    }
    else {
        
    }
    link = robot.joints[endeffector_joint].parent;
    joint = robot.links[link].parent;
    while (typeof robot.joints[joint] !== "undefined") {
        //joint = robot.links[link].parent;
        angle = robot.joints[joint].angle;
        axis = robot.joints[joint].axis;
        axis = vector_normalize(axis);
        axis = [axis[0], axis[1], axis[2], 1];
        if(robot.links_geom_imported) {
            type = robot.joints[joint].type;
        }
        else {
            type = 'continuous';
        }
        if (type === 'prismatic') {
            v1 = matrix_multiply_vector(robot.joints[joint].xform,axis);
            v2 = matrix_multiply_vector(robot.joints[joint].xform,[0,0,0,1]);
            J0 = vector_sub(v1,v2);
            J0 = [J0[0], J0[1], J0[2],0,0,0];
            J0 = vector_normalize(J0);
            JT.push(J0);
        }
        else if (type === 'continuous' || type === 'revolute') {
            v1 = matrix_multiply_vector(robot.joints[joint].xform,axis);
            v2 = matrix_multiply_vector(robot.joints[joint].xform,[0,0,0,1]);
            v3 = endeffector_world;
            J1 = vector_sub(v1,v2);
            J1 = vector_normalize(J1);
            J2 = vector_sub(v3,v2);
            J0 = vector_cross(J1,J2);   
            J0 = [J0[0], J0[1], J0[2], J1[0], J1[1], J1[2] ];   
            JT.push(J0);
        }
        else {
            
        }
    
        link = robot.joints[joint].parent;
        joint = robot.links[link].parent;
    }
    
    angley = Math.asin(robot.joints[endeffector_joint].xform[0][2]);
    anglez = Math.asin(robot.joints[endeffector_joint].xform[0][1]/-1/Math.cos(angley));
    anglex = Math.asin(robot.joints[endeffector_joint].xform[1][2]/-1/Math.cos(angley));
    if (kineval.params.ik_orientation_included) {
        delta_x = [ endeffector_target_world.position[0] - endeffector_world[0] ,
                endeffector_target_world.position[1] - endeffector_world[1] ,   
                endeffector_target_world.position[2] - endeffector_world[2] ,   
                endeffector_target_world.orientation[0] - anglex ,
                endeffector_target_world.orientation[1] - angley ,   
                endeffector_target_world.orientation[2] - anglez   ];
    }
    else {
        delta_x = [ endeffector_target_world.position[0] - endeffector_world[0] ,
                endeffector_target_world.position[1] - endeffector_world[1] ,   
                endeffector_target_world.position[2] - endeffector_world[2] ,   
                0 ,
                0 ,   
                0 ];
    }
    if (kineval.params.ik_pseudoinverse) {
        delta_theta = matrix_multiply_vector(matrix_pseudoinverse(matrix_transpose(JT)), delta_x);
    }
    else {
        delta_theta = matrix_multiply_vector(JT , delta_x);
    }

    for (var i=0;i<delta_theta.length;i++){
        delta_theta[i] = kineval.params.ik_steplength * delta_theta[i];
    }
    // console.log(delta_theta);
    //     console.log(2);
    if (robot.joints[endeffector_joint].type !== "fixed" ){
        robot.joints[endeffector_joint].control = delta_theta[0];
        var j=1;
    }
    else {
        var j=0;
    }
    link = robot.joints[endeffector_joint].parent;
    joint = robot.links[link].parent;
    // console.log(joint);
    //         console.log(j);
    //         console.log("1");
    while (typeof robot.joints[joint] !== "undefined") {
        //joint = robot.links[link].parent;
        if (type === 'prismatic') {

            robot.joints[joint].control = delta_theta[j];
            // console.log(joint);
            // console.log(j);
            // console.log("2");
            j=j+1;
        //     console.log(delta_theta);
        // console.log(3);
        }
        else if (type === 'continuous' || type === 'revolute') {
            
            robot.joints[joint].control = delta_theta[j];
            // console.log(joint);
            // console.log(j);
            // console.log("3");
            j=j+1;
        }
        else {
            
        }
        link = robot.joints[joint].parent;
        joint = robot.links[link].parent;
        // console.log(joint);
        // console.log(j);
        // console.log("4");
    }

    // STENCIL: implement inverse kinematics iteration
    // Your code for a single IK iteration
    // output sets updated controls for each joint
}

