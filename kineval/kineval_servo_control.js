
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

    // STENCIL: implement FSM to cycle through dance pose setpoints
    var posepoint = kineval.params.dance_pose_index;
    var flag = false;
    for (x in robot.joints) {
        if (robot.joints[x].error < 0.0001){
            flag = true;
        }
        else {
            flag = false;
            break
        }
    }


    if (flag) {
        posepoint += 1;
        if (posepoint == kineval.params.dance_sequence_index.length) {
            posepoint = 0;
        }
        kineval.params.dance_pose_index = posepoint;
        kineval.setPoseSetpoint(kineval.params.dance_sequence_index[posepoint]);
    }
    // else {
        // kineval.setPoseSetpoint(posepoint);
    // }

}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
   
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    for (x in robot.joints) {
        robot.joints[x].servo.p_desired = kineval.params.setpoint_target[x];
        if(!robot.joints[x].previouserror){
            robot.joints[x].previouserror = robot.joints[x].error;
        }
        robot.joints[x].error = robot.joints[x].servo.p_desired - robot.joints[x].angle; 
        var derivative_error = - (robot.joints[x].error - robot.joints[x].previouserror)/1;
        robot.joints[x].control = robot.joints[x].servo.p_gain * robot.joints[x].error
            + robot.joints[x].servo.d_gain*derivative_error;
        robot.joints[x].previouserror = robot.joints[x].error;
    }

}


