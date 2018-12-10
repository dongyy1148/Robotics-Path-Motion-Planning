
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);
    eps = 1;
    q_new = q_start_config;
}



function robot_rrt_planner_iterate() {
    
    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();
        rrt_iter_count += 1;
    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
    if (rrt_alg ===1){
    q_rand = random_config();
    var index = nearest_neighbor(T_a,q_rand);
    q_near = T_a.vertices[index].vertex;
    q_newpre = new_config(q_near,q_rand,eps);

    if (!kineval.poseIsCollision(q_newpre)){
        q_new = q_newpre;
        rrt_extend(T_a,q_new,index);
        var index2 = nearest_neighbor(T_b,q_new);
        var q_near2 = T_b.vertices[index2].vertex;
        var q_newpre2 = new_config(q_near2,q_new,eps);
        //var q_old = q_near2;
        while (!kineval.poseIsCollision(q_newpre2)){
            rrt_extend(T_b,q_newpre2,index2);
            index2 = nearest_neighbor(T_b,q_new);
            q_near2 = T_b.vertices[index2].vertex;
            q_newpre2 = new_config(q_near2,q_new,eps);
            var flag2 = true;
            for (var n = 0; n<q_newpre2.length;n++){
                if (Math.abs(q_new[n]-q_newpre2[n])>eps){
                    flag2 = false;
                }
            }
            if (flag2&&!kineval.poseIsCollision(q_newpre2)){
                rrt_extend(T_b,q_newpre2,index2);
                var path1 = path_dfs1(T_b,T_b.newest);
                find_path(path1);
            
                var path2 = path_dfs2(T_a,T_a.newest);
                find_path(path2);
                if ((T_a.vertices[0].vertex[0]-q_goal_config[0])<eps){
                    path2 = path2.reverse();
                    kineval.motion_plan = path2.concat(path1).reverse();
                }
                else {
                    path1 = path1.reverse();
                    kineval.motion_plan = path1.concat(path2).reverse();
                }
                // drawHighlightedPath(path); 
                // tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
                // for (l in path) {
                //     kineval.motion_plan.push(path[path.length-l-1]);
                // }
                
                // drawHighlightedPath(path);       
                rrt_iterate=false;
                return "reached"
            }
        }
        T_c = T_a;
        T_a = T_b;
        T_b = T_c;
        }
    }

    if (rrt_alg ===2){
        var flag3 = true;
        for (var n = 0; n<q_goal_config.length;n++){
            if (Math.abs(q_new[n]-q_goal_config[n])>eps){
                flag3 = false;
            }
        }
        if (flag3){

            var path = path_dfs2(T_a,T_a.newest);
            find_path(path);
            kineval.motion_plan = path.reverse();
            // path = path_dfs2(T_a,T_a.newest);
            // find_path(path);
            // kineval.motion_plan = path.reverse();
            // drawHighlightedPath(path);    
            rrt_iterate=false;
            return "reached"
        }
      
        else{  
            q_rand = random_config();
            var index = nearest_neighbor(T_a,q_rand);
            q_near = T_a.vertices[index];
            q_newpre = new_config(q_near.vertex,q_rand,eps);
    
            if (!kineval.poseIsCollision(q_newpre)){
                var indexsum = findnear(T_a, q_newpre, 1);
                var parent = chooseparent(q_near, indexsum, q_newpre);
                q_new = q_newpre;
                var index_parent = T_a.vertices.indexOf(parent);
                rrt_extend(T_a,q_new,index_parent);
                var distance = 0;
                for (var p=0;p<parent.vertex.length;p++){
                    distance = distance + Math.pow((parent.vertex[p]-q_new[p]),2); 
                }
                distance = Math.sqrt(distance);
                distance = distance + parent.cost;
                T_a.vertices[T_a.newest].cost = distance;

                T_a = rewire(T_a,indexsum,parent,T_a.vertices[T_a.newest]);
            }
        }  

        }

    }

    
}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
    tree.vertices[0].cost = 0;
    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}

function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    function rewire(tree,index,q_min,z_new){
        indexout = tree.vertices.indexOf(q_min);
        for (var i=0;i<index.length;i++) {
            var m = index[i];
            var q_pre2 = tree.vertices[m];
            if (m !== indexout){
                if (!kineval.poseIsCollision(q_pre2.vertex)){
                    var distance2 = 0;
                    for (var p=0;p<q_pre2.vertex.length;p++){
                        distance2 = distance2 + Math.pow((q_pre2.vertex[p]-z_new.vertex[p]),2); 
                    }
                    distance2 = Math.sqrt(distance2);
                    distance2 = distance2 + z_new.cost;

                    if (distance2 < q_pre2.cost) {
                        q_pre2.cost = distance2;
                        q_pre2.edges[0] = z_new;
                    }
                } 
            }
        }
        return tree;
    }

    function chooseparent(q_near, index ,q_newpre){
        var z_min = q_near;
        var j = 0;
        var c_min = 0;
        for (var m=0;m<z_min.vertex.length;m++){
            c_min = c_min + Math.pow((z_min.vertex[m]-q_newpre[m]),2); 
        }
        c_min = Math.sqrt(c_min);
        c_min = c_min + z_min.cost;
            
        for (var i=0;i<index.length;i++) {
            var n = index[i];
            var q_pre = T_a.vertices[n];
            if (!kineval.poseIsCollision(q_pre.vertex)){
                var distance = 0;
                for (var p=0;p<q_pre.vertex.length;p++){
                    distance = distance + Math.pow((q_pre.vertex[p]-q_newpre[p]),2); 
                }
                distance = Math.sqrt(distance);
                distance = distance + q_pre.cost;
                if (distance < c_min) {
                    c_min = distance;
                    z_min = q_pre;
                }
            }      
        }
        return z_min;
            
    }

    function findnear(tree,q,radius){
        var index =[];
        for (var i=0;i<tree.vertices.length;i++) {
            var dis = 0;
            for (var m=0;m<q.length;m++){
                dis = dis + Math.pow((tree.vertices[i].vertex[m]-q[m]),2); 
            }
            dis = Math.sqrt(dis);
            if (dis < radius) {
                index.push(i);
            }      
        }
        return index;
    }
    //   rrt_extend
    function rrt_extend(tree,q_new,index){
        tree_add_vertex(tree,q_new);
        tree_add_edge(tree,index,tree.newest);
    }
    //   rrt_connect
    
    //   random_config
    function random_config(){
        var q_rand = [];
        var ran = Math.random();
        if (ran > 0.9){
            q_rand = q_goal_config;
        }
        else{
        q_rand[0] = (robot_boundary[1][0] - robot_boundary[0][0])*Math.random() + robot_boundary[0][0];
        q_rand[1] = q_start_config[1];
        q_rand[2] = (robot_boundary[1][2] - robot_boundary[0][2])*Math.random() + robot_boundary[0][2];
        q_rand[3] = q_start_config[3];
        q_rand[4] = 2*Math.PI*Math.random() - Math.PI;
        q_rand[5] = q_start_config[5];
        if(robot.links_geom_imported) {
            for (x in robot.joints){
                if ((robot.joints[x].type === 'prismatic')
                || (robot.joints[x].type === 'revolute')){
                    var low = robot.joints[x].limit.lower;
                    var high = robot.joints[x].limit.upper;
                    q_rand = q_rand.concat((high-low)*Math.random()+low);
                    
                }
                else if (robot.joints[x].type === 'fixed'){
                    q_rand = q_rand.concat(0);
                }
                else {
                    q_rand = q_rand.concat(2*Math.PI*Math.random()-Math.PI);
                }
            }
        }
        else {
            for (x in robot.joints){
                
                q_rand = q_rand.concat(2*Math.PI*Math.random()-Math.PI);
            }
        }
        }
        // if ( (curRobot.joints[x].type === 'prismatic')
        //      || (curRobot.joints[x].type === 'revolute') ) { 
        //     curRobot.joints[x].angle += curRobot.joints[x].control;
        //     angletest = curRobot.joints[x].angle;
        //     if (angletest < curRobot.joints[x].limit.lower) {
        //         curRobot.joints[x].angle = curRobot.joints[x].limit.lower;
        //     }

        //     else if (angletest > curRobot.joints[x].limit.upper) {
        //         curRobot.joints[x].angle = curRobot.joints[x].limit.upper;
        //     }
            
        //     else {
        //         curRobot.joints[x].angle = curRobot.joints[x].angle;
        //     }
        // }

        return q_rand;
    }
    //   new_config
    function new_config(q_old,q_goal,eps) {
        var dis = 0;
        for (var m=0;m<q_goal.length;m++){
            dis = dis + Math.pow((q_old[m]-q_goal[m]),2); 
        }
        dis = Math.sqrt(dis);

        var q_newpre =[];
        if (dis > eps){
            for (var m=0;m<q_goal.length;m++){
                q_newpre[m] = q_old[m] + (q_goal[m]-q_old[m])/dis*eps; 
            }
        }
        else {
            q_newpre = q_goal;
        }
        
        return q_newpre
    }
    //   nearest_neighbor
    function nearest_neighbor(tree,goal){
        var min_dis = 1000;
        var j = 0;
        for (var i=0;i<tree.vertices.length;i++) {
            var distance = 0;
            for (var m=0;m<goal.length;m++){
                distance = distance + Math.pow((tree.vertices[i].vertex[m]-goal[m]),2); 
            }
            distance = Math.sqrt(distance);
            if (distance < min_dis) {
                min_dis = distance;
                j = i;
            }      
        }
        return j;
    }
    //   normalize_joint_state
    //   find_path
    //   path_dfs
    function path_dfs1(tree,index){
        var path = [];
        q_current = tree.vertices[index];
        path.push(q_current);
        // kineval.motion_plan.unshift(q_current);
        while (tree.vertices.indexOf(q_current) !== 0){
            q_current = q_current.edges[0];
            path.push(q_current);
            // kineval.motion_plan.unshift(q_current);
        }
        return path;
    }

    function path_dfs2(tree,index){
        var path = [];
        q_current = tree.vertices[index];
        path.push(q_current);
        // kineval.motion_plan.push(q_current);
        while (tree.vertices.indexOf(q_current) !== 0){
            q_current = q_current.edges[0];
            path.push(q_current);
            // kineval.motion_plan.push(q_current);
        }
        return path;
    }

    function find_path(path){
        for (x in path){
                path[x].geom.material.color = {r:1,g:0,b:0};
        }
    }










