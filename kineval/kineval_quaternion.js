//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply
function quaternion_from_axisangle(theta, axis) {
   
    var q=[];
    q[0]=Math.cos(theta/2);
    q[1]=axis[0]*Math.sin(theta/2);
    q[2]=axis[1]*Math.sin(theta/2);
    q[3]=axis[2]*Math.sin(theta/2);
    
    return q;
}

function quaternion_normalize(p1) {
   

    var p = [];
    var sum = 0;
    var i;

    for (i=0;i<p1.length;i++) { 
        sum+=p1[i]*p1[i];
    }

    for (i=0;i<p1.length;i++) {
        p[i] = p1[i]/Math.sqrt(sum);
    }
    return p;
}

function quaternion_to_rotation_matrix(q) {
   

    var mat = [];
    q0=q[0];
    q1=q[1];
    q2=q[2];
    q3=q[3];
    mat=[[1-2*(q2*q2+q3*q3), 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3), 0],
        [2*(q1*q2+q0*q3), 1-2*(q1*q1+q3*q3), 2*(q2*q3-q0*q1), 0],
        [2*(q1*q3-q0*q2), 2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2), 0],
        [0, 0, 0, 1]];
    
    return mat;
}

function quaternion_multiply(q1,q2) {
    var q=[];
    a=q1[0];
    b=q1[1];
    c=q1[2];
    d=q1[3];
    e=q2[0];
    f=q2[1];
    g=q2[2];
    h=q2[3];
    q=[a*e-b*f-c*g-d*h,
        a*f+b*e+c*h-d*g,
        a*g-b*h+c*e+d*f,
        a*h+b*g-c*f+d*e];

    return q;
}