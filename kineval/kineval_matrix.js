//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}

function matrix_multiply(m1,m2) {


    var mat = [];
    var i,j,m;
    if (m1[0].length === m2.length){
        for (i=0;i<m1.length;i++) { // for each row of m1
            mat[i] = [];
            for (j=0;j<m2[0].length;j++) { // for each column of m1
                mat[i][j]=0;
                for (m=0;m<m2.length;m++){
                    mat[i][j] += m1[i][m]*m2[m][j];
                }
            }
        }
    return mat;
    }

    else{
        return "false"
    }

}

function matrix_transpose(m1) {


    var mat = [];
    var i,j;

    for (i=0;i<m1[0].length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1.length;j++) { // for each column of m1
            mat[i][j] = m1[j][i];
        }
    }
    return mat;
}

function matrix_pseudoinverse(m1) {


    var mat = [];
    var i,j;
    var n=m1.length;
    var m=m1[0].length;
    if (n>m){
        m1_t=matrix_transpose(m1);
        m1_1=numeric.inv(matrix_multiply(m1_t,m1));
        m1_left=matrix_multiply(m1_1,m1_t);
        return m1_left;
    }

    else if (n===m) {
        return numeric.inv(m1);
    }

    else {
        m1_t=matrix_transpose(m1);
        m1_1=numeric.inv(matrix_multiply(m1,m1_t));
        m1_right=matrix_multiply(m1_t,m1_1);
        return m1_right;
    }

}

function matrix_invert_affine(t) {


    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[j][i] = m1[i][j];
        }
    }
    return mat;
}

function vector_normalize(v1) {


    var v = [];
    var sum = 0;
    var i;

    for (i=0;i<v1.length;i++) { // for each row of m1
        sum+=v1[i]*v1[i];
    }

    for (i=0;i<v1.length;i++) {
        v[i] = v1[i]/Math.sqrt(sum);
    }
    return v;
}

// function vector_cross(v1,v2) {
//
//     var v = [];
//     var mat = [];
//     var v3=[];
//     var i,j;
//
//     for (i=0;i<3;i++) {
//         mat[i] = [];
//         v3[i] = v2[i];
//         for (j=0;j<3;j++) {
//             mat[i][j] = 0;
//         }
//     }
//     mat[0][1] = -v1[2];
//     mat[0][2] = v1[1];
//     mat[1][0] = v1[2];
//     mat[1][2] = -v1[0];
//     mat[2][0] = -v1[1];
//     mat[2][1] = v1[0];
//     v = matrix_multiply_vector(mat,v3);
//     return v;
// }

function vector_cross(v1, v2){
    var v = [];
    v[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v[2] = v1[0]*v2[1] - v1[1]*v2[0];
    return v;
}

function generate_identity(x) {
    var mat = [];
    var i,j;
    for (i=0;i<x;i++) {
        mat[i]=[];
        for (j=0;j<x;j++){
            if (i===j){
                mat[i][j]=1;
            }
            else {
                mat[i][j]=0;
            }
        }
    }
    return mat;
}

function generate_translation_matrix(tx,ty,tz) {
    var mat = generate_identity(4);
    mat[0][3] = tx;
    mat[1][3] = ty;
    mat[2][3] = tz;
    return mat;
}

function generate_rotation_matrix_X(x) {
    var mat= [];
    mat=[[1,0,0,0],
        [0,Math.cos(x),-Math.sin(x),0],
        [0,Math.sin(x),Math.cos(x),0],
        [0,0,0,1]];
    return mat;
}

function generate_rotation_matrix_Y(x) {
    var mat= [];
    mat=[[Math.cos(x),0,Math.sin(x),0],
        [0,1,0,0],
        [-Math.sin(x),0,Math.cos(x),0],
        [0,0,0,1]];
    return mat;
}

function generate_rotation_matrix_Z(x) {
    var mat= [];
    mat=[[Math.cos(x),-Math.sin(x),0,0],
        [Math.sin(x),Math.cos(x),0,0],
        [0,0,1,0],
        [0,0,0,1]];
    return mat;
}
    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    //   matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z

    function matrix_multiply_vector(m1,m2) {
        var mat = [];
        var i,m;
        if (m1[0].length === m2.length){
            for (i=0;i<m1.length;i++) { // for each row of m1
                mat[i] = 0;
                for (m=0;m<m2.length;m++){
                    mat[i] += m1[i][m]*m2[m];
                }
            }
        return mat;
        }
        else{
            return "false"
        }
    }

    function vector_sub(v1,v2) {
        var v = [];
        var i;
        if (v1.length === v2.length){
            for (i=0;i<v1.length;i++) { // for each row of m1
                v[i] = v1[i] - v2[i];
            }
        return v;
        }
        else{
            return "false"
        }
    }
