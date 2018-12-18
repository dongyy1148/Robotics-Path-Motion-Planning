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

    function LU(A) {  
        var n = A.length; 
        var L = generate_identity(n); 
        var P = [];
        for (var i = 0; i<n; i++){
            P[i] = i;
        }
        var U = generate_identity(n);
        for (var i = 0; i < n; i++){
            var max = Math.abs(U[i][i]);
            var row = i;
            //  Find the maximun value in a column in order to change lines
            for (var k = i+1; k<n; k++){
                if (Math.abs(U[k][i]) > max){
                    max = Math.abs(U[k][i]);
                    row = k;
                }
            }
            if (max == 0){
                return "false";
            }

            var tmp = P[i];
            P[i] = P[row];
            P[row]=tmp;

            // Swap the rows pivoting the maxRow, i is the current row
            for (var k = i; k<n; k++){
                var temp = A[row][k];
                A[row][k] = A[i][k];
                A[i][k] = temp;
            }
            var u =A[i][i];
            // Subtract lines
            for (var j = i+1; j<n; j++){
                var l = A[j][i]/u;
                A[j][i] = l;
                for (var k = i+1; k<n; k++){
                    A[j][k] = A[j][k] - A[i][k]*l;
                }
            }
            // Make the rows bellow this one zero in the current column
            for (var k = i+1; k<n; k++){
                U[k][i] = 0;
            }

        }


        for (var i=0 ; i<n ;i++){
            for (var j=0; j<i+1; j++){
                if (i !== j){
                    L[i][j]=A[i][j];
                }
                else{
                    L[i][j]=1;
                }
            }
            for (var k=i; k<n; k++) {
                U[i][k]=A[i][k];
            }
105     }

        return [L, U, P];
    }

    function matrix_inverse(A){
        var A2 = [];
        var n = A.length;
        var I = generate_identity(n);
        var A_inv = generate_identity(n);
        for (var i=0; i<n; i++){
            A2 = matrix_copy(A);
            var b = [];
            for (var j = 0; j<n; j++){
                b[j] = I[j][i];                
            }
            var c = linear_solve(A2,b);
            for (var m = 0; m<n; m++){
                A_inv[m][i] = c[m];
            }
        }
        return A_inv;
    }

    function linear_solve(A,b){
        var y =[];
        var A2 = matrix_copy(A);
        var result = LU(A2);
        var L = result[0];
        var U = result[1];
        var P = result[2];
        var n = L.length;
        for (var i = 0; i < n; i++)  {
            y[i] = b[P[i]];
            for(var j = 0; j < i; j++){
                y[i] = y[i] - L[i][j]*y[j];
            }
        }
        var x =[];
        for (var i = n-1; i > -1; i--)        {
            x[i] = y[i];
            for (var j = n-1; j > i; j--){
                x[i] = x[i] - U[i][j]*x[j];
            }
            x[i] = x[i] / U[i][i];
        }
        return x;
    }

    

    // A = [[1,2,3,4],[10,15,7,9],[5,16,12,11],[8,14,6,13]];
    // b = [5,6,7,8];
    // c = linear_solve(A,b);
    