import {transpose, identity_matrix, mul_matrix_matrix, mul_matrix_scalar, add_matrix_matrix} from "../js/utils/utils_math.js";

export function get_SE3_from_SO3_and_t(so3, t) {
    let se3 = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]]
    
    for (let i = 0; i < 3; i++){
        for (let j = 0; j < 3; j++){
            se3[i][j] = so3[i][j];
        }
    }
    se3[0][3] = t[0][0];
    se3[1][3] = t[1][0];
    se3[2][3] = t[2][0];
    return se3;
}

export function get_SO3_and_t_from_SE3(se3) {
    let so3 = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    
    for (let i = 0; i < 3; i++){
        for (let j = 0; j < 3; j++){
            so3[i][j] = se3[i][j];
        }
    }

    let t = [[se3[0][3]], [se3[1][3]], [se3[2][3]]];

    return [so3, t];
}

function get_so3_from_u(u){
    let so3 = [[0, -u[2][0], u[1][0]],
               [u[2][0], 0, -u[0][0]],
               [-u[1][0], u[0][0], 0]];
    return so3;
}

function get_se3_from_so3_and_t(so3, t){
    get_SE3_from_SO3_and_t(so3, t);
}

function get_little_so3_at_pi(SO3){
    let pi_so3_matrix = [[0.0, -Math.PI*Math.sqrt(0.5 * (SO3[2][2] + 1)), Math.PI*Math.sqrt(0.5 * (SO3[1][1] + 1))],
                [Math.PI*Math.sqrt(0.5 * (SO3[2][2] + 1)), 0.0, -Math.PI*Math.sqrt(0.5 * (SO3[0][0] + 1))],
                [-Math.PI*Math.sqrt(0.5 * (SO3[1][1] + 1))], Math.PI*Math.sqrt(0.5 * (SO3[0][0] + 1)), 0.0];
    return pi_so3_matrix;
}
function so3_log_map(SO3){
    let tr = SO3[0][0] + SO3[1][1] + SO3[2][2];
    let beta = Math.acos((tr - 1) / 2.0);

    if (Math.abs(beta) < 0.0001) {
        let r_minus_rT = add_matrix_matrix(SO3, mul_matrix_scalar(transpose(SO3), -1));
        let beta_sum = 0.5 + Math.pow(beta, 2)/12.0 + Math.pow(beta, 4)*7/720.0;
        return mul_matrix_scalar(r_minus_rT, beta_sum);
    }
    else if (beta == Math.PI) {
        return get_little_so3_at_pi(SO3);
    }
    else {
        let r_minus_rT = add_matrix_matrix(SO3, mul_matrix_scalar(transpose(SO3), -1));
        let beta_func = beta / (2.0 * Math.sin(beta));
        return mul_matrix_scalar(r_minus_rT, beta_func);
    }
}

function get_alphas_from_little_so3(so3) {
    let a1 = so3[2][1];
    let a2 = so3[0][2];
    let a3 = so3[1][0];
    return [a1, a2, a3];
}

function get_little_t(so3, beta, t) {
    let p = 0.0;
    let q = 0.0;
    if (Math.abs(beta) < 0.0001) {
        p = (1/6.0) - (Math.pow(beta, 2)/120.0) + (Math.pow(beta, 4)/ 5040.0);
        q = 0.5 - (Math.pow(beta, 2)/24.0) + (Math.pow(beta, 4)/720.0);
    } else {
        p = (beta - Math.sin(beta)) / Math.pow(beta, 3);
        q = (1 - Math.cos(beta)) / Math.pow(beta, 2);
    }

    let I = identity_matrix(3);
    let p_so3_squared = mul_matrix_scalar(mul_matrix_matrix(so3, so3), p);
    let q_so3 = mul_matrix_scalar(so3, q);
    let sum_matrix = add_matrix_matrix(add_matrix_matrix(I, p_so3_squared), q_so3);

    let sum_matrix_inv = numeric.inv(sum_matrix);
    let little_t = mul_matrix_matrix(sum_matrix_inv, t);

    console.log(t);
    return [little_t[0][0], little_t[1][0], little_t[2][0]];
}

function dis(so3_1, t1, so3_2, t2){

    let se3_1 = get_SE3_from_SO3_and_t(so3_1, t1);
    let se3_2 = get_SE3_from_SO3_and_t(so3_2, t2);

    let disp = mul_matrix_matrix(numeric.inv(se3_1), se3_2);
    let [disp_so3, disp_t] = get_SO3_and_t_from_SE3(disp);

    let little_so3 = so3_log_map(disp_so3);
    
    let [a1, a2, a3] = get_alphas_from_little_so3(little_so3);
    let beta = Math.sqrt(Math.pow(a1, 2) + Math.pow(a2, 2) + Math.pow(a3, 2));

    let [a4, a5, a6] = get_little_t(little_so3, beta, disp_t);
    let dis = Math.sqrt(a1*a1 + a2*a2 + a3*a3 + a4*a4 + a5*a5 + a6*a6);
    return dis;
}

export function se3_log_map(se3){
    let [disp_so3, disp_t] = get_SO3_and_t_from_SE3(se3);
    let little_so3 = so3_log_map(disp_so3);
    let [a1, a2, a3] = get_alphas_from_little_so3(little_so3);
    let beta = Math.sqrt(Math.pow(a1, 2) + Math.pow(a2, 2) + Math.pow(a3, 2));
    let little_t = get_little_t(little_so3, beta, disp_t);
    return [[a1, a2, a3], little_t];
}

function so3_exponential_map(so3, beta){
    let p = 0.0;
    let q = 0.0;
    if (Math.abs(beta) < 0.0001) {
        p = 1 - (Math.pow(beta, 2)/6.0) + (Math.pow(beta, 4)/120.0);
        q = 0.5 - (Math.pow(beta, 2)/24.0) + (Math.pow(beta, 4)/ 720.0);
    } else {
        p = (Math.sin(beta)) / beta;
        q = (1 - Math.cos(beta)) / Math.pow(beta, 2);
    }

    let term_1 = mul_matrix_scalar(so3, p);
    let term_2 = mul_matrix_scalar(mul_matrix_matrix(so3, so3), q);      
    let SO3 = add_matrix_matrix(add_matrix_matrix(identity_matrix(3), term_1), term_2);

    return SO3;
}

export function se3_exponential_map(u, t_alphas){
    let beta = Math.sqrt(Math.pow(u[0][0], 2) + Math.pow(u[1][0], 2) + Math.pow(u[2][0], 2));

    let so3 = get_so3_from_u(u);
    let SO3 = so3_exponential_map(so3, beta);

    let p = 0.0;
    let q = 0.0;
    if (Math.abs(beta) < 0.0001) {
        p = 0.5 - (Math.pow(beta, 2)/24.0) + (Math.pow(beta, 4)/720.0);
        q = (1/6.0) - (Math.pow(beta, 2)/120.0) + (Math.pow(beta, 4)/ 5040.0);
    } else {
        p = (1 - Math.cos(beta)) / Math.pow(beta, 2);
        q = (beta - Math.sin(beta)) / Math.pow(beta, 3);
    }

    let term_1 = mul_matrix_scalar(so3, p);
    let term_2 = mul_matrix_scalar(mul_matrix_matrix(so3, so3), q);
    let C = add_matrix_matrix(add_matrix_matrix(identity_matrix(3), term_1), term_2);

    let t = mul_matrix_matrix(C, t_alphas);

    let SE3 = get_SE3_from_SO3_and_t(SO3, t);
    
    return SE3;
}

