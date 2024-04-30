import { mul_matrix_matrix, mul_matrix_scalar} from "../js/utils/utils_math.js";
import { se3_log_map, se3_exponential_map, get_SE3_from_SO3_and_t} from "./se3_log_and_exponential.js";
import { forward_kinematics } from "./kinematics.js";

export function interpolate_implicit(SO3_1, t_1, SO3_2, t_2, t){
    let SE3_1 = get_SE3_from_SO3_and_t(SO3_1, t_1);
    let SE3_2 = get_SE3_from_SO3_and_t(SO3_2, t_2);
    return interpolate(SE3_1, SE3_2, t);
}

export function interpolate(SE3_1, SE3_2, t){
    let disp = mul_matrix_matrix(numeric.inv(SE3_1), SE3_2);

    let [u, t_alphas] = se3_log_map(disp);

    let u_inter = mul_matrix_scalar(u, t);
    let t_alphas_inter = mul_matrix_scalar(t_alphas, t);

    let disp_int = se3_exponential_map(u_inter, t_alphas_inter);

    let interpolation = mul_matrix_matrix(SE3_1, disp_int);
    return interpolation;
}

// export function interpolate_between_states(robot, state_1, state_2, t){
//     let state_1_poses = forward_kinematics(robot, state_1);
//     let state_2_poses = forward_kinematics(robot, state_2);
//     let inter_poses = []
//     for (let i = 0; i < state_1_poses.length; i++){
//         inter_poses.push(interpolate(state_1_poses[i], state_2_poses[i], t));
//     }
//     return inter_poses;
// }

export function interpolate_between_states(robot, state_1, state_2, t){
    //let state_1_poses = forward_kinematics(robot, state_1);
    //let state_2_poses = forward_kinematics(robot, state_2);
    let inter_states = []
    for (let i = 0; i < state_1.length; i++){
        inter_states.push((state_2[i] - state_1[i]) * t + state_1[i]);
    }
    let inter_poses = forward_kinematics(robot, inter_states);
    return inter_poses;
}