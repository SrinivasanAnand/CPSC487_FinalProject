import {frobenius_norm_matrix, identity_matrix, mul_matrix_matrix, mul_matrix_scalar, add_matrix_matrix} from "../js/utils/utils_math.js";
import {se3_exponential_map, get_SO3_and_t_from_SE3} from "./se3_log_and_exponential.js";
import {optimization_solve} from "../js/utils/utils_optimization.js";

function identity_element() {
    return identity_matrix(4);
}
function group_operator(SE3_1, SE3_2) {
    return mul_matrix_matrix(SE3_1, SE3_2);
}

export function forward_kinematics(robot, state) {
    let output_poses = [];


    let joints = robot.joints;
    let links = robot.links;
    let kh = robot.kinematic_hierarchy;

    for(let i = 0; i < links.length; i++) {
        output_poses.push(identity_element());
    }

    kh.forEach(layer => {
        layer.forEach(link_idx => {
            let curr_link = links[link_idx];
            let parent_link_idx = curr_link.parent_link_idx;
            let parent_joint_idx = curr_link.parent_joint_idx;
            let curr_joint = joints[parent_joint_idx];

            if (parent_joint_idx === null) {
                return;
            }
            
            let joint_type = curr_joint.joint_type_string;
            let curr_pose = output_poses[parent_link_idx];
            let T_c = curr_joint.xyz_rpy_SE3_matrix;
            curr_pose = group_operator(curr_pose, T_c);
            
            // T_v will depend on some values in the state
            let T_v = identity_element();
            if (joint_type === 'revolute') {
                let joint_value = state[curr_joint.dof_idx];
                let axis = curr_joint.axis;
                let u = mul_matrix_scalar(axis, joint_value);
                T_v = se3_exponential_map(u, [[0], [0], [0]]);

            } else if (joint_type === 'prismatic') {
                let joint_value = state[curr_joint.dof_idx];
                let axis = curr_joint.axis;
                let u = mul_matrix_scalar(axis, joint_value);
                T_v = se3_exponential_map([[0], [0], [0]], u);

            }
        
            curr_pose = group_operator(curr_pose, T_v);
            
            output_poses[link_idx] = curr_pose;

        });



    });

    return output_poses;
}

export function inverse_kinematics(robot, target, n_dof){
    let f = state => {
        let out_poses = forward_kinematics(robot, state);
        let [grab_rotation, grab_position] = get_SO3_and_t_from_SE3(out_poses[19]);
        console.log(target);
        console.log(grab_position);
        let distance = add_matrix_matrix(grab_position, mul_matrix_scalar(target, -1));
        let square_norm = Math.pow(frobenius_norm_matrix(distance), 2);

        return square_norm;
    }

    let init_state = [];
    for (let i = 0; i < n_dof; i++){
        init_state.push(0.0);
    }
    let optimized_state = optimization_solve(f, init_state, undefined, 'bfgs');

    return optimized_state;
}