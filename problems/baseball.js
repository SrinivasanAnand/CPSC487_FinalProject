import * as THREE from 'three';
import {add_matrix_matrix, mul_matrix_scalar} from "../js/utils/utils_math.js";

export class baseball {
    constructor(engine, position){
        let sphere_idx = engine.add_sphere_as_mesh_object([0.00, 0.00, 0.00], 0.03, 10, 0xf6f0f0, 1.0, true);
        let torus_idx = engine.add_torus_specific_as_mesh_object(0.020, 0.0055, 50, 100, 6.28318); //0.006
        let torus_idx2 = engine.add_torus_specific_as_mesh_object(0.020, 0.0055, 50, 100, 6.28318); //0.006

        this.sphere = engine.mesh_objects[sphere_idx];
        this.torus1 = engine.mesh_objects[torus_idx];
        this.torus2 = engine.mesh_objects[torus_idx2];

        this.components = [this.sphere, this.torus1, this.torus2];

        this.initialize_position(position);
    }

    initialize_position(position){
        let tc_1 = new THREE.Matrix4();
        let tc_2 = new THREE.Matrix4();
        let tc_3 = new THREE.Matrix4();
        tc_1.set( 1, 0, 0, position[0][0],
            0, 1, 0, position[1][0],
            0, 0, 1, position[2][0], //0.00
            0, 0, 0, 1);
        tc_2.set( 1, 0, 0, position[0][0],
            0, 1, 0, position[1][0],
            0, 0, 1, position[2][0] + 0.015, //0.00
            0, 0, 0, 1);
        tc_3.set( 1, 0, 0, position[0][0],
            0, 1, 0, position[1][0],
            0, 0, 1, position[2][0] - 0.015, //0.00
            0, 0, 0, 1);
        
        this.sphere.applyMatrix4(tc_1);
        this.torus1.applyMatrix4(tc_2);
        this.torus2.applyMatrix4(tc_3);
    }
    set_pose(SE3){
        let t = new THREE.Matrix4();
        t.set(SE3[0][0], SE3[0][1], SE3[0][2], SE3[0][3],
            SE3[1][0], SE3[1][1], SE3[1][2], SE3[1][3],
            SE3[2][0], SE3[2][1], SE3[2][2], SE3[2][3],
            SE3[3][0], SE3[3][1], SE3[3][2], SE3[3][3]);
        
        this.components.forEach(component => {
            component.applyMatrix4(t);
        })
    }
    set_pos(position){
        this.components.forEach(component => {
            component.position.x = position[0][0];
            component.position.y = position[1][0];
            component.position.z = position[2][0];
        });
    }
    translate(velocity){
        this.components.forEach(component => {
            component.position.x += velocity[0][0];
            component.position.y += velocity[1][0];
            component.position.z += velocity[2][0];
        });
    }
    rotate(axis, angle){
        let v = new THREE.Vector3(axis[0][0], axis[1][0], axis[2][0]);

        this.sphere.rotateOnAxis(v, angle);

        this.torus1.translateZ(-0.015);
        this.torus1.rotateOnAxis(v, angle);
        this.torus1.translateZ(0.015);

        this.torus2.translateZ(+0.015);
        this.torus2.rotateOnAxis(v, angle);
        this.torus2.translateZ(-0.015);
    }
}

// weird representation mismatch fix
export function three_representation(vec){
    let three = [[vec[0][0]], [vec[2][0]], [-vec[1][0]]];
    return three;
}
export class projectile {
    constructor(engine, position, radius){
        this.engine = engine;
        this.init_pos = position;
        this.pos = position;
        this.radius = radius;
        this.vel = [[0.000], [0.000], [0.000]];
        this.accel = [[0.000], [0.000], [0.000]];
        this.spin_theta = 0.0;
        this.spin_accel = 0.0;
        this.caught = false;
        this.passed_strike = false;
        this.strike_x_coord = 1;

        let three_pos = three_representation(this.pos);
        this.ball = new baseball(engine, three_pos);
    }

    draw_projectile() {
        let three_vel = three_representation(this.vel);
        // let SE3 = [[1, 0, 0, three_vel[0][0]],
        //            [0, 1, 0, three_vel[1][0]],
        //            [0, 1, 1, three_vel[2][0]],
        //            [0, 0, 0, 1]];
        // console.log(SE3);
        // this.ball.set_pose(SE3);
        this.ball.translate(three_vel);
    }

    get_pos(){
        return this.pos;
    }

    set_pos(pos){
        let vel = add_matrix_matrix(pos, mul_matrix_scalar(this.pos, -1))
        this.pos = pos;
        let three_vel = three_representation(vel);
        this.ball.translate(three_vel);
    }

    set_velocity(vel) {
        this.vel = vel;
    }

    set_acceleration(accel){
        this.accel = accel;
    }

    set_spin_theta(theta){
        this.spin_theta = theta;
    }

    next_pos() {
        this.pos = add_matrix_matrix(this.pos, this.vel);
    }

    next_vel() {
        this.vel = add_matrix_matrix(this.vel, this.accel);
    }

    bounce() {
        let rebound = 0.5;
        if (this.pos[2][0] - this.radius <= 0.00){
            if (this.vel[2][0] < 0) {
                this.vel[2][0] = -this.vel[2][0] * rebound;
            }
        }
    }

    spin() {
        let axis = [[1 / (Math.sqrt(3))], [1 / (Math.sqrt(3))], [1 / (Math.sqrt(3))]];
        let three_axis = three_representation(axis);
        this.ball.rotate(three_axis, this.spin_theta);
    }

    next_spin() {
        this.spin_theta = this.spin_theta + this.spin_accel;
        if (this.spin_theta <= 0) {
            this.spin_theta = 0;
        }
    }

    stop_if_necessary() {
        if (this.pos[0][0] < 0.5 && !this.caught){
            this.vel = [[0.0], [0.0], [0.0]];
            this.accel = [[0.0], [0.0], [0.0]];
            this.spin_theta = 0.0;
            console.log(this.pos);
            this.caught = true;
        }
        if (this.pos[0][0] > this.init_pos[0][0]){
            this.vel = [[0.0], [0.0], [0.0]];
            this.accel = [[0.0], [0.0], [0.0]];
            this.caught = false;
            this.passed_strike = false;
            this.set_pos(this.init_pos);
            this.spin_accel = -0.005;
        }
    }

    draw_strike() {
        if (this.pos[0][0] < this.strike_x_coord && !this.passed_strike){
            console.log("Strike");
            console.log(this.pos);
            this.engine.draw_debug_sphere([this.pos[0][0], this.pos[1][0], this.pos[2][0]], this.radius, 0xff00ff, 1.0, 10);
            //this.engine.draw_debug_plane([this.pos[0][0]+0.1, this.pos[1][0], this.pos[2][0]], [[0], [1], [0]], [[0], [0], [1]], 0.5, 0.5, 0xC8AE7D, 1);
            console.log([this.pos[0][0], this.pos[1][0], this.pos[2][0]]);
            this.passed_strike = true;
        }
    }

    time_step() {
        this.draw_projectile();
        this.spin();
        this.next_spin();
        this.next_pos();
        this.next_vel();
        this.bounce();
        this.stop_if_necessary();
        this.draw_strike();
    }

}

export function calculate_collision_steps(init_pos, init_vel, plane_x_coord){
    let t = ( init_pos[0][0] - plane_x_coord ) / Math.abs(init_vel[0][0]);
    return t;
}

export function calculate_collision(init_pos, init_vel, init_accel, plane_x_coord){

    // t = d / r (no x-acceleration)
    let t = ( init_pos[0][0] - plane_x_coord ) / Math.abs(init_vel[0][0]);

    // want to find y- and z- positions at time t
    let y_f = init_vel[1][0] * t + 0.5 * init_accel[1][0] * Math.pow(t, 2) + init_pos[1][0];
    let z_f = init_vel[2][0] * t + 0.5 * init_accel[2][0] * Math.pow(t, 2) + init_pos[2][0];

    return [[plane_x_coord], [y_f], [z_f]];
}

export function calculate_launch_speed(launch_pos, gravity, mound_position){

    // t = d / r (no x-acceleration)
    let t = Math.sqrt( 2 * (mound_position[2][0] - launch_pos[2][0]) * gravity);
    let vx_i = (mound_position[0][0] - launch_pos[0][0]) / t;

    return [[0.08], [0.0], [0.00]];
}

export function setup_scene(engine){
    // engine.draw_debug_plane([2, 2, -0.02], [[1], [1], [0]], [[-1], [1], [0]], 2.5, 0.5, 0xC8AE7D, 1);
    // engine.draw_debug_plane([2, -2, -0.021], [[1], [-1], [0]], [[-1], [-1], [0]], 2.5, 0.5, 0xC8AE7D, 1);
    // engine.draw_debug_plane([5.1, -3.1, -0.023], [[1], [1], [0]], [[-1], [1], [0]], 2.5, 0.5, 0xC8AE7D, 1);
    //engine.draw_debug_plane([2, -2, -0.021], [[1], [-1], [0]], [[-1], [-1], [0]], 2.5, 0.5, 0xC8AE7D, 1);
    engine.draw_debug_plane([1.0, 0.0, 0.5], [[0], [0], [1]], [[0], [1], [0]], 0.1, 0.1, 0x00ff00, 0.3);

    engine.draw_debug_plane([4, 0, -0.023], [[1], [1], [0]], [[-1], [1], [0]], 3.5, 3.5, 0xC8AE7D, 1);
    engine.draw_debug_plane([4, 0, -0.022], [[1], [1], [0]], [[-1], [1], [0]], 2.5, 2.5, 0x90EE90, 1);

    engine.draw_debug_plane([4, -4.2, -0.021], [[1], [1], [0]], [[-1], [1], [0]], 0.2, 0.2, 0xF9F6EE, 1);
    engine.draw_debug_plane([4, 4.2, -0.021], [[1], [1], [0]], [[-1], [1], [0]], 0.2, 0.2, 0xF9F6EE, 1);
    engine.draw_debug_plane([8.2, 0, -0.021], [[1], [1], [0]], [[-1], [1], [0]], 0.2, 0.2, 0xFFFFFF, 1);
    //engine.draw_debug_plane([3.8, 0, 0.021], [[0], [1], [0]], [[1], [0], [0]], 0.15, 0.05, 0xFFFFFF, 1);

    const geometry = new THREE.CircleGeometry( 1, 32 ); 
    const material = new THREE.MeshBasicMaterial( { color: 0x836539 } ); 
    const circle = new THREE.Mesh( geometry, material );
    circle.rotateX(- Math.PI / 2);
    engine.scene.add( circle );

    const geometry2 = new THREE.CircleGeometry( 1.5, 32 ); 
    const material2 = new THREE.MeshBasicMaterial( { color: 0x0A6847 } ); 
    const circle2 = new THREE.Mesh( geometry2, material2 );
    circle2.rotateX(- Math.PI / 2);
    circle2.translateZ(-0.01);
    engine.scene.add( circle2 );

    const geometry3 = new THREE.CircleGeometry( 0.5, 32 ); 
    const material3 = new THREE.MeshBasicMaterial( { color: 0x836539 } ); 
    const mound = new THREE.Mesh( geometry3, material3 );
    mound.rotateX(- Math.PI / 2);
    mound.translateX(4);
    engine.scene.add( mound );
}
