use crate::{
    body::{Body, BodyArena, BodyHandle},
    broadphase::broadphase,
    constraints::ConstraintArena,
    contact::{Contact, ContactArena},
    intersect::intersect_dynamic,
    manifold::ManifoldCollector,
    scene_shapes::*,
};
use glam::{const_vec3, Quat, Vec3};

#[allow(dead_code)]
fn add_sphere(bodies: &mut BodyArena) {
    bodies.add(Body {
        position: Vec3::new(-10.0, 5.0, 0.0),
        inv_mass: 1.0,
        elasticity: 0.9,
        friction: 0.5,
        shape: make_sphere(1.0),
        ..Body::default()
    });
}

#[allow(dead_code)]
fn add_convex_hull(bodies: &mut BodyArena) {
    bodies.add(Body {
        position: Vec3::new(-10.0, 10.0, 0.0),
        inv_mass: 1.0,
        elasticity: 1.0,
        friction: 0.5,
        shape: make_diamond(),
        ..Body::default()
    });
}

#[allow(dead_code)]
fn add_teleportation_fix(bodies: &mut BodyArena) {
    // demonstrates continuous collision detection
    bodies.add(Body {
        position: Vec3::new(10.0, 3.0, -10.0),
        linear_velocity: Vec3::new(-100.0, 0.0, 0.0),
        inv_mass: 1.0,
        elasticity: 0.5,
        friction: 0.5,
        shape: make_sphere(0.5),
        ..Body::default()
    });

    bodies.add(Body {
        position: Vec3::new(-10.0, 3.0, -10.0),
        linear_velocity: Vec3::new(100.0, 0.0, 0.0),
        angular_velocity: Vec3::new(0.0, 0.0, 10.0),
        inv_mass: 1.0,
        elasticity: 0.5,
        friction: 0.5,
        shape: make_diamond(),
        ..Body::default()
    });
}

#[allow(dead_code)]
fn add_orientation_constraint(bodies: &mut BodyArena, constraints: &mut ConstraintArena) {
    let cube_shape = make_cube_small();

    let handle_a = bodies.add(Body {
        position: Vec3::new(5.0, 5.0, 0.0),
        inv_mass: 0.0,
        elasticity: 0.9,
        friction: 0.5,
        shape: cube_shape.clone(),
        ..Body::default()
    });

    let handle_b = bodies.add(Body {
        position: Vec3::new(6.0, 5.0, 0.0),
        inv_mass: 0.001,
        elasticity: 1.0,
        friction: 0.5,
        shape: cube_shape.clone(),
        ..Body::default()
    });

    constraints.add_orientation_constraint(bodies, handle_a, handle_b)
}

#[allow(dead_code)]
fn add_dynamic_balls(bodies: &mut BodyArena) {
    let ball_shape = make_sphere(0.5);
    for x in 0..6 {
        let radius = 0.5;
        let xx = ((x as f32) - 1.0) * radius * 1.5;
        for z in 0..6 {
            let zz = ((z as f32) - 1.0) * radius * 1.5;
            bodies.add(Body {
                position: Vec3::new(xx, 10.0, zz),
                orientation: Quat::IDENTITY,
                linear_velocity: Vec3::ZERO,
                angular_velocity: Vec3::ZERO,
                inv_mass: 1.0,
                elasticity: 0.5,
                friction: 0.5,
                shape: ball_shape.clone(),
            });
        }
    }
}

#[allow(dead_code)]
fn add_distance_constraint(bodies: &mut BodyArena, constraints: &mut ConstraintArena) {
    let cube_shape = make_cube_small();

    let handle_a = bodies.add(Body {
        position: Vec3::new(0.0, 5.0, 0.0),
        orientation: Quat::IDENTITY,
        linear_velocity: Vec3::ZERO,
        angular_velocity: Vec3::ZERO,
        inv_mass: 0.0,
        elasticity: 1.0,
        friction: 0.5,
        shape: cube_shape.clone(),
    });

    let handle_b = bodies.add(Body {
        position: Vec3::new(1.0, 5.0, 0.0),
        orientation: Quat::IDENTITY,
        linear_velocity: Vec3::ZERO,
        angular_velocity: Vec3::ZERO,
        inv_mass: 1.0,
        elasticity: 1.0,
        friction: 0.5,
        shape: cube_shape.clone(),
    });

    constraints.add_distance_constraint(bodies, handle_a, handle_b);
}

#[allow(dead_code)]
fn add_box_chain(bodies: &mut BodyArena, constraints: &mut ConstraintArena) {
    let cube_shape = make_cube_small();

    let mut handle_a = bodies.add(Body {
        position: Vec3::new(0.0, NUM_JOINTS as f32 + 3.0, 5.0),
        orientation: Quat::IDENTITY,
        inv_mass: 0.0,
        elasticity: 1.0,
        shape: cube_shape.clone(),
        ..Body::default()
    });

    const NUM_JOINTS: usize = 5;
    for _ in 0..NUM_JOINTS {
        let body_a = bodies.get_body(handle_a);

        let body_b = Body {
            position: body_a.position + Vec3::X,
            inv_mass: 1.0,
            elasticity: 1.0,
            shape: cube_shape.clone(),
            ..Body::default()
        };

        let handle_b = bodies.add(body_b);

        constraints.add_distance_constraint(bodies, handle_a, handle_b);

        handle_a = handle_b;
    }
}

#[allow(dead_code)]
fn add_box_stack(bodies: &mut BodyArena) {
    let cube_shape = make_cube_unit();

    let x = 0;
    let z = 0;
    const STACK_HEIGHT: usize = 5;
    for y in 0..STACK_HEIGHT {
        let offset = if y & 1 == 0 { 0.0 } else { 0.15 };
        let xx = x as f32 + offset;
        let zz = z as f32 + offset;
        let delta = 0.04;
        let scale_height = 2.0 + delta;
        let delta_height = 1.0 + delta;
        bodies.add(Body {
            position: Vec3::new(
                xx * scale_height,
                delta_height + y as f32 * scale_height,
                zz * scale_height,
            ),
            orientation: Quat::IDENTITY,
            inv_mass: 1.0,
            elasticity: 0.5,
            friction: 0.5,
            shape: cube_shape.clone(),
            ..Body::default()
        });
    }
}

#[allow(dead_code)]
fn add_hinge_constraint(bodies: &mut BodyArena, constraints: &mut ConstraintArena) {
    let cube_shape = make_cube_small();

    let handle_a = bodies.add(Body {
        position: Vec3::new(-2.0, 6.0, -5.0),
        orientation: Quat::from_axis_angle(
            Vec3::new(1.0, 1.0, 1.0).normalize(),
            std::f32::consts::FRAC_PI_4,
        ),
        inv_mass: 0.0,
        elasticity: 0.9,
        friction: 0.5,
        shape: cube_shape.clone(),
        ..Body::default()
    });

    let handle_b = bodies.add(Body {
        position: Vec3::new(-2.0, 5.0, -5.0),
        orientation: Quat::from_axis_angle(
            Vec3::new(0.0, 1.0, 1.0).normalize(),
            std::f32::consts::FRAC_PI_4,
        ),
        inv_mass: 1.0,
        elasticity: 1.0,
        friction: 0.5,
        shape: cube_shape.clone(),
        ..Body::default()
    });

    let body_a = bodies.get_body(handle_a);
    constraints.add_hinge_constraint(
        bodies,
        handle_a,
        handle_b,
        body_a.position,
        body_a.orientation.inverse() * Vec3::X,
    );
}

#[allow(dead_code)]
fn add_constant_velocity_constraint(bodies: &mut BodyArena, constraints: &mut ConstraintArena) {
    let cube_shape = make_cube_small();

    let handle_a = bodies.add(Body {
        position: Vec3::new(2.0, 6.0, -5.0),
        orientation: Quat::from_axis_angle(
            Vec3::new(1.0, 1.0, 1.0).normalize(),
            std::f32::consts::FRAC_PI_2,
        ),
        inv_mass: 0.0,
        elasticity: 0.9,
        friction: 0.5,
        shape: cube_shape.clone(),
        ..Body::default()
    });

    let handle_b = bodies.add(Body {
        position: Vec3::new(2.0, 5.0, -5.0),
        orientation: Quat::from_axis_angle(
            Vec3::new(0.0, 1.0, 1.0).normalize(),
            std::f32::consts::FRAC_PI_2,
        ),
        inv_mass: 1.0,
        elasticity: 1.0,
        friction: 0.5,
        shape: cube_shape.clone(),
        ..Body::default()
    });

    let body_a = bodies.get_body(handle_a);
    constraints.add_constant_velocity_constraint(
        bodies,
        handle_a,
        handle_b,
        body_a.position,
        body_a.orientation.inverse() * Vec3::X,
    )
}

#[allow(dead_code)]
fn add_rag_doll(bodies: &mut BodyArena, constraints: &mut ConstraintArena, offset: Vec3) {
    const T2: f32 = 0.25;
    const W2: f32 = T2 * 2.0;
    const H3: f32 = T2 * 4.0;
    const BOX_TORSO: [Vec3; 8] = [
        const_vec3!([-T2, -H3, -W2]),
        const_vec3!([T2, -H3, -W2]),
        const_vec3!([-T2, -H3, W2]),
        const_vec3!([T2, -H3, W2]),
        const_vec3!([-T2, H3, -W2]),
        const_vec3!([T2, H3, -W2]),
        const_vec3!([-T2, H3, W2]),
        const_vec3!([T2, H3, W2]),
    ];

    const H2: f32 = 0.25;
    const BOX_LIMB: [Vec3; 8] = [
        const_vec3!([-H3, -H2, -H2]),
        const_vec3!([H3, -H2, -H2]),
        const_vec3!([-H3, -H2, H2]),
        const_vec3!([H3, -H2, H2]),
        const_vec3!([-H3, H2, -H2]),
        const_vec3!([H3, H2, -H2]),
        const_vec3!([-H3, H2, H2]),
        const_vec3!([H3, H2, H2]),
    ];

    let head_shape = make_cube_small();
    let torso_shape = make_box_from_points(&BOX_TORSO);
    let limb_shape = make_box_from_points(&BOX_LIMB);

    let head_handle = bodies.add(Body {
        position: Vec3::new(0.0, 5.5, 0.0) + offset,
        inv_mass: 2.0,
        elasticity: 1.0,
        friction: 1.0,
        shape: head_shape,
        ..Body::default()
    });

    let torso_handle = bodies.add(Body {
        position: Vec3::new(0.0, 4.0, 0.0) + offset,
        inv_mass: 0.5,
        elasticity: 1.0,
        friction: 1.0,
        shape: torso_shape,
        ..Body::default()
    });

    let left_arm_handle = bodies.add(Body {
        position: Vec3::new(0.0, 4.75, 2.0) + offset,
        orientation: Quat::from_axis_angle(Vec3::Y, -3.1415 / 2.0),
        inv_mass: 1.0,
        elasticity: 1.0,
        friction: 1.0,
        shape: limb_shape.clone(),
        ..Body::default()
    });

    let right_arm_handle = bodies.add(Body {
        position: Vec3::new(0.0, 4.75, -2.0) + offset,
        orientation: Quat::from_axis_angle(Vec3::Y, 3.1415 / 2.0),
        inv_mass: 1.0,
        elasticity: 1.0,
        friction: 1.0,
        shape: limb_shape.clone(),
        ..Body::default()
    });

    let left_leg_handle = bodies.add(Body {
        position: Vec3::new(0.0, 2.5, 1.0) + offset,
        orientation: Quat::from_axis_angle(Vec3::Z, 3.1415 / 2.0),
        inv_mass: 1.0,
        elasticity: 1.0,
        friction: 1.0,
        shape: limb_shape.clone(),
        ..Body::default()
    });

    let right_leg_handle = bodies.add(Body {
        position: Vec3::new(0.0, 2.5, -1.0) + offset,
        orientation: Quat::from_axis_angle(Vec3::Z, 3.1415 / 2.0),
        inv_mass: 1.0,
        elasticity: 1.0,
        friction: 1.0,
        shape: limb_shape.clone(),
        ..Body::default()
    });

    // neck
    {
        let head_body = bodies.get_body(head_handle);
        constraints.add_hinge_constraint(
            bodies,
            head_handle,
            torso_handle,
            head_body.position + Vec3::new(0.0, -0.5, 0.0),
            head_body.orientation.inverse() * Vec3::Z,
        );
    }

    let torso_body = bodies.get_body(torso_handle);

    // shoulder left
    {
        let left_arm_body = bodies.get_body(left_arm_handle);
        constraints.add_constant_velocity_constraint(
            bodies,
            torso_handle,
            left_arm_handle,
            left_arm_body.position + Vec3::new(0.0, 0.0, -1.0),
            torso_body.orientation.inverse() * Vec3::Z,
        );
    }

    // shoulder right
    {
        let right_arm_body = bodies.get_body(right_arm_handle);
        constraints.add_constant_velocity_constraint(
            bodies,
            torso_handle,
            right_arm_handle,
            right_arm_body.position + Vec3::new(0.0, 0.0, 1.0),
            torso_body.orientation.inverse() * -Vec3::Z,
        );
    }

    // hip left
    {
        let left_leg_body = bodies.get_body(left_leg_handle);
        constraints.add_constant_velocity_constraint(
            bodies,
            torso_handle,
            left_leg_handle,
            left_leg_body.position + Vec3::new(0.0, 0.5, 0.0),
            torso_body.orientation.inverse() * Vec3::Z,
        );
    }

    // hip right
    {
        let right_leg_body = bodies.get_body(right_leg_handle);
        constraints.add_constant_velocity_constraint(
            bodies,
            torso_handle,
            right_leg_handle,
            right_leg_body.position + Vec3::new(0.0, 0.5, 0.0),
            torso_body.orientation.inverse() * Vec3::Z,
        );
    }
}

#[allow(dead_code)]
fn add_motor_constraint(bodies: &mut BodyArena, constraints: &mut ConstraintArena) {
    const L: f32 = 3.0;
    const T: f32 = 0.25;
    const BOX_BEAM: [Vec3; 8] = [
        const_vec3!([-L, -T, -T]),
        const_vec3!([L, -T, -T]),
        const_vec3!([-L, -T, T]),
        const_vec3!([L, -T, T]),
        const_vec3!([-L, T, -T]),
        const_vec3!([L, T, -T]),
        const_vec3!([-L, T, T]),
        const_vec3!([L, T, T]),
    ];

    let box_small = make_cube_small();
    let box_beam = make_box_from_points(&BOX_BEAM);

    let motor_pos = Vec3::new(5.0, 2.0, 0.0);
    let motor_axis = Vec3::Y;
    let motor_orient = Quat::from_xyzw(1.0, 0.0, 0.0, 0.0);

    let handle_a = bodies.add(Body {
        position: motor_pos,
        inv_mass: 0.0,
        elasticity: 0.9,
        friction: 0.5,
        shape: box_small,
        ..Body::default()
    });

    let handle_b = bodies.add(Body {
        position: motor_pos - motor_axis,
        orientation: motor_orient,
        inv_mass: 0.01,
        elasticity: 1.0,
        friction: 0.5,
        shape: box_beam,
        ..Body::default()
    });

    let body_a = bodies.get_body(handle_a);

    constraints.add_constraint_motor(
        bodies,
        handle_a,
        handle_b,
        body_a.position,
        body_a.orientation.inverse() * motor_axis,
        2.0,
    );

    bodies.add(Body {
        position: motor_pos + Vec3::new(2.0, 2.0, 0.0),
        inv_mass: 1.0,
        elasticity: 0.1,
        friction: 0.9,
        shape: make_sphere(1.0),
        ..Body::default()
    });
}

#[allow(dead_code)]
fn add_mover_constraint(bodies: &mut BodyArena, constraints: &mut ConstraintArena) {
    const L: f32 = 3.0;
    const T: f32 = 0.25;
    const BOX_PLATFORM: [Vec3; 8] = [
        const_vec3!([-L, -T, -L]),
        const_vec3!([L, -T, -L]),
        const_vec3!([-L, -T, L]),
        const_vec3!([L, -T, L]),
        const_vec3!([-L, T, -L]),
        const_vec3!([L, T, -L]),
        const_vec3!([-L, T, L]),
        const_vec3!([L, T, L]),
    ];

    let box_platform = make_box_from_points(&BOX_PLATFORM);
    let handle_a = bodies.add(Body {
        position: Vec3::new(10.0, 5.0, 0.0),
        inv_mass: 0.0,
        elasticity: 0.1,
        friction: 0.9,
        shape: box_platform,
        ..Body::default()
    });

    constraints.add_constraint_mover(bodies, handle_a);

    bodies.add(Body {
        position: Vec3::new(10.0, 6.3, 0.0),
        inv_mass: 1.0,
        elasticity: 0.1,
        friction: 0.9,
        shape: make_cube_unit(),
        ..Body::default()
    });
}

#[allow(dead_code)]
fn add_standard_sandbox(bodies: &mut BodyArena) {
    let wall_color = Vec3::splat(0.5);

    let box_ground = make_box_ground();
    let box_wall0 = make_box_wall0();
    let box_wall1 = make_box_wall1();

    bodies.add_with_color(
        Body {
            position: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            elasticity: 0.5,
            friction: 0.5,
            shape: box_ground,
        },
        Vec3::new(0.3, 0.5, 0.3),
    );

    bodies.add_with_color(
        Body {
            position: Vec3::new(50.0, 0.0, 0.0),
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            elasticity: 0.5,
            friction: 0.0,
            shape: box_wall0.clone(),
        },
        wall_color,
    );

    bodies.add_with_color(
        Body {
            position: Vec3::new(-50.0, 0.0, 0.0),
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            elasticity: 0.5,
            friction: 0.0,
            shape: box_wall0,
        },
        wall_color,
    );

    bodies.add_with_color(
        Body {
            position: Vec3::new(0.0, 0.0, 25.0),
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            elasticity: 0.5,
            friction: 0.0,
            shape: box_wall1.clone(),
        },
        wall_color,
    );

    bodies.add_with_color(
        Body {
            position: Vec3::new(0.0, 0.0, -25.0),
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            elasticity: 0.5,
            friction: 0.0,
            shape: box_wall1,
        },
        wall_color,
    );
}

fn resolve_contact(bodies: &mut BodyArena, contact: &Contact) {
    let (body_a, body_b) = bodies.get_body_pair_mut(contact.handle_a, contact.handle_b);
    debug_assert!(!body_a.has_infinite_mass() || !body_b.has_infinite_mass());

    let point_on_a = body_a.local_to_world(contact.local_point_a);
    let point_on_b = body_b.local_to_world(contact.local_point_b);

    let elasticity = body_a.elasticity * body_b.elasticity;

    let inv_inertia_world_a = body_a.inv_intertia_tensor_world();
    let inv_inertia_world_b = body_b.inv_intertia_tensor_world();

    let ra = point_on_a - body_a.centre_of_mass_world();
    let rb = point_on_b - body_b.centre_of_mass_world();

    let angular_j_a = (inv_inertia_world_a * ra.cross(contact.normal)).cross(ra);
    let angular_j_b = (inv_inertia_world_b * rb.cross(contact.normal)).cross(rb);
    let angular_factor = (angular_j_a + angular_j_b).dot(contact.normal);

    // calculate the world space velocity of the motion and rotation
    let vel_a = body_a.linear_velocity + body_a.angular_velocity.cross(ra);
    let vel_b = body_b.linear_velocity + body_b.angular_velocity.cross(rb);

    // calculate the collision impulse
    let vab = vel_a - vel_b;
    let total_inv_mass = body_a.inv_mass + body_b.inv_mass;
    let impulse_j =
        (1.0 + elasticity) * vab.dot(contact.normal) / (total_inv_mass + angular_factor);
    let vec_impulse_j = contact.normal * impulse_j;

    body_a.apply_impulse(point_on_a, -vec_impulse_j);
    body_b.apply_impulse(point_on_b, vec_impulse_j);

    // calculate the impulse caused by friction
    let friction = body_a.friction * body_b.friction;

    // find the normal direction of the velocity with respect to the normal of the collision
    let vel_normal = contact.normal * contact.normal.dot(vab);

    // find the tangent direction of the velocity with respect to the normal of the collision
    let vel_tan = vab - vel_normal;

    // get the tangential velocities relative to the other body
    let rel_vel_tan = vel_tan.normalize_or_zero();

    let inertia_a = (inv_inertia_world_a * ra.cross(rel_vel_tan)).cross(ra);
    let inertia_b = (inv_inertia_world_b * rb.cross(rel_vel_tan)).cross(rb);
    let inv_inertia = (inertia_a + inertia_b).dot(rel_vel_tan);

    // calculate the tangential impulse for friction
    let reduced_mass = 1.0 / (total_inv_mass + inv_inertia);
    let impulse_friction = vel_tan * reduced_mass * friction;

    // apply kinetic friction
    body_a.apply_impulse(point_on_a, -impulse_friction);
    body_b.apply_impulse(point_on_b, impulse_friction);

    // also move colliding objects to just outside of each other (projection method)
    if contact.time_of_impact == 0.0 {
        let ds = point_on_b - point_on_a;

        let rcp_total_inv_mass = 1.0 / total_inv_mass;
        let t_a = body_a.inv_mass * rcp_total_inv_mass;
        let t_b = body_b.inv_mass * rcp_total_inv_mass;

        body_a.position += ds * t_a;
        body_b.position -= ds * t_b;
    }
}

pub struct PhysicsScene {
    bodies: BodyArena,
    constraints: ConstraintArena,
    contacts: ContactArena,
    manifolds: ManifoldCollector,
    step_num: u64,
    pub paused: bool,
}

impl PhysicsScene {
    pub fn new() -> Self {
        let mut scene = PhysicsScene {
            bodies: BodyArena::default(),
            constraints: ConstraintArena::default(),
            contacts: ContactArena::default(),
            manifolds: ManifoldCollector::default(),
            step_num: 0,
            paused: true,
        };
        scene.reset();
        scene
    }

    pub fn reset(&mut self) {
        self.step_num = 0;
        self.bodies.clear();
        self.constraints.clear();
        self.contacts.clear();
        self.manifolds.clear();

        // add_dynamic_balls(&mut self.bodies);

        // add_distance_constraint(&mut self.bodies, &mut self.constraints);

        add_rag_doll(
            &mut self.bodies,
            &mut self.constraints,
            Vec3::new(-5.0, 0.0, 0.0),
        );

        add_box_chain(&mut self.bodies, &mut self.constraints);

        add_box_stack(&mut self.bodies);

        add_sphere(&mut self.bodies);

        add_convex_hull(&mut self.bodies);

        add_motor_constraint(&mut self.bodies, &mut self.constraints);

        add_mover_constraint(&mut self.bodies, &mut self.constraints);

        add_hinge_constraint(&mut self.bodies, &mut self.constraints);

        add_constant_velocity_constraint(&mut self.bodies, &mut self.constraints);

        add_teleportation_fix(&mut self.bodies);

        add_orientation_constraint(&mut self.bodies, &mut self.constraints);

        add_standard_sandbox(&mut self.bodies);

        let max_contacts = self.bodies.len() * self.bodies.len();
        self.contacts.clear_with_capacity(max_contacts);

        self.paused = true;
    }

    pub fn update(&mut self, delta_seconds: f32) {
        self.step_num += 1;

        self.manifolds.remove_expired(&self.bodies);

        // gravity impulse
        for body in self.bodies.iter_mut() {
            if !body.has_infinite_mass() {
                // gravity needs to be an impulse
                // I = dp, F = dp/dt => dp = F * dt => I = F * dt
                // F = mgs
                let impulse_gravity =
                    Vec3::new(0.0, -10.0, 0.0) * body.inv_mass.recip() * delta_seconds;
                body.apply_impulse_linear(impulse_gravity);
            }
        }

        // broadphase (build potential collision pairs)
        let collision_pairs = broadphase(&self.bodies, delta_seconds);

        // narrowphase (perform actual collision detection)
        self.contacts.clear();
        for pair in collision_pairs {
            let (body_a, body_b) = self.bodies.get_body_pair_mut(pair.a, pair.b);

            // skip body pairs with infinite mass
            if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
                continue;
            }

            // check for intersection
            if let Some(contact) = intersect_dynamic(pair.a, body_a, pair.b, body_b, delta_seconds)
            {
                if contact.time_of_impact == 0.0 {
                    // static contact
                    self.manifolds.add_contact(&self.bodies, contact);
                } else {
                    // ballistic contact
                    self.contacts.push(contact)
                }
            }
        }

        // sort the times of impact from earliest to latest
        self.contacts.sort();

        // solve constraints
        self.constraints.pre_solve(&mut self.bodies, delta_seconds);
        self.manifolds.pre_solve(&mut self.bodies, delta_seconds);

        const MAX_ITERS: u32 = 5;
        for _ in 0..MAX_ITERS {
            self.constraints.solve(&mut self.bodies);
            self.manifolds.solve(&mut self.bodies);
        }

        self.constraints.post_solve();
        self.manifolds.post_solve();

        // apply ballistic impulses
        let mut accumulated_time = 0.0;
        for contact in self.contacts.iter() {
            let contact_time = contact.time_of_impact - accumulated_time;

            // position update
            for body in self.bodies.iter_mut() {
                body.update(contact_time)
            }

            resolve_contact(&mut self.bodies, contact);
            accumulated_time += contact_time;
        }

        // update positions for the rest of this frame's time
        let time_remaining = delta_seconds - accumulated_time;
        if time_remaining > 0.0 {
            for body in self.bodies.iter_mut() {
                body.update(time_remaining);
            }
        }

        // self.bodies.print_bodies(self.step_num, delta_seconds);
    }

    pub fn get_body(&self, handle: BodyHandle) -> &Body {
        self.bodies.get_body(handle)
    }

    pub fn get_body_with_color(&self, handle: BodyHandle) -> (&Body, Vec3) {
        self.bodies.get_body_with_color(handle)
    }

    pub fn iter_body_handles(&self) -> core::slice::Iter<BodyHandle> {
        self.bodies.handles().iter()
    }
}

impl Default for PhysicsScene {
    fn default() -> Self {
        Self::new()
    }
}
