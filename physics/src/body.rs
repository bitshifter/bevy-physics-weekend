use crate::shapes::Shape;
use glam::{Mat3, Quat, Vec3};

#[derive(Clone, Debug)]
pub struct Body {
    pub position: Vec3,
    pub orientation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub inv_mass: f32,
    pub elasticity: f32,
    pub friction: f32,
    pub shape: Shape,
}

impl Default for Body {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 1.0,
            elasticity: 0.5,
            friction: 0.5,
            shape: Shape::default(),
        }
    }
}

impl Body {
    pub fn centre_of_mass_world(&self) -> Vec3 {
        let com = self.shape.centre_of_mass();
        self.position + self.orientation * com
    }

    // pub fn centre_of_mass_local(&self) -> Vec3 {
    //     self.shape.centre_of_mass()
    // }

    pub fn world_to_local(&self, world_point: Vec3) -> Vec3 {
        let tmp = world_point - self.centre_of_mass_world();
        let inv_orientation = self.orientation.conjugate();
        inv_orientation * tmp // body_space
    }

    pub fn local_to_world(&self, body_point: Vec3) -> Vec3 {
        self.centre_of_mass_world() + self.orientation * body_point // world_point
    }

    pub fn inv_intertia_tensor_world(&self) -> Mat3 {
        let inv_inertia_tensor = self.inv_intertia_tensor_local();
        let orientation = Mat3::from_quat(self.orientation);
        orientation * inv_inertia_tensor * orientation.transpose()
    }

    pub fn inv_intertia_tensor_local(&self) -> Mat3 {
        self.shape.inertia_tensor().inverse() * self.inv_mass
    }

    pub fn apply_impulse(&mut self, impulse_point: Vec3, impulse: Vec3) {
        if self.has_infinite_mass() {
            return;
        }

        // impulse_point is the world space location of the application of the impulse
        // impulse is the world space direction and magnitude of the impulse
        self.apply_impulse_linear(impulse);

        // applying impulses must produce torques through the centre of mass
        let position = self.centre_of_mass_world();

        let r = impulse_point - position;
        // this is in world space
        let dl = r.cross(impulse);

        self.apply_impulse_angular(dl);
    }

    pub fn apply_impulse_angular(&mut self, impulse: Vec3) {
        if self.has_infinite_mass() {
            return;
        }

        // L = I w = r x p
        // dL = I dw = r x J
        // => dw = I^-1 * (r x J)
        self.angular_velocity += self.inv_intertia_tensor_world() * impulse;

        // clamp angular_velocity - 30 rad/s is fast enough for us
        const MAX_ANGULAR_SPEED: f32 = 30.0;
        const MAX_ANGULAR_SPEED_SQ: f32 = MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED;
        if self.angular_velocity.length_squared() > MAX_ANGULAR_SPEED_SQ {
            self.angular_velocity = self.angular_velocity.normalize() * MAX_ANGULAR_SPEED;
        }
    }

    pub fn apply_impulse_linear(&mut self, impulse: Vec3) {
        if self.has_infinite_mass() {
            return;
        }

        // p = mv
        // dp = m dv = J
        // => dv = J / m
        self.linear_velocity += impulse * self.inv_mass;
    }

    pub fn update(&mut self, delta_seconds: f32) {
        self.position += self.linear_velocity * delta_seconds;

        // we have an angular velocity around the centre of mass, this needs to be converted to
        // relative body position. This way we can properly update the orientation of the model

        let position_com = self.centre_of_mass_world();
        let com_to_position = self.position - position_com;

        // total torque is equal to external applied torques + internal torque (precession)
        // T = T_external + omega x I * omega
        // T_external = 0 because it was applied in the collision response function
        // T = Ia = w x I * w
        // a = I^-1 (w x I * w)
        let orientation = Mat3::from_quat(self.orientation);
        let inertia_tensor = orientation * self.shape.inertia_tensor() * orientation.transpose();
        let alpha = inertia_tensor.inverse()
            * (self
                .angular_velocity
                .cross(inertia_tensor * self.angular_velocity));
        self.angular_velocity += alpha * delta_seconds;

        // update orientation
        let d_angle = self.angular_velocity * delta_seconds;
        let angle = d_angle.length();
		let rcp_angle = angle.recip();
		let dq = if rcp_angle.is_finite() {
			Quat::from_axis_angle(d_angle * rcp_angle, angle)
		} else {
			Quat::IDENTITY
		};
        self.orientation = (dq * self.orientation).normalize();

        // now get the new body position
        self.position = position_com + dq * com_to_position;
    }

    pub fn has_infinite_mass(&self) -> bool {
        self.inv_mass == 0.0
    }
}
