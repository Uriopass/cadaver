use geom::{Radians, Shape, Vec2, AABB, OBB};

pub struct RigidBody {
    // Static
    /// Mass in kilograms
    mass: f32,
    shape: AABB,
    /// Moment of inertia in kg m^2
    moment_of_inertia: f32,

    // Dynamic
    pub pos: Vec2,
    /// Angle in radians
    pub angle: Radians,

    /// Linear momentum in m/s
    pub linear_velocity: Vec2,
    /// Angular momentum in rad/s
    pub angular_velocity: Radians,
}

impl RigidBody {
    pub fn new(shape: AABB, mass: f32) -> Self {
        Self {
            mass,

            shape,
            moment_of_inertia: shape.moment_of_inertia(mass),

            pos: Default::default(),
            angle: Default::default(),

            linear_velocity: Default::default(),
            angular_velocity: Default::default(),
        }
    }

    pub fn with_pos(self, pos: Vec2) -> Self {
        Self { pos, ..self }
    }

    pub fn mass(&self) -> f32 {
        self.mass
    }

    pub fn shape(&self) -> AABB {
        self.shape
    }

    pub fn oriented_shape(&self) -> OBB {
        OBB::from_rotated_aabb(self.shape, self.angle.vec2()) + self.pos
    }

    pub fn bbox(&self) -> AABB {
        todo!()
    }

    /// Returns the center of mass in world coordinates.
    pub fn center(&self) -> Vec2 {
        self.shape.center() + self.pos
    }

    /// 1/2 mv^2
    pub fn linear_energy(&self) -> f32 {
        (0.5 * self.mass) * self.linear_velocity.mag2()
    }

    /// 1/2 Iw^2
    pub fn angular_energy(&self) -> f32 {
        (0.5 * self.moment_of_inertia) * (self.angular_velocity.0 * self.angular_velocity.0)
    }

    pub fn energy(&self) -> f32 {
        0.5 * (self.mass * self.linear_velocity.mag2()
            + self.moment_of_inertia * self.angular_velocity.0 * self.angular_velocity.0)
    }

    pub fn step(&mut self, dt: f32) {
        self.pos += self.linear_velocity * dt;
        self.angle += self.angular_velocity * dt;
    }

    /// Apply an impulse to the rigid body.
    /// The impulse is applied at the contact point.
    /// The impulse is in N.s or kg m/s.
    pub fn apply_impulse(&mut self, impulse: Vec2, contact: Vec2) {
        match (self.center() - contact).try_normalize() {
            Some(normal) => {
                let angle_coeff = normal.dot(impulse.normalize()).abs();
                let angle_dir = normal.perp_dot(impulse).signum();

                self.linear_velocity += (impulse / self.mass) * angle_coeff;
                self.angular_velocity -= Radians(
                    impulse.mag2() / self.moment_of_inertia * (1.0 - angle_coeff) * angle_dir,
                );
            }
            None => {
                self.linear_velocity += impulse / self.mass;
            }
        }
    }
}
