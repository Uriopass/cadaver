use geom::{Radians, Shape, Vec2, AABB, OBB};

pub struct RigidBody {
    // Static
    /// Mass in kilograms
    mass: f32,
    shape: AABB,
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
        self.shape.bbox()
    }

    pub fn center(&self) -> Vec2 {
        self.shape.center()
    }

    pub fn linear_energy(&self) -> f32 {
        (0.5 * self.mass) * self.linear_velocity.mag2()
    }

    pub fn angular_energy(&self) -> f32 {
        (0.5 * self.mass) * (self.angular_velocity.0 * self.angular_velocity.0)
    }

    pub fn energy(&self) -> f32 {
        (0.5 * self.mass)
            * (self.linear_velocity.mag2() + self.angular_velocity.0 * self.angular_velocity.0)
    }

    /// Apply an impulse to the rigid body.
    /// The impulse is applied at the contact point.
    /// The impulse is in N.s or kg m/s.
    pub fn apply_impulse(&mut self, mut impulse: Vec2, contact: Vec2) {
        #[cfg(debug_assertions)]
        let before_energy = self.energy();

        impulse /= self.mass;

        match (self.pos - contact).try_normalize() {
            Some(normal) => {
                let angle_coeff = normal.dot(impulse).abs();
                let angle_dir = normal.perp_dot(impulse).signum();

                self.linear_velocity += impulse * angle_coeff.abs();
                self.angular_velocity += Radians(impulse.mag() * angle_coeff * angle_dir);
            }
            None => {
                self.linear_velocity += impulse;
            }
        }

        #[cfg(debug_assertions)]
        {
            let after_energy = self.energy();
            if (before_energy - after_energy).abs() >= 1e-6 {
                eprintln!(
                    "Energy was not conserved: before = {}, after = {}. (diff = {})",
                    before_energy,
                    after_energy,
                    (before_energy - after_energy).abs()
                );
            }
        }
    }
}
