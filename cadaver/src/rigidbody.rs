use crate::gjk::GJKObject;
use geom::{Circle, Radians, Shape, Vec2, AABB, OBB};

#[derive(Copy, Clone, Debug)]
pub enum RigidBodyShape {
    AABB(AABB),
    OBB(OBB),
    Circle(Circle),
}

impl From<AABB> for RigidBodyShape {
    fn from(aabb: AABB) -> Self {
        RigidBodyShape::AABB(aabb)
    }
}

impl From<OBB> for RigidBodyShape {
    fn from(obb: OBB) -> Self {
        RigidBodyShape::OBB(obb)
    }
}

impl From<Circle> for RigidBodyShape {
    fn from(circle: Circle) -> Self {
        RigidBodyShape::Circle(circle)
    }
}

impl GJKObject for RigidBodyShape {
    fn support(&self, dir: Vec2) -> Vec2 {
        match self {
            RigidBodyShape::AABB(aabb) => aabb.support(dir),
            RigidBodyShape::OBB(obb) => obb.support(dir),
            RigidBodyShape::Circle(circle) => circle.support(dir),
        }
    }
}

impl RigidBodyShape {
    pub fn bbox(&self) -> AABB {
        match self {
            RigidBodyShape::AABB(aabb) => *aabb,
            RigidBodyShape::OBB(obb) => obb.bbox(),
            RigidBodyShape::Circle(circle) => circle.bbox(),
        }
    }

    pub fn center(&self) -> Vec2 {
        match self {
            RigidBodyShape::AABB(aabb) => aabb.center(),
            RigidBodyShape::OBB(obb) => obb.center(),
            RigidBodyShape::Circle(circle) => circle.center,
        }
    }

    pub fn moment_of_inertia(&self, mass: f32) -> f32 {
        match self {
            RigidBodyShape::AABB(aabb) => aabb.moment_of_inertia(mass),
            RigidBodyShape::OBB(obb) => obb.moment_of_inertia(mass),
            RigidBodyShape::Circle(circle) => circle.moment_of_inertia(mass),
        }
    }
}

pub struct RigidBody {
    // Static
    /// Mass in kilograms
    mass: f32,
    shape: RigidBodyShape,
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
    pub fn new(shape: impl Into<RigidBodyShape>, mass: f32) -> Self {
        let shape = shape.into();
        Self {
            mass,

            moment_of_inertia: shape.moment_of_inertia(mass),
            shape,

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

    pub fn shape(&self) -> RigidBodyShape {
        self.shape
    }

    pub fn oriented_shape(&self) -> RigidBodyShape {
        match self.shape {
            RigidBodyShape::AABB(aabb) => {
                RigidBodyShape::OBB(OBB::from_rotated_aabb(aabb, self.angle.vec2()) + self.pos)
            }
            RigidBodyShape::OBB(obb) => RigidBodyShape::OBB(obb + self.pos),
            RigidBodyShape::Circle(circle) => RigidBodyShape::Circle(Circle {
                center: circle.center + self.pos,
                radius: circle.radius,
            }),
        }
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
