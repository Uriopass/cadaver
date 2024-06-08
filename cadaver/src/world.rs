use crate::epa::epa;
use crate::gjk::gjk;
use crate::rigidbody::RigidBody;
use geom::{Intersect, Shape};
use slotmap::{new_key_type, SlotMap};

new_key_type! {
    pub struct RigidBodyID;
}

#[derive(Default)]
pub struct RigidWorld {
    pub bodies: SlotMap<RigidBodyID, RigidBody>,
}

pub struct CollisionPair {
    pub id1: RigidBodyID,
    pub id2: RigidBodyID,
    pub depth: f32,
    pub normal: geom::Vec2,
    pub contact_1: geom::Vec2,
    pub contact_2: geom::Vec2,
}

impl RigidWorld {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_body(&mut self, body: RigidBody) -> RigidBodyID {
        self.bodies.insert(body)
    }

    pub fn step(&mut self, dt: f32) {
        for (_, body) in self.bodies.iter_mut() {
            body.step(dt);
        }
    }

    pub fn collisions(&self) -> Vec<CollisionPair> {
        let mut pairs = Vec::new();
        for (id1, body1) in self.bodies.iter() {
            for (id2, body2) in self.bodies.iter() {
                if id1 >= id2 {
                    continue;
                }
                let body1 = body1.oriented_shape();
                let body2 = body2.oriented_shape();

                if !body1.bbox().intersects(&body2.bbox()) {
                    continue;
                }

                if let Some(t) = gjk(body1, body2) {
                    if let Some(res) = epa(body1, body2, t) {
                        pairs.push(CollisionPair {
                            id1,
                            id2,
                            depth: res.distance,
                            normal: res.normal,
                            contact_1: res.contact_a,
                            contact_2: res.contact_b,
                        });
                    }
                }
            }
        }
        pairs
    }

    pub fn get(&self, id: RigidBodyID) -> Option<&RigidBody> {
        self.bodies.get(id)
    }

    pub fn get_mut(&mut self, id: RigidBodyID) -> Option<&mut RigidBody> {
        self.bodies.get_mut(id)
    }

    pub fn iter(&self) -> impl Iterator<Item = (RigidBodyID, &RigidBody)> {
        self.bodies.iter()
    }
}
