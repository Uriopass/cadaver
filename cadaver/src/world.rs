use crate::gjk::gjk;
use crate::rigidbody::RigidBody;
use slotmap::{new_key_type, SlotMap};

new_key_type! {
    pub struct RigidBodyID;
}

#[derive(Default)]
pub struct RigidWorld {
    pub bodies: SlotMap<RigidBodyID, RigidBody>,
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

    pub fn collisions(&self) -> Vec<(RigidBodyID, RigidBodyID)> {
        let mut pairs = Vec::new();
        for (id1, body1) in self.bodies.iter() {
            for (id2, body2) in self.bodies.iter() {
                if id1 == id2 {
                    continue;
                }
                if gjk(body1.oriented_shape(), body2.oriented_shape()) {
                    pairs.push((id1, id2));
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
