pub mod epa;
pub mod gjk;
pub mod rigidbody;
pub mod world;

use geom::Vec2;
lazy_static::lazy_static! {
    pub static ref DEBUG_POINTS: std::sync::Mutex<Vec<Vec2>> = Default::default();
    pub static ref DEBUG_LINES: std::sync::Mutex<Vec<(Vec2, Vec2)>> = Default::default();
}

pub fn debug_line(start: impl Into<Vec2>, end: impl Into<Vec2>) {
    DEBUG_LINES.lock().unwrap().push((start.into(), end.into()));
}

pub fn read_debug_lines() -> Vec<(Vec2, Vec2)> {
    DEBUG_LINES.lock().unwrap().drain(..).collect()
}

pub fn debug_point(point: impl Into<Vec2>) {
    DEBUG_POINTS.lock().unwrap().push(point.into());
}

pub fn read_debug_points() -> Vec<Vec2> {
    DEBUG_POINTS.lock().unwrap().drain(..).collect()
}
