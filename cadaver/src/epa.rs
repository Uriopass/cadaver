use crate::debug_line;
use crate::gjk::{perp_towards_dir, GJKObject, GJKTriangle};
use geom::Vec2;

pub struct EPAResult {
    pub normal: Vec2,
    pub distance: f32,
    pub contact_a: Vec2,
    pub contact_b: Vec2,
}

pub fn epa(
    a: impl GJKObject,
    b: impl GJKObject,
    starting_simplex: GJKTriangle,
) -> Option<EPAResult> {
    let mut polytope: Vec<Vec2> = vec![
        starting_simplex.last_point,
        starting_simplex.b,
        starting_simplex.c,
    ];

    let winding = (starting_simplex.b - starting_simplex.c)
        .cross(starting_simplex.b - starting_simplex.last_point);

    if winding < 0.0 {
        polytope.swap(1, 2);
    }
    for (a, b) in polytope.iter().zip(polytope.iter().cycle().skip(1)) {
        debug_line(*a, *b);
    }
    for i in 0..256 {
        let (normal, distance, index) = find_closest_edge(&polytope);
        let new_point = a.support(normal) - b.support(-normal);

        let d = new_point.dot(normal);
        if d - distance < 0.01 {
            return Some(EPAResult {
                normal,
                distance,
                contact_a: a.support(normal),
                contact_b: b.support(-normal),
            });
        }
        polytope.insert(index, new_point);
    }

    None
}

fn find_closest_edge(polytope: &[Vec2]) -> (Vec2, f32, usize) {
    let mut closest_edge = (Vec2::ZERO, 0);
    let mut closest_distance = f32::INFINITY;
    for i in 0..polytope.len() {
        let mut j = i + 1;
        if j == polytope.len() {
            j = 0;
        }
        let a = polytope[i];
        let b = polytope[j];

        let e = b - a;
        let n = e.perpendicular().normalize();

        let d = n.dot(a);

        if d <= closest_distance {
            closest_edge = (n, j);
            closest_distance = d;
        }
    }
    (closest_edge.0, closest_distance, closest_edge.1)
}
