use geom::{Circle, Segment, Vec2, Vec3, AABB, OBB};

pub trait GJKObject {
    /// Returns the support point in the direction of `dir`.
    /// Dir is a normalized vector.
    fn support(&self, dir: Vec2) -> Vec2;
}

impl<T> GJKObject for &T
where
    T: GJKObject,
{
    fn support(&self, dir: Vec2) -> Vec2 {
        (*self).support(dir)
    }
}

impl GJKObject for AABB {
    fn support(&self, dir: Vec2) -> Vec2 {
        match (dir.x > 0.0, dir.y > 0.0) {
            (true, true) => self.ur,
            (true, false) => Vec2::new(self.ur.x, self.ll.y),
            (false, true) => Vec2::new(self.ll.x, self.ur.y),
            (false, false) => self.ll,
        }
    }
}

impl GJKObject for OBB {
    fn support(&self, dir: Vec2) -> Vec2 {
        let mut max_dot = self.corners[0].dot(dir);
        let mut best_corner = self.corners[0];

        for i in 1..4 {
            let corner = self.corners[i];
            let dot = corner.dot(dir);
            if dot >= max_dot {
                max_dot = dot;
                best_corner = corner;
            }
        }

        best_corner
    }
}

impl GJKObject for Circle {
    fn support(&self, dir: Vec2) -> Vec2 {
        self.center + dir * self.radius
    }
}

pub fn gjk(a: impl GJKObject, b: impl GJKObject) -> Option<GJKTriangle> {
    let minkowski_diff = |dir: Vec2| a.support(dir) - b.support(-dir);

    let first_point = minkowski_diff(Vec2::X);
    let second_point = minkowski_diff(-first_point.normalize());

    if first_point.dot(second_point) > 0.0 {
        return None;
    }

    let mut simplex = Segment::new(first_point, second_point);
    let mut dir = segment_dir_towards_zero(&simplex);

    // limit iterations to 32
    // If we're going there it means it's barely touching anyway
    for _ in 0..32 {
        let new_point = minkowski_diff(dir);

        if new_point.dot(dir) < 0.0 {
            return None;
        }

        let triangle = GJKTriangle {
            b: simplex.src,
            c: simplex.dst,
            last_point: new_point,
        };

        let Some((new_simplex, new_dir)) = triangle.nearest_simplex() else {
            return Some(triangle);
        };
        simplex = new_simplex;
        dir = new_dir;
    }

    return None;
}

fn segment_dir_towards_zero(segment: &Segment) -> Vec2 {
    let ab: Vec2 = segment.dst - segment.src;
    let ao = -segment.src;

    perp_towards_dir(ab, ao).normalize()
}

#[derive(Copy, Clone)]
pub struct GJKTriangle {
    pub b: Vec2,
    pub c: Vec2,
    pub last_point: Vec2,
}

impl GJKTriangle {
    /// Returns the simplex of self closest to the origin and a direction (normalized) toward the origin that is normal to the new simplex.
    /// Returns None if the origin is inside the simplex.
    /// This function must be called in gjk context as it assumes the simplex was found through GJK,
    /// therefore the closest simplex of a triangle can never be a point, same goes for segment.
    pub fn nearest_simplex(&self) -> Option<(Segment, Vec2)> {
        let GJKTriangle {
            b,
            c,
            last_point: a,
        } = *self;
        let ab = b - a;
        let ac = c - a;

        let ab_normal = -perp_towards_dir(ab, ac);
        let ac_normal = -perp_towards_dir(ac, ab);

        let is_ab_normal_towards_origin = ab_normal.dot(a) <= 0.0;
        let is_ac_normal_towards_origin = ac_normal.dot(a) <= 0.0;

        if !is_ab_normal_towards_origin && !is_ac_normal_towards_origin {
            // Origin is inside the triangle
            None
        } else if is_ab_normal_towards_origin {
            Some((Segment::new(a, b), ab_normal.normalize()))
        } else {
            Some((Segment::new(a, c), ac_normal.normalize()))
        }
    }
}

/// Computes (v x dir) x v
pub fn perp_towards_dir(v: Vec2, dir: Vec2) -> Vec2 {
    dir.cross(v).signum() * v.perpendicular()
}
