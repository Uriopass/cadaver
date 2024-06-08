use cadaver::rigidbody::{RigidBody, RigidBodyShape};
use cadaver::world::{RigidBodyID, RigidWorld};
use cadaver::{read_debug_lines, read_debug_points};
use geom::{vec2, AABB};
use macroquad::prelude::*;
use std::collections::HashSet;
use std::time::Instant;

async fn amain() {
    let mut state = RigidWorld::new();

    let key1 = state.add_body(RigidBody::new(
        AABB::centered(vec2(0.0, 0.0), vec2(100.0, 100.0)),
        1.0,
    ));

    let key2 = state.add_body(
        RigidBody::new(geom::Circle::new(geom::Vec2::ZERO, 50.0), 1.0).with_pos(vec2(200.0, 0.0)),
    );

    let mut last = Instant::now();
    loop {
        let dt = last.elapsed().as_secs_f32().min(1.0 / 30.0);
        last = Instant::now();

        state.step(dt);

        if is_mouse_button_pressed(MouseButton::Left) {
            let body = state.get_mut(key1).unwrap();
            body.apply_impulse(vec2(0.0, 20.0), body.center());

            let body = state.get_mut(key2).unwrap();
            body.apply_impulse(vec2(0.0, 10.0), body.center() + vec2(10.0, 0.0));
        }

        let camera = Camera2D {
            zoom: macroquad::prelude::vec2(1.0 / screen_width(), -1.0 / screen_height()),
            ..Default::default()
        };
        let sw = screen_width();
        let sh = screen_height();
        set_camera(&camera);
        clear_background(BLACK);

        draw_grid(20, 1.0, DARKGRAY, DARKGRAY);

        let mouse_screen_pos = mouse_position();
        let mouse_pos = camera.screen_to_world(macroquad::prelude::vec2(
            mouse_screen_pos.0,
            mouse_screen_pos.1,
        ));

        state.get_mut(key2).unwrap().pos = vec2(mouse_pos.x, mouse_pos.y);

        let collisions = state.collisions();

        let ids_colliding: HashSet<RigidBodyID> = collisions
            .iter()
            .flat_map(|info| [info.id1, info.id2])
            .collect();

        for (body_id, body) in state.iter() {
            let color = if ids_colliding.contains(&body_id) {
                RED
            } else {
                WHITE
            };

            let shape = body.oriented_shape();

            match shape {
                RigidBodyShape::OBB(o) => {
                    for segment in o.segments() {
                        draw_line(
                            segment.src.x,
                            segment.src.y,
                            segment.dst.x,
                            segment.dst.y,
                            3.0,
                            color,
                        );
                    }
                }
                RigidBodyShape::Circle(c) => {
                    draw_circle(c.center.x, c.center.y, c.radius, color);
                }
                _ => {}
            }
        }

        for pair in collisions {
            let body1 = state.get(pair.id1).unwrap();
            let body2 = state.get(pair.id2).unwrap();

            let normal = pair.normal.normalize();

            draw_circle(pair.contact_1.x, pair.contact_1.y, 5.0, GREEN);
            draw_circle(pair.contact_2.x, pair.contact_2.y, 5.0, GREEN);
        }

        for point in read_debug_points() {
            draw_circle(point.x, point.y, 5.0, GREEN);
        }

        for (src, dst) in read_debug_lines() {
            draw_line(src.x, src.y, dst.x, dst.y, 3.0, GREEN);
        }

        next_frame().await
    }
}

fn main() {
    macroquad::Window::from_config(Conf::default(), amain());
}
