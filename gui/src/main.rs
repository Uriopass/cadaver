use cadaver::rigidbody::RigidBody;
use cadaver::world::{RigidBodyID, RigidWorld};
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
        RigidBody::new(AABB::centered(vec2(0.0, 0.0), vec2(100.0, 100.0)), 1.0)
            .with_pos(vec2(200.0, 0.0)),
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

        let mouse_screen_pos = mouse_position();
        let mouse_pos = camera.screen_to_world(macroquad::prelude::vec2(
            mouse_screen_pos.0,
            mouse_screen_pos.1,
        ));

        state.get_mut(key2).unwrap().pos = vec2(mouse_pos.x, mouse_pos.y);

        let collisions: HashSet<RigidBodyID> = state
            .collisions()
            .iter()
            .flat_map(|(a, b)| [*a, *b])
            .collect();

        for (body_id, body) in state.iter() {
            let color = if collisions.contains(&body_id) {
                RED
            } else {
                WHITE
            };

            let shape = body.oriented_shape();
            for segment in shape.segments() {
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

        next_frame().await
    }
}

fn main() {
    macroquad::Window::from_config(Conf::default(), amain());
}
