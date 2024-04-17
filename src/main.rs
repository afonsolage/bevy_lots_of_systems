//! A simple 3D scene with light shining over a cube sitting on a plane.

use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use rand::random;
use repeat_macro::simulations;
use std::f32::consts::PI;

fn main() {
    let mut app = App::new();
    app.add_plugins((DefaultPlugins, RapierPhysicsPlugin::<NoUserData>::default()))
        .add_systems(Startup, setup_graphics);
    app.add_plugins((LogDiagnosticsPlugin::default(), FrameTimeDiagnosticsPlugin));
    simulations!(app);
    app.run();
}

fn setup_graphics(mut commands: Commands) {
    // Add a camera to see the render.
    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-3.0, 3.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..Default::default()
    });
}

fn setup_physics<const T: usize>(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    fn spawn(
        commands: &mut Commands,
        meshes: &mut Assets<Mesh>,
        materials: &mut Assets<StandardMaterial>,
        n: usize,
    ) -> (Entity, Entity) {
        /* Create the ground. */
        let offset = 10.0 * (n as f32);
        let ground = commands
            .spawn(PbrBundle {
                mesh: meshes.add(Plane3d::default().mesh().size(10.0, 10.0)),
                material: materials.add(Color::WHITE),
                transform: Transform::from_xyz(0.0 + offset, 0.0, 0.0).with_rotation(
                    Quat::from_euler(
                        EulerRot::XYZ,
                        (random::<f32>() - 0.5) * 5.0 * PI / 180.0,
                        0.0,
                        (random::<f32>() - 0.5) * 5.0 * PI / 180.0,
                    ),
                ),
                ..Default::default()
            })
            .insert(Name::new(format!("Board{}", n)))
            .insert(RigidBody::KinematicPositionBased)
            .insert(LockedAxes::TRANSLATION_LOCKED | LockedAxes::ROTATION_LOCKED_Y)
            .insert(Collider::cuboid(5.0, 0.1, 5.0))
            .insert(IsBoard)
            .id();

        /* Create the bouncing ball. */
        let ball = commands
            .spawn(PbrBundle {
                mesh: meshes.add(Sphere::new(0.5).mesh()),
                material: materials.add(Color::rgb_u8(124, 144, 255)),
                transform: Transform::from_xyz(0.0 + offset, 4.5, 0.0),
                ..default()
            })
            .insert(Name::new(format!("Ball{}", n)))
            .insert(RigidBody::Dynamic)
            .insert(Velocity::default())
            .insert(Collider::ball(0.5))
            .insert(Restitution::coefficient(0.7))
            .id();

        (ground, ball)
    }

    let (ground, ball) = spawn(&mut commands, &mut meshes, &mut materials, T);
    commands.entity(ground).insert(Simulation::<T>);
    commands.entity(ball).insert(Simulation::<T>);
}

/// Determines if ball falls below threshold and environment must be reset
fn must_reset<const T: usize>(
    query: Query<&Transform, (With<Simulation<T>>, With<Restitution>)>,
) -> bool {
    let transform = query.single();
    transform.translation.y < -2.0
}

/// Resets ball position and changes board to random angle
fn reset_simulation<const T: usize>(
    mut query: Query<(&mut Transform, &RigidBody, Option<&mut Velocity>), With<Simulation<T>>>,
) {
    fn reset(transform: &mut Transform, maybe_ball: Option<&mut Velocity>, n: usize) {
        if let Some(ball_vel) = maybe_ball {
            // Current entity is the ball, reset height
            transform.translation = Vec3::new(0.0 + 10.0 * (n as f32), 4.5, 0.0);
            ball_vel.linvel = Vec3::new(0.0, 0.0, 0.0);
            ball_vel.angvel = Vec3::new(0.0, 0.0, 0.0);
        } else {
            transform.rotation = Quat::from_euler(
                EulerRot::XYZ,
                (random::<f32>() - 0.5) * 5.0 * PI / 180.0,
                0.0,
                (random::<f32>() - 0.5) * 5.0 * PI / 180.0,
            );
        }
    }

    for (mut transform, _rigid_body, mut is_ball) in query.iter_mut() {
        reset(&mut transform, is_ball.as_deref_mut(), T);
    }
}

/// Randomly moves board as environment "action"
fn board_movement<const T: usize>(
    mut query: Query<&mut Transform, (With<Simulation<T>>, With<IsBoard>)>,
) {
    fn move_board(transform: &mut Transform) {
        let mut rotation = Vec3::new(0.0, 0.0, 0.0);
        let input_dir: i8 = random::<i8>() % 4;
        if input_dir == 0 && transform.rotation.x < 0.1 {
            rotation.x = 1.0;
        }
        if input_dir == 1 && transform.rotation.x > -0.1 {
            rotation.x = -1.0;
        }
        if input_dir == 2 && transform.rotation.z > -0.1 {
            rotation.z = -1.0;
        }
        if input_dir == 3 && transform.rotation.z < 0.1 {
            rotation.z = 1.0;
        }
        rotation *= PI / 180.0;
        transform.rotate(Quat::from_euler(
            EulerRot::XYZ,
            rotation.x,
            rotation.y,
            rotation.z,
        ));
    }
    let mut transform = query.single_mut();
    move_board(&mut transform);
}

/// Tag component indicating if entity is a board
#[derive(Component)]
struct IsBoard;

/// Marker tag component indicating which environment entities are part of
#[derive(Component)]
#[component(storage = "SparseSet")]
struct Simulation<const T: usize>;
