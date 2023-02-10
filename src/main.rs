use std::{ops::Range, iter::{zip}, fs::File, path::Path, fmt::Write};

use indicatif::{ProgressBar, ProgressStyle, ProgressState};
use rand::{Rng};
use rapier3d::{prelude::*, na::Vector3};

use rmp_serde::Serializer;
use serde_derive::{Serialize, Deserialize};

fn main() {
    let params = SimulationParams {
        gravitational_constant: 1.0,
        planet_amount: 10000,
        planet_spawn_area: Vector3::new(500.0, 5.0, 500.0),
        planet_mass_range: (1.0..1.0),
        planet_size_range: (1.0..1.0),
        simulation_time: 30.0,
        frames_per_second: 50,
    };

    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    // let collider = ColliderBuilder::cuboid(100.0, 0.1, 100.0).build();
    // collider_set.insert(collider);

    // /* Create the bounding ball. */
    // let rigid_body = RigidBodyBuilder::dynamic()
    //         .translation(vector![0.0, 10.0, 0.0])
    //         .build();
    // let collider = ColliderBuilder::ball(0.5).restitution(0.7).build();
    // let ball_body_handle = rigid_body_set.insert(rigid_body);
    // collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);

    let planets = random_planets(
        params.planet_amount, 
        &params.planet_spawn_area,
        &params.planet_mass_range,
        &params.planet_size_range
    );
    let body_handles: Vec<RigidBodyHandle> = planets.iter().map(|planet| { rigid_body_set.insert(planet.rigid_body.to_owned()) }).collect();
    for pair in zip(&planets, &body_handles) {
        collider_set.insert_with_parent(pair.0.collider.clone(), *pair.1, &mut rigid_body_set);
    }

    /* Create other structures necessary for the simulation. */
    let gravity = vector![0.0, 0.0, 0.0];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    // let mut query_pipeline = QueryPipeline::new();
    let physics_hooks = ();
    let event_handler = ();

    let time_steps = params.simulation_time as u32 * params.frames_per_second;
    let display_progress_bar = ProgressBar::new(time_steps.into());
    display_progress_bar
        .set_style(
            ProgressStyle::with_template("{spinner} [{elapsed_precise}] <{pos}/{len}>(percent) [{wide_bar}]  (eta: {eta_precise}) (per sec.:{per_sec})")
            .unwrap()
            .with_key("eta", |state: &ProgressState, w: &mut dyn Write| write!(w, "{:.1}s", state.eta().as_secs_f64()).unwrap())
        );

    let mut positions: Vec<Vec<Vector3<f32>>> = Vec::new();
    /* Run the game loop, stepping the simulation once per frame. */
    for _ in 0..time_steps {
        physics_pipeline.step(
        &gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut rigid_body_set,
        &mut collider_set,
        &mut impulse_joint_set,
        &mut multibody_joint_set,
        &mut ccd_solver,
        None,
        &physics_hooks,
        &event_handler,
        );

        for planet in &planets {

        }

        positions.push(
            body_handles.iter()
                .map(|handle| -> Vector3<f32> {
                    *(&rigid_body_set[*handle].translation().xyz())
                }
                )
                .collect()
            );

        // let body = &rigid_body_set[body_handles[0]];

        // let ball_body = &rigid_body_set[ball_body_handle];
        // println!(
        // "Ball altitude: {}",
        // ball_body.translation().y
        // );
        display_progress_bar.inc(1);
    }
    display_progress_bar.finish_and_clear();

    let data = SimulationData {
        params: params,
        planets: planets.iter().map(|planet| -> Planet {planet.planet}).collect(),
        positions: positions
    };
    let filepath = Path::new("./sim.msgpack");
    let file = File::create(&filepath).expect("Unable to create file");
    let mut serializer = &mut Serializer::new(file);
    serde::Serialize::serialize(&data, serializer).expect("Failed to serialize simulation data");
    println!("Saved data to {}", filepath.display())
    // data.serialize(serializer)
    // f.write_all(data.as_bytes()).expect("Unable to write data");
}


#[derive(Serialize, Deserialize)]
struct SimulationParams {
    gravitational_constant: f32,
    planet_amount: i32,
    planet_spawn_area: Vector3<f32>,
    planet_mass_range: Range<f32>,
    planet_size_range: Range<f32>,
    simulation_time: f32,
    frames_per_second: u32,
}

struct SimulatedPlanet {
    collider: Collider,
    rigid_body: RigidBody,
    planet: Planet
}

#[derive(Serialize, Deserialize)]
#[derive(Clone, Copy)]
struct Planet {
    mass: f32,
    size: f32
}

#[derive(Serialize, Deserialize)]
struct SimulationData {
    params: SimulationParams,
    planets: Vec<Planet>,
    positions: Vec<Vec<Vector3<f32>>>,
}

fn random_planets(amount: i32, spawn_area: &Vector3<f32>, mass_range: &Range<f32>, size_range: &Range<f32>) -> Vec<SimulatedPlanet>{
    let mut rng = rand::thread_rng();
    let mut rng2 = rand::thread_rng();

    (0..amount)
        .map(|_| { Planet{
            mass: rng.gen_range(mass_range.start..=mass_range.end),
            size: rng.gen_range(size_range.start..=size_range.end),
        }})
        .map(|planet| -> SimulatedPlanet { SimulatedPlanet {
            // pos: Vector3::new(
            //     rng.gen_range(-spawn_area.x..spawn_area.x),
            //     rng.gen_range(-spawn_area.y..spawn_area.y),
            //     rng.gen_range(-spawn_area.z..spawn_area.z),
            // ),
            collider: ColliderBuilder::ball(0.5)
                .restitution(0.7)
                .build(),
            rigid_body: RigidBodyBuilder::dynamic()
                .can_sleep(false)
                .ccd_enabled(true)
                .translation(vector![
                    rng2.gen_range(-spawn_area.x..=spawn_area.x),
                    rng2.gen_range(-spawn_area.y..=spawn_area.y),
                    rng2.gen_range(-spawn_area.z..=spawn_area.z)
                ])
                .build(),
            planet: planet,
        }}).collect()

}