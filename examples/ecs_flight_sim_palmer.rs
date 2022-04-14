use std::{process};

use gphys::palmer::fdm;  // Grant Palmer's flight dynamics model
use hecs;                // ECS
use device_query::{DeviceQuery, DeviceState, Keycode};  // used to query keyboard
use console::Term;       // used to print output

fn spawn_entity(world: &mut hecs::World) -> hecs::Entity
{
    // Cessna 172 properties
    let prop = fdm::Cessna172Properties {
        wing_area: 16.2,
        wing_span: 10.9,
        tail_area: 2.0,
        cl_slope0: 0.0889,      // slope of Cl-alpha curve
        cl0: 0.178,             // intercept of Cl-alpha curve
        cl_slope1: -0.1,        // post-stall slope of Cl-alpha curve
        cl1: 3.2,               // post-stall intercept of Cl-alpha curve
        alpha_cl_max: 16.0,     // alpha when Cl=Clmax
        cdp: 0.034,             // parasite drag coefficient
        eff: 0.77,              // induced drag efficiency coefficient
        mass: 1114.0,
        engine_power: 119310.0,
        engine_rps: 40.0,       // revolutions per second
        prop_diameter: 1.905,
        a: 1.83,                //  propeller efficiency coefficient
        b: -1.32,               //  propeller efficiency coefficient
    };

    // simulation data (inputs/outputs)
    let ctrl_inputs = fdm::CtrlInputs {
        bank: 0.0,         // roll angle
        alpha: 4.0,        // pitch angle
        throttle: 0.0,     // throttle percentage
        flap: 0.0,         // flap deflection
    };

    // simulation data (inputs/outputs)
    let rk4_data = fdm::Rk4Data {
        num_eqns: 6,
        q: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    };
    
    world.spawn((prop, ctrl_inputs, rk4_data))
}

fn sys_input(world: &mut hecs::World, device_state: &DeviceState, prev_keys: Vec<Keycode>) -> Vec<Keycode>
{
    let keys: Vec<Keycode> = device_state.get_keys();
    if keys != prev_keys {
        for (_id, mut inputs) in &mut world.query::<&mut fdm::CtrlInputs>() {
            if keys.contains(&Keycode::E) {
                inputs.throttle += 0.1;
            } else if keys.contains(&Keycode::D) {
                inputs.throttle -= 0.1;
            } else if keys.contains(&Keycode::Up) {
                inputs.alpha += 1.0;
            } else if keys.contains(&Keycode::Down) {
                inputs.alpha -= 1.0;
            } else if keys.contains(&Keycode::Left) {
                inputs.bank -= 1.0;
            } else if keys.contains(&Keycode::Right) {
                inputs.bank += 1.0;
            } else if keys.contains(&Keycode::L) {
                inputs.flap -= 1.0;
            } else if keys.contains(&Keycode::K) {
                inputs.flap += 1.0;
            } else if keys.contains(&Keycode::Q) {
                process::exit(0);
            }
        }
    }
    keys
}

fn sys_integrate_motion(world: &mut hecs::World, dt: f64) {
    for (_id, (prop, ctrl_inputs, mut rk4_data)) in &mut world.query::<(&fdm::Cessna172Properties, &fdm::CtrlInputs, &mut fdm::Rk4Data)>() {
        fdm::eom_rk4(&prop, &ctrl_inputs, &mut rk4_data, dt);
    }
}

fn sys_print_display(world: &hecs::World, term: &Term) {
    for (_id, (input, rk4_data)) in &mut world.query::<(&fdm::CtrlInputs, &fdm::Rk4Data)>() {
        // velocity information
        let vx = rk4_data.q[0];
        let vy = rk4_data.q[2];
        let vz = rk4_data.q[4];
        // position information
        let x = rk4_data.q[1];
        let y = rk4_data.q[3];
        let z = rk4_data.q[5];

        let heading = vy.atan2(vx).to_degrees();
        let vh: f32 = ((vx * vx + vy * vy).sqrt()) as f32;
        let climb_angle = (vz as f32 / vh as f32).atan().to_degrees();
        let air_speed = (vx * vx + vy * vy + vz * vz).sqrt();
    
        let s = format!("Throttle: {:>3.1}% | Ang of Atk: {:>3.1} deg | Bank: {:>3.1} deg | Flap Defl: {:>3.1} deg | Heading: {:>3.1} deg | Climb: {:>3.1} deg | Air Speed: {:>3.1} | Climb Rate: {:>3.1} | Altitude: {:>3.1} | X : {:>3.1} | Y : {:>3.1} | ",
                        (input.throttle * 100.0).round(),
                        input.alpha,
                        input.bank,
                        input.flap,
                        heading,
                        climb_angle,
                        air_speed,
                        vz,
                        z,
                        x,
                        y);
        term.clear_line().unwrap();
        term.write_str(&s).unwrap();
    }
}

fn main() {
    // create world and spawn an ownship entity
    let mut world = hecs::World::new();
    let _ownship = spawn_entity(&mut world);

    let device_state = device_query::DeviceState::new();  // device state
    let mut prev_keys: Vec<Keycode> = vec![];             // previous keys

    let term = Term::stdout();

    const DT: f64 = 0.5;

    loop {
        // execute all systems
        prev_keys = sys_input(&mut world, &device_state, prev_keys);  // read keyboard inputs
        sys_integrate_motion(&mut world, DT);                         // update flight dynamics model
        sys_print_display(&world, &term);                             // display output
    }
}
