use std::f64::consts::PI;
static G: f64 = -9.81;

#[derive(Debug, Default)]
pub struct Cessna172Properties
{
    pub wing_area: f64,
    pub wing_span: f64,
    pub tail_area: f64,
    pub cl_slope0: f64,      // slope of Cl-alpha curve
    pub cl0: f64,            // intercept of Cl-alpha curve
    pub cl_slope1: f64,      // post-stall slope of Cl-alpha curve
    pub cl1: f64,            // post-stall intercept of Cl-alpha curve
    pub alpha_cl_max: f64,   // alpha when Cl=Clmax
    pub cdp: f64,            // parasite drag coefficient
    pub eff: f64,            // induced drag efficiency coefficient
    pub mass: f64,
    pub engine_power: f64,
    pub engine_rps: f64,     // revolutions per second
    pub prop_diameter: f64,
    pub a: f64,              //  propeller efficiency coefficient
    pub b: f64,              //  propeller efficiency coefficient
}

#[derive(Debug, Default)]
pub struct CtrlInputs
{
    pub bank: f64,         // roll angle
    pub alpha: f64,        // pitch angle
    pub throttle: f64,     // throttle percentage
    pub flap: f64,         // flap deflection
}

#[derive(Debug, Default)]
pub struct Rk4Data
{
    pub num_eqns: i32,
    pub q: [f64;6],        // vx, x, vy, y, vz, z    
}

fn calculate_forces(prop: &Cessna172Properties, inputs: &CtrlInputs, altitude: f64, velocity: f64) -> (f64, f64, f64, f64)
{
    // compute the air density
    let temperature: f64 = 288.15 - 0.0065 * altitude;
    let grp: f64 = 1.0 - 0.0065 * altitude / 288.15;
    let pressure: f64 = 101325.0 * (grp.powf(5.25));
    let density: f64 = 0.00348 * pressure / temperature;
    
    // compute power drop-off factor
    let omega: f64 = density / 1.225;
    let factor: f64 = (omega - 0.12)/  0.88;
    
    // compute thrust 
    let advance_ratio: f64 = velocity / (prop.engine_rps * prop.prop_diameter);
    let thrust: f64 = inputs.throttle * factor * prop.engine_power * (prop.a + prop.b * advance_ratio * advance_ratio) / (prop.engine_rps * prop.prop_diameter);
    
    // compute lift coefficient - the Cl curve is modeled using two straight lines
    let mut cl: f64;
    if inputs.alpha < prop.alpha_cl_max {
        cl = prop.cl_slope0 * inputs.alpha + prop.cl0;
    } else {
        cl = prop.cl_slope1 * inputs.alpha + prop.cl1;
    }
    
    // include effects of flaps and ground effects
    // -- ground effects are present if the plane is within 5 meters of the ground
    if inputs.flap == 20.0 {
        cl += 0.25;
    }
    if inputs.flap == 40.0 {
        cl += 0.5;
    }
    if altitude < 5.0 {
        cl += 0.25;
    }
    
    // compute lift
    let lift: f64 = 0.5 * cl * density * velocity * velocity * prop.wing_area;
    
    // compute drag coefficient
    let aspect_ratio: f64 = prop.wing_span * prop.wing_span / prop.wing_area;
    let cd = prop.cdp + cl * cl / (PI * aspect_ratio * prop.eff);
    
    // compute drag force
    let drag: f64 = 0.5 * cd * density * velocity * velocity * prop.wing_area;

    // add the gravity force to the z-direction force.
    let mut gravity: f64 = prop.mass * G;

    // since the plane can't sink into the ground, if the altitude is less than or equal to zero and the z-component
    // of force is less than zero, set the z-force to be zero
    if altitude <= 0.0 && gravity <= 0.0 {
        gravity = 0.0;
    }
    
    (thrust, lift, drag, gravity)
}

//-----------------------------------------------------
// calculates the forces associated with an aircraft
// given a set of properties and current state
//-----------------------------------------------------
fn plane_rhs(prop: &Cessna172Properties,
             inputs: &CtrlInputs,
             q: &[f64; 6], delta_q: &[f64; 6],
             dt: f64, q_scale: f64,
             dq: &mut [f64; 6])
{
    // compute the intermediate values of the dependent variables
    let mut new_q : [f64; 6] = [0.0; 6];
    for i in 0..6 {
        new_q[i] = q[i] + q_scale * delta_q[i]; 
    }

    // assign convenenience variables to the intermediate
    // values of the locations and velocities
    let vx: f64 = new_q[0];
    let vy: f64 = new_q[2];
    let vz: f64 = new_q[4];
    let _x: f64 = new_q[1];
    let _y: f64 = new_q[3];
    let z: f64 = new_q[5];
    let vh: f64 = (vx * vx + vy * vy).sqrt();
    let velocity: f64 = (vx * vx + vy * vy + vz * vz).sqrt();

    let (thrust, lift, drag, gravity) = calculate_forces(&prop, &inputs, z, velocity);

    // convert bank angle from degrees to radians
    // angle of attack is not converted because the
    // Cl-alpha curve is defined in terms of degrees
    let bank = inputs.bank.to_radians();

    // define some shorthand convenience variables for use with the rotation matrix
    // compute the sine and cosines of the climb angle, bank angle, and heading angle

    let cos_w: f64 = bank.cos(); 
    let sin_w: f64 = bank.sin(); 

    let cos_p: f64;   //  climb angle
    let sin_p: f64;   //  climb angle
    let cos_t: f64;   //  heading angle
    let sin_t: f64;   //  heading angle
    if velocity == 0.0 {
        cos_p = 1.0;
        sin_p = 0.0;
    } else {
        cos_p = vh / velocity;  
        sin_p = vz / velocity;  
    }

    if vh == 0.0 {
        cos_t = 1.0;
        sin_t = 0.0;
    } else {
        cos_t = vx / vh;
        sin_t = vy / vh;
    }

    // convert the thrust, drag, and lift forces into x-, y-, and z-components using the rotation matrix
    let fx: f64 = cos_t * cos_p * (thrust - drag) + (sin_t * sin_w - cos_t * sin_p * cos_w) * lift;
    let fy: f64 = sin_t * cos_p * (thrust - drag) + (-cos_t * sin_w - sin_t * sin_p * cos_w) * lift;
    let mut fz: f64 = sin_p * (thrust - drag) + cos_p * cos_w * lift;

    // add the gravity force to the z-direction force.
    fz += prop.mass * G;

    // since the plane can't sink into the ground, if the altitude is less than or equal to zero and the z-component
    // of force is less than zero, set the z-force to be zero
    if z <= 0.0 && gravity <= 0.0 {
        fz = 0.0;
    }

    // load the right-hand sides of the ODE's
    dq[0] = dt * (fx / prop.mass);
    dq[1] = dt * vx;
    dq[2] = dt * (fy / prop.mass);
    dq[3] = dt * vy;
    dq[4] = dt * (fz / prop.mass);
    dq[5] = dt * vz;
}

//-----------------------------------------------------
// solves the equations of motion using the Runge-Kutta
// integration method
//-----------------------------------------------------
pub fn eom_rk4(prop: &Cessna172Properties, ctrl_inputs: &CtrlInputs, rk4_data: &mut Rk4Data, dt: f64)
{
    //let num_eqns = rk4_data.num_eqns;

    let mut q : [f64; 6] = [0.0; 6];
    let mut dq1 : [f64; 6] = [0.0; 6];
    let mut dq2 : [f64; 6] = [0.0; 6];
    let mut dq3 : [f64; 6] = [0.0; 6];
    let mut dq4 : [f64; 6] = [0.0; 6];

    // retrieve the current values of the dependent and independent variables
    for j in 0..6 {
        q[j] = rk4_data.q[j];
    }

    // compute the four Runge-Kutta steps, then return 
    // value of planeRightHandSide method is an array
    // of delta-q values for each of the four steps
    plane_rhs(&prop, &ctrl_inputs, &q, &q, dt, 0.0, &mut dq1);
    plane_rhs(&prop, &ctrl_inputs, &q, &dq1, dt, 0.5, &mut dq2);
    plane_rhs(&prop, &ctrl_inputs, &q, &dq2, dt, 0.5, &mut dq3);
    plane_rhs(&prop, &ctrl_inputs, &q, &dq3, dt, 1.0, &mut dq4);

    // update the dependent and independent variable values
    // at the new dependent variable location and store the
    // values in the ODE object arrays
    for i in 0..6 {
        q[i] = q[i] + (dq1[i] + 2.0 * dq2[i] + 2.0 * dq3[i] + dq4[i]) / 6.0;
        rk4_data.q[i] = q[i];
    }
}
