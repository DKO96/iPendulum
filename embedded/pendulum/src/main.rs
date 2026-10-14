use plotly::{Plot, Scatter};
use std::f64::consts::PI;

const CPR: f64 =  4096.0;
const POLES: f64 =  11.0;
const OFFSET: f64 =  0.0;

fn wrap_2pi(angle: f64) -> f64 {
    let mut x: f64 = angle % (2.0 * PI);
    if x < 0.0 {
        x += 2.0 * PI;
    }
    return x;
}

fn main() {
    let v_d: f64 = 0.0;
    let v_q: f64 = 4.4;

    let mut ticks_v = Vec::new();
    let mut a_v = Vec::new();
    let mut b_v = Vec::new();
    let mut c_v = Vec::new();

    for ticks in 0..8192 {
        let theta_m: f64= ticks as f64 * 2.0 * PI / CPR;
        let mut theta_e: f64= POLES * (theta_m - OFFSET);

        theta_e = wrap_2pi(theta_e);

        let v_alpha = v_d * f64::cos(theta_e) - v_q * f64::sin(theta_e);
        let v_beta = v_d * f64::sin(theta_e) + v_q * f64::cos(theta_e);

        let v_a: f64 = v_alpha;
        let v_b: f64 = (-v_alpha + f64::sqrt(3.0) * v_beta) / 2.0;
        let v_c: f64 = (-v_alpha - f64::sqrt(3.0) * v_beta) / 2.0;

        let v_max: f64 = v_a.max(v_b).max(v_c);
        let v_min: f64 = v_a.min(v_b).min(v_c);
        let v_off: f64 = -(v_max + v_min) / 2.0;

        let v_a_out = v_a + v_off;
        let v_b_out = v_b + v_off;
        let v_c_out = v_c + v_off;

        ticks_v.push(ticks);
        a_v.push(v_a_out);
        b_v.push(v_b_out);
        c_v.push(v_c_out);
    }

    let mut plot = Plot::new();
    plot.add_trace(Scatter::new(ticks_v.clone(), a_v).name("v_a"));
    plot.add_trace(Scatter::new(ticks_v.clone(), b_v).name("v_b"));
    plot.add_trace(Scatter::new(ticks_v.clone(), c_v).name("v_c"));
    plot.write_html("phases.html");


}
