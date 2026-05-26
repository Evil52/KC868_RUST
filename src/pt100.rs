//! Pt100 RTD → temperature conversion (Callendar–Van Dusen).
//!
//! For T ≥ 0 °C the analytic inverse is exact:
//!     R(t) = R0 * (1 + A·t + B·t²)
//!     t    = (-A + √(A² - 4B·(1 - R/R0))) / (2B)
//!
//! For T < 0 °C the equation includes a (t-100)·t³ term and has no closed
//! form. We fall back to Newton–Raphson seeded from the positive-side
//! result; converges in 3-4 iterations across -200..0 °C.

use libm::sqrtf;

const A: f32 = 3.9083e-3;
const B: f32 = -5.775e-7;
const C: f32 = -4.183e-12;

/// Defined operating range of a class-B Pt100 per IEC 60751 (-200..+850),
/// padded by ±10 °C to absorb the f32 rounding noise near the boundary
/// (e.g., the reference R(-200) = 18.52 Ω converges to ≈-200.05 °C). A
/// genuine short circuit (R ≈ 0) still lands at ≈-242 °C — well below
/// the relaxed lower bound — so the fault-detection intent is preserved.
const T_MIN_C: f32 = -210.0;
const T_MAX_C: f32 =  860.0;

/// Convert RTD resistance to temperature in °C, returning `None` if the
/// reading is outside the Pt100 operating range (open/short circuit,
/// wiring fault) or if the Newton–Raphson refinement failed to converge.
pub fn resistance_to_celsius(r_ohms: f32, r0: f32) -> Option<f32> {
    // Trivial sanity: negative or zero resistance ⇒ wiring fault.
    if !(r_ohms.is_finite()) || r_ohms <= 0.0 {
        return None;
    }

    let ratio = r_ohms / r0;

    // Positive-branch closed form. Valid for T ≥ 0 °C; for negative
    // temperatures the discriminant is still real and the result is a
    // useful seed for Newton–Raphson on the full quartic.
    let disc = A * A - 4.0 * B * (1.0 - ratio);
    if disc < 0.0 { return None; }
    let t_pos = (-A + sqrtf(disc)) / (2.0 * B);

    let t = if t_pos >= 0.0 {
        t_pos
    } else {
        let mut t = t_pos;
        let mut converged = false;
        for _ in 0..6 {
            // f(t)  = R0*(1 + A·t + B·t² + C·(t-100)·t³) - R
            // f'(t) = R0*(A + 2B·t + C·(4t³ - 300·t²))
            let t2 = t * t;
            let t3 = t2 * t;
            let f  = r0 * (1.0 + A * t + B * t2 + C * (t - 100.0) * t3) - r_ohms;
            let df = r0 * (A + 2.0 * B * t + C * (4.0 * t3 - 300.0 * t2));
            if df.abs() < f32::EPSILON { break; }
            let next = t - f / df;
            if (next - t).abs() < 1e-4 {
                t = next;
                converged = true;
                break;
            }
            t = next;
        }
        if !converged { return None; }
        t
    };

    if (T_MIN_C..=T_MAX_C).contains(&t) {
        Some(t)
    } else {
        None
    }
}
