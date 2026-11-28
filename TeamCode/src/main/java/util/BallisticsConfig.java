package util;

import com.bylazar.configurables.annotations.Configurable;

/**
 * BallisticsConfig — Centralized live-tunable parameters for projectile ballistics.
 *
 * <h3>How to use with FTC Dashboard / Panels</h3>
 * All fields are {@code public static} and <b>non-{@code final}</b> so they appear
 * as editable entries when the class is annotated with {@code @Configurable}. Every field is read
 * fresh each control-loop cycle by {@link BallisticsCalculator} — nothing is cached
 * at construction time.
 */
@Configurable
public class BallisticsConfig {

    // ═══════════════════════════════════════════════════════════════════════════
    //  §1  Gravity & Air-Resistance Compensation
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Gravitational acceleration in inches/s².
     * Standard Earth gravity ≈ 386.09 in/s²  (9.80665 m/s² × 39.3701 in/m).
     */
    public static double GRAVITY_IN_S2 = 386.09;

    /**
     * Empirical velocity-loss / drag compensation factor (≥ 1.0).
     * <p>Multiplied into the computed exit speed to account for aerodynamic drag
     * and energy losses the ideal parabolic model ignores.
     * <ul>
     *   <li>1.00 = no compensation (vacuum trajectory)</li>
     *   <li>1.05 = 5 % velocity over-shoot to fight drag</li>
     * </ul>
     * Clamp: internally forced to [1.0, 2.0].
     */
    public static double DRAG_COMPENSATION_FACTOR = 1.00;

    // ═══════════════════════════════════════════════════════════════════════════
    //  §2  Target & Launcher Geometry
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Vertical height of the goal opening center <b>above the floor</b> (inches).
     * Measure from tile surface to the center of the HIVE goal entry.
     */
    public static double TARGET_HEIGHT_IN = 36.0;

    /**
     * Vertical height of the shooter release point <b>above the floor</b> (inches).
     * Measure from tile surface to the point where the game element leaves the hood.
     */
    public static double LAUNCH_HEIGHT_IN = 10.0;

    /**
     * Depth offset (inches) added to the horizontal distance so the trajectory
     * apex / landing point sits inside the goal bucket rather than right at the lip.
     */
    public static double TARGET_DEPTH_OFFSET_IN = 3.0;

    // ═══════════════════════════════════════════════════════════════════════════
    //  §3  Launch Angle (Fixed Hood)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Fixed mechanical launch angle in <b>degrees above horizontal</b>.
     * For an adjustable hood, write an additional control loop that sets this
     * value each cycle before the ballistics calculation runs.
     */
    public static double LAUNCH_ANGLE_DEG = 45.0;

    /** Minimum allowable launch angle (degrees).  Prevents nonsensical flat shots. */
    public static double LAUNCH_ANGLE_MIN_DEG = 10.0;

    /** Maximum allowable launch angle (degrees).  Prevents nearly vertical lobs. */
    public static double LAUNCH_ANGLE_MAX_DEG = 80.0;

    // ═══════════════════════════════════════════════════════════════════════════
    //  §4  Distance Calibration
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Additive offset (inches) applied to the raw Euclidean distance from the
     * robot / turret pose to the goal.  Positive = act as if the goal is farther.
     * Use to compensate for shooter position not being at the robot center, or
     * systematic localizer bias.
     */
    public static double DISTANCE_OFFSET_IN = 0.0;

    /**
     * Multiplicative scale factor applied to (rawDistance + DISTANCE_OFFSET_IN).
     * <ul>
     *   <li>1.00 = no scale</li>
     *   <li>1.02 = act 2 % farther (compensates for localizer under-reading)</li>
     * </ul>
     * Clamp: internally forced to [0.5, 2.0].
     */
    public static double DISTANCE_SCALE_FACTOR = 1.0;

    // ═══════════════════════════════════════════════════════════════════════════
    //  §5  Flywheel → Projectile Exit-Speed Conversion
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Outer circumference of the flywheel in <b>inches</b>.
     * <p>Formula: {@code Math.PI * diameter_in}.
     * Default: π × 2.75″ (compliant wheel) ≈ 8.639″.
     */
    public static double WHEEL_CIRCUMFERENCE_IN = Math.PI * 2.75;

    /**
     * Launch efficiency / slip factor  (0 &lt; η ≤ 1.0).
     * <p>Fraction of flywheel surface speed that is transferred to the game element.
     * For a single-wheel hood shooter, typical range is 0.45–0.60.
     * <ul>
     *   <li>0.50 = 50 % of wheel surface speed becomes projectile speed</li>
     * </ul>
     * Clamp: internally forced to [0.05, 1.0].
     */
    public static double LAUNCH_EFFICIENCY = 0.50;

    /**
     * Empirical RPM multiplier applied <b>after</b> the kinematic RPM is
     * computed.  Used by the tuning OpMode to dial in a scale correction that
     * accounts for systematic model error.
     * <p>Default 1.0 = no correction.
     */
    public static double TUNING_RPM_MULTIPLIER = 1.0;

    /**
     * Empirical RPM offset applied <b>after</b> the multiplier.
     * <p>Positive adds RPM; negative subtracts.
     */
    public static double TUNING_RPM_OFFSET = 0.0;

    // ── Velocity clamps (ticks/sec) ──────────────────────────────────────────

    /**
     * Minimum shooter velocity in <b>ticks/sec</b> that the ballistics calculator
     * will ever return.  Acts as a safety floor so the flywheel never stalls on
     * short-range shots.
     */
    public static double MIN_VELOCITY_TPS = 500.0;

    /**
     * Maximum shooter velocity in <b>ticks/sec</b>.  Prevents commanding speeds
     * the motor cannot physically reach.
     */
    public static double MAX_VELOCITY_TPS = 3500.0;

    // ── Encoder constant (copied from TurretShooterSubsystem for convenience) ─

    /** Encoder ticks per motor revolution.  Must match the shooter motor encoder. */
    public static double ENCODER_TICKS_PER_REV = 28.0;

    // ═══════════════════════════════════════════════════════════════════════════
    //  §6  Fallback Defaults
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Fallback velocity (ticks/sec) returned when the ballistics model cannot
     * produce a valid result (impossible geometry, NaN, etc.).
     * <p>This should match {@code TurretShooterSubsystem.outtakeVelocityShort}
     * so behavior degrades gracefully to the old hardcoded default.
     */
    public static double FALLBACK_VELOCITY_TPS = 2193.33;
}
