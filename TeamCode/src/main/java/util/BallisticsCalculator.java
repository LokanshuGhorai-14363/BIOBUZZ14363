package util;

/**
 * BallisticsCalculator — Pure-math projectile kinematics for the BIOBUZZ shooter.
 *
 * <p>All tunable physical parameters live in {@link BallisticsConfig} as
 * {@code public static} non-{@code final} fields and are read <b>fresh every
 * call</b> so they can be hot-tuned from FTC Dashboard / Panels without
 * recompiling.
 *
 * <h3>Coordinate System &amp; Units</h3>
 * <pre>
 *   Distance / height → inches
 *   Angles            → degrees (converted to radians internally)
 *   Gravity           → inches / s²
 *   Velocity output   → ticks / s  (compatible with {@code TurretShooterSubsystem.setTargetVelocity})
 * </pre>
 *
 * <h3>Physics Model (2-D Parabolic Trajectory, No Drag)</h3>
 * <pre>
 *   x(t) = v₀ · cos(θ) · t
 *   y(t) = v₀ · sin(θ) · t  −  ½ g t²
 *
 *   Solving for v₀ when x = D (horizontal range) and y = Δh (height delta):
 *
 *       v₀ = (D / cos θ) · √( g / [2 (D tan θ − Δh)] )
 *
 *   where Δh = targetHeight − launchHeight.
 * </pre>
 *
 * <h3>Exit-Speed → Flywheel RPM → Ticks/sec Pipeline</h3>
 * <pre>
 *   v_exit  = v₀ · dragCompensation
 *   v_wheel = v_exit / launchEfficiency
 *   RPM     = (v_wheel · 60) / wheelCircumference
 *   RPM_tuned = RPM · tuningMultiplier + tuningOffset
 *   tps     = (RPM_tuned / 60) · encoderTicksPerRev
 *   output  = clamp(tps, minVelocity, maxVelocity)
 * </pre>
 *
 * This class is <b>stateless</b> — every public method is {@code static} and
 * reads {@link BallisticsConfig} fields on every invocation.
 */
public final class BallisticsCalculator {

    // Private constructor — utility class, no instances.
    private BallisticsCalculator() {}

    // ═══════════════════════════════════════════════════════════════════════════
    //  Primary API
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Computes the shooter target velocity in <b>ticks/sec</b> for a given
     * horizontal distance to the goal.
     *
     * <p>The distance is first adjusted by the calibration offset and scale
     * factor from {@link BallisticsConfig}, then the full kinematic pipeline
     * runs.  If the trajectory is physically impossible (e.g., the goal is
     * higher than the parabola can reach at this angle/distance), the method
     * returns {@link BallisticsConfig#FALLBACK_VELOCITY_TPS}.
     *
     * @param rawDistanceIn raw Euclidean distance from the robot/turret to the
     *                      goal, in inches (as reported by the localizer).
     *                      Must be ≥ 0; negative values are clamped to 0.
     * @return target flywheel velocity in ticks/sec, clamped to
     *         [{@link BallisticsConfig#MIN_VELOCITY_TPS},
     *          {@link BallisticsConfig#MAX_VELOCITY_TPS}], or the fallback
     *         value on model failure.
     */
    public static double computeTargetVelocityTps(double rawDistanceIn) {
        // ── 1. Sanitise & calibrate the distance ─────────────────────────────
        double distance = Math.max(0.0, rawDistanceIn);
        distance += BallisticsConfig.DISTANCE_OFFSET_IN;
        distance *= clamp(BallisticsConfig.DISTANCE_SCALE_FACTOR, 0.5, 2.0);
        distance += BallisticsConfig.TARGET_DEPTH_OFFSET_IN;
        distance = Math.max(1.0, distance); // prevent division by zero

        // ── 2. Compute the exit speed from projectile kinematics ─────────────
        double exitSpeedInPerSec = computeExitSpeed(distance);
        if (Double.isNaN(exitSpeedInPerSec) || exitSpeedInPerSec <= 0.0) {
            return BallisticsConfig.FALLBACK_VELOCITY_TPS;
        }

        // ── 3. Convert exit speed → flywheel RPM → ticks/sec ────────────────
        double tps = exitSpeedToTicksPerSec(exitSpeedInPerSec);
        if (Double.isNaN(tps) || tps <= 0.0) {
            return BallisticsConfig.FALLBACK_VELOCITY_TPS;
        }

        return clamp(tps, BallisticsConfig.MIN_VELOCITY_TPS, BallisticsConfig.MAX_VELOCITY_TPS);
    }

    /**
     * Convenience wrapper that returns the target velocity in <b>RPM</b> instead
     * of ticks/sec.
     *
     * @param rawDistanceIn raw horizontal distance to goal (inches)
     * @return target flywheel RPM
     */
    public static double computeTargetRPM(double rawDistanceIn) {
        double tps = computeTargetVelocityTps(rawDistanceIn);
        return ticksPerSecToRPM(tps);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Kinematic Core
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Solves the 2-D parabolic trajectory equation for the required projectile
     * exit speed (inches/sec).
     *
     * <pre>
     *   v₀ = (D / cos θ) · √( g / [2 · (D · tan θ  −  Δh)] )
     * </pre>
     *
     * @param horizontalDistanceIn calibrated horizontal distance (inches), must be &gt; 0
     * @return required projectile exit speed in inches/sec, or {@code NaN} if
     *         the trajectory is impossible at the current angle.
     */
    static double computeExitSpeed(double horizontalDistanceIn) {
        // Read live-tunable config values every call — never cache across cycles
        double g      = Math.max(1.0, BallisticsConfig.GRAVITY_IN_S2);
        double dh     = BallisticsConfig.TARGET_HEIGHT_IN - BallisticsConfig.LAUNCH_HEIGHT_IN;
        double drag   = clamp(BallisticsConfig.DRAG_COMPENSATION_FACTOR, 1.0, 2.0);

        // Clamp launch angle to configured bounds, then convert to radians
        double angleDeg = clamp(
                BallisticsConfig.LAUNCH_ANGLE_DEG,
                BallisticsConfig.LAUNCH_ANGLE_MIN_DEG,
                BallisticsConfig.LAUNCH_ANGLE_MAX_DEG);
        double thetaRad = Math.toRadians(angleDeg);

        double cosTheta = Math.cos(thetaRad);
        double tanTheta = Math.tan(thetaRad);

        // Guard: near-vertical angle makes cos(θ) ≈ 0 → division by zero
        if (Math.abs(cosTheta) < 1.0e-6) {
            return Double.NaN;
        }

        double D = horizontalDistanceIn; // alias for readability

        // Denominator of the square-root argument:
        //   2 · (D · tan(θ) − Δh)
        // This must be > 0 for a real (non-imaginary) exit speed.
        // If Δh > D·tan(θ), the projectile cannot reach the target height at
        // this angle and distance — the shot is geometrically impossible.
        double denom = 2.0 * (D * tanTheta - dh);
        if (denom <= 0.0) {
            return Double.NaN; // impossible trajectory
        }

        // v₀ = (D / cosθ) · √(g / denom)
        double v0 = (D / cosTheta) * Math.sqrt(g / denom);

        // Apply drag compensation multiplier
        return v0 * drag;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Exit-Speed → Ticks/Sec Conversion
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Converts a required projectile exit speed (inches/sec) into the
     * corresponding flywheel target velocity in ticks/sec, applying the
     * efficiency factor, tuning multiplier / offset, and unit conversions.
     *
     * @param exitSpeedInPerSec projectile exit speed in inches/sec
     * @return flywheel velocity in encoder ticks/sec
     */
    static double exitSpeedToTicksPerSec(double exitSpeedInPerSec) {
        // Flywheel surface speed required (inches/sec)
        double efficiency = clamp(BallisticsConfig.LAUNCH_EFFICIENCY, 0.05, 1.0);
        double wheelSurfaceSpeed = exitSpeedInPerSec / efficiency;

        // Wheel circumference (inches) — guard against zero
        double circumference = Math.max(0.1, BallisticsConfig.WHEEL_CIRCUMFERENCE_IN);

        // Surface speed → RPM:  RPM = (v_surface × 60) / circumference
        double rpm = (wheelSurfaceSpeed * 60.0) / circumference;

        // Apply empirical tuning multiplier & offset
        rpm = rpm * BallisticsConfig.TUNING_RPM_MULTIPLIER + BallisticsConfig.TUNING_RPM_OFFSET;

        // RPM → ticks/sec:  tps = (RPM / 60) × ticksPerRev
        double ticksPerRev = Math.max(1.0, BallisticsConfig.ENCODER_TICKS_PER_REV);
        return (rpm / 60.0) * ticksPerRev;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Unit-Conversion Helpers
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Converts ticks/sec → RPM using the live encoder ticks-per-rev config.
     *
     * @param tps velocity in encoder ticks per second
     * @return velocity in revolutions per minute
     */
    public static double ticksPerSecToRPM(double tps) {
        double ticksPerRev = Math.max(1.0, BallisticsConfig.ENCODER_TICKS_PER_REV);
        return (tps / ticksPerRev) * 60.0;
    }

    /**
     * Converts RPM → ticks/sec using the live encoder ticks-per-rev config.
     *
     * @param rpm velocity in revolutions per minute
     * @return velocity in encoder ticks per second
     */
    public static double rpmToTicksPerSec(double rpm) {
        double ticksPerRev = Math.max(1.0, BallisticsConfig.ENCODER_TICKS_PER_REV);
        return (rpm / 60.0) * ticksPerRev;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Internal Helpers
    // ═══════════════════════════════════════════════════════════════════════════

    /** Clamps {@code value} to [{@code min}, {@code max}]. */
    static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
