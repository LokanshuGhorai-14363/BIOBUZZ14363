package util;

import com.pedropathing.math.Pose;
import Subsystems.VisionSubsystem.Alliance;

/**
 * HiveGoalTracker — Pure-math utility for BIOBUZZ HIVE goal aiming.
 *
 * <h3>Responsibilities</h3>
 * <ul>
 *   <li>Stores Alliance-specific Near and Far goal coordinates.</li>
 *   <li>Selects the active goal based on the robot's Y-position with hysteresis
 *       to prevent rapid flickering at the 72" threshold.</li>
 *   <li>Computes the field-relative heading angle (radians) from any robot pose
 *       to the active goal.</li>
 *   <li>Computes Euclidean distance to the active goal for dynamic launcher
 *       velocity scaling.</li>
 *   <li>Provides a PID-style heading correction output for chassis or turret
 *       alignment.</li>
 * </ul>
 *
 * <h3>Field Coordinate System (Pedro Pathing / Road Runner convention)</h3>
 * <pre>
 *   Origin (0, 0)       → bottom-left (Audience-left / Red Garden corner)
 *   X-axis              → runs left-to-right along the Audience Wall
 *   Y-axis              → runs from Audience Wall (Y = 0) to Back Wall (Y = 144)
 *   Positive heading    → counter-clockwise from +X axis (standard math convention)
 * </pre>
 *
 * <h3>HIVE Goal Coordinates (inches)</h3>
 * <pre>
 *   Red  Near Goal (Y &lt; 72"):  (60.0, 60.0)
 *   Red  Far  Goal (Y ≥ 72"):  (60.0, 84.0)
 *   Blue Near Goal (Y &lt; 72"):  (84.0, 60.0)
 *   Blue Far  Goal (Y ≥ 72"):  (84.0, 84.0)
 * </pre>
 *
 * This class is intentionally stateless with respect to hardware — it operates
 * purely on {@link Pose} data and an {@link Alliance} enum so it can be tested
 * off-robot and reused across OpModes.
 */
public class HiveGoalTracker {

    // ═══════════════════════════════════════════════════════════════════════════
    //  Field & Goal Constants
    // ═══════════════════════════════════════════════════════════════════════════

    /** Field tile area: 144 in × 144 in. */
    public static final double FIELD_SIZE_IN = 144.0;

    /** Y-coordinate threshold separating Near and Far goals. */
    public static final double MID_FIELD_Y = 72.0;

    /**
     * Hysteresis half-width (inches) around {@link #MID_FIELD_Y}.
     * The target will not switch until the robot crosses
     * {@code MID_FIELD_Y ± HYSTERESIS_BAND}.  This prevents rapid oscillation
     * when the robot hovers near Y = 72".
     */
    public static final double HYSTERESIS_BAND = 1.5;

    // ── Red Alliance goal positions ──────────────────────────────────────────
    public static final double RED_GOAL_X       = 60.0;
    public static final double RED_NEAR_GOAL_Y  = 60.0;   // Audience side
    public static final double RED_FAR_GOAL_Y   = 84.0;   // Back Wall side

    // ── Blue Alliance goal positions ─────────────────────────────────────────
    public static final double BLUE_GOAL_X      = 84.0;
    public static final double BLUE_NEAR_GOAL_Y = 60.0;   // Audience side
    public static final double BLUE_FAR_GOAL_Y  = 84.0;   // Back Wall side

    // ═══════════════════════════════════════════════════════════════════════════
    //  PID Heading Correction Gains (chassis align-to-goal)
    // ═══════════════════════════════════════════════════════════════════════════

    /** Proportional gain for heading error → rotational power. */
    public static double HEADING_kP = 1.2;

    /** Integral gain — keep small to avoid windup; useful for steady-state error. */
    public static double HEADING_kI = 0.0;

    /** Derivative gain — dampens overshoot on fast heading changes. */
    public static double HEADING_kD = 0.08;

    /** Maximum magnitude of the integral accumulator (anti-windup). */
    public static double HEADING_MAX_I = 0.25;

    // ═══════════════════════════════════════════════════════════════════════════
    //  Launcher Velocity Mapping (distance → ticks/sec)
    // ═══════════════════════════════════════════════════════════════════════════

    /** Distance (inches) at or below which we use the minimum launch velocity. */
    public static double DIST_MIN_IN = 12.0;

    /** Distance (inches) at or above which we use the maximum launch velocity. */
    public static double DIST_MAX_IN = 72.0;

    /** Flywheel velocity (ticks/sec) for shortest-range shots. */
    public static double VEL_MIN_TPS = 925.0;

    /** Flywheel velocity (ticks/sec) for longest-range shots. */
    public static double VEL_MAX_TPS = 1075.0;

    // ═══════════════════════════════════════════════════════════════════════════
    //  Instance State
    // ═══════════════════════════════════════════════════════════════════════════

    private final Alliance alliance;

    /** Goal X for this alliance (constant once constructed). */
    private final double goalX;

    /** Near-goal Y for this alliance. */
    private final double nearGoalY;

    /** Far-goal Y for this alliance. */
    private final double farGoalY;

    /**
     * {@code true} = currently targeting the Far goal;
     * {@code false} = currently targeting the Near goal.
     * Initialized to Near at construction.
     */
    private boolean targetingFar = false;

    /** Currently active target Y-coordinate (updated by {@link #update}). */
    private double activeTargetY;

    // ── PID state for heading correction ─────────────────────────────────────
    private double headingIntegral = 0.0;
    private double headingLastError = 0.0;
    private long   lastTimeNanos = 0;

    // ── Cached output values (refreshed each call to {@link #update}) ────────
    private double cachedHeadingRad = 0.0;
    private double cachedDistanceIn = 0.0;
    private double cachedHeadingErrorRad = 0.0;

    // ═══════════════════════════════════════════════════════════════════════════
    //  Construction
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Creates a new tracker locked to the given alliance.
     *
     * @param alliance the alliance whose HIVE goals this tracker targets
     */
    public HiveGoalTracker(Alliance alliance) {
        this.alliance = alliance;

        if (alliance == Alliance.RED) {
            goalX     = RED_GOAL_X;
            nearGoalY = RED_NEAR_GOAL_Y;
            farGoalY  = RED_FAR_GOAL_Y;
        } else {
            goalX     = BLUE_GOAL_X;
            nearGoalY = BLUE_NEAR_GOAL_Y;
            farGoalY  = BLUE_FAR_GOAL_Y;
        }

        // Start targeting the Near goal
        targetingFar   = false;
        activeTargetY  = nearGoalY;
        lastTimeNanos  = System.nanoTime();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Core Update — call once per loop
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Recalculates the active goal, heading, and distance from the given robot
     * pose.  <strong>Call this exactly once per control loop</strong> before
     * reading any output getters.
     *
     * @param robotPose current robot pose from the localizer
     *                  ({@code x, y} in inches; {@code heading} in radians,
     *                  CCW-positive from +X)
     */
    public void update(Pose robotPose) {
        // ── 1. Hysteresis-based target selection ─────────────────────────────
        double robotY = robotPose.y();

        if (targetingFar) {
            // Switch back to Near only when robot drops below the low threshold
            if (robotY < MID_FIELD_Y - HYSTERESIS_BAND) {
                targetingFar  = false;
                activeTargetY = nearGoalY;
            }
        } else {
            // Switch to Far only when robot rises above the high threshold
            if (robotY >= MID_FIELD_Y + HYSTERESIS_BAND) {
                targetingFar  = true;
                activeTargetY = farGoalY;
            }
        }

        // ── 2. Compute vector from robot to target ───────────────────────────
        double dx = goalX - robotPose.x();
        double dy = activeTargetY - robotPose.y();

        cachedDistanceIn    = Math.hypot(dx, dy);
        cachedHeadingRad    = Math.atan2(dy, dx);                // field-frame angle
        cachedHeadingErrorRad = normalizeAngle(cachedHeadingRad - robotPose.heading());
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Output Getters (read after calling update())
    // ═══════════════════════════════════════════════════════════════════════════

    /** @return active target goal X-coordinate (inches). */
    public double getTargetX() {
        return goalX;
    }

    /** @return active target goal Y-coordinate (inches). */
    public double getTargetY() {
        return activeTargetY;
    }

    /** @return {@code true} if currently locked onto the Far goal. */
    public boolean isTargetingFar() {
        return targetingFar;
    }

    /**
     * @return the field-relative heading angle from the robot to the target
     *         (radians, CCW-positive from +X).
     */
    public double getTargetHeadingRad() {
        return cachedHeadingRad;
    }

    /**
     * @return the field-relative heading angle from the robot to the target
     *         (degrees, CCW-positive from +X).
     */
    public double getTargetHeadingDeg() {
        return Math.toDegrees(cachedHeadingRad);
    }

    /**
     * @return signed heading error (radians) = desired heading − robot heading,
     *         normalized to (−π, π].  Positive → robot should turn CCW.
     */
    public double getHeadingErrorRad() {
        return cachedHeadingErrorRad;
    }

    /** @return heading error in degrees. */
    public double getHeadingErrorDeg() {
        return Math.toDegrees(cachedHeadingErrorRad);
    }

    /**
     * @return Euclidean distance (inches) from robot center to active goal.
     */
    public double getDistanceInches() {
        return cachedDistanceIn;
    }

    /** @return the alliance this tracker was constructed for. */
    public Alliance getAlliance() {
        return alliance;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  PID Heading Correction
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Computes a PID-controlled rotational power correction to steer the
     * chassis (or turret) toward the active goal.
     * <p>
     * Typical usage: add this value to the driver's rotational stick input
     * (or use it as the sole rotation source when the driver wants full
     * auto-aim).
     * <p>
     * <strong>Call {@link #update(Pose)} first</strong> so the heading error
     * is fresh.
     *
     * @return rotational power in the range [−1.0, 1.0]; positive = turn CCW.
     */
    public double calculateHeadingCorrection() {
        long   now = System.nanoTime();
        double dt  = (now - lastTimeNanos) / 1.0e9;
        lastTimeNanos = now;

        // Guard against first-call or implausible dt
        if (dt <= 0 || dt > 0.2) {
            dt = 0.02;
        }

        double error = cachedHeadingErrorRad;

        // Integral with anti-windup
        headingIntegral += error * dt;
        if (HEADING_kI > 0) {
            double maxAccum = HEADING_MAX_I / HEADING_kI;
            headingIntegral = clamp(headingIntegral, -maxAccum, maxAccum);
        }

        // Derivative
        double derivative = (error - headingLastError) / dt;
        headingLastError = error;

        double output = (HEADING_kP * error)
                       + (HEADING_kI * headingIntegral)
                       + (HEADING_kD * derivative);

        return clamp(output, -1.0, 1.0);
    }

    /**
     * Resets the integral and derivative state of the heading PID.
     * Call when the driver regains manual rotation control or on OpMode
     * transitions.
     */
    public void resetHeadingPID() {
        headingIntegral  = 0.0;
        headingLastError = 0.0;
        lastTimeNanos    = System.nanoTime();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Launcher Velocity Helper (Optimized Zero-Division Fast-Path)
    // ═══════════════════════════════════════════════════════════════════════════

    private static final double DIST_RANGE_IN = DIST_MAX_IN - DIST_MIN_IN;
    private static final double VEL_RANGE_TPS = VEL_MAX_TPS - VEL_MIN_TPS;
    private static final double VEL_SLOPE     = VEL_RANGE_TPS / DIST_RANGE_IN;

    /**
     * Maps current distance to suggested flywheel velocity (ticks/sec)
     * using zero-division fast-path linear interpolation.
     *
     * @return suggested flywheel velocity in ticks per second
     */
    public double getSuggestedLaunchVelocity() {
        double distOffset = cachedDistanceIn - DIST_MIN_IN;
        if (distOffset <= 0.0) return VEL_MIN_TPS;
        if (distOffset >= DIST_RANGE_IN) return VEL_MAX_TPS;
        return VEL_MIN_TPS + distOffset * VEL_SLOPE;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Static Helpers
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Normalizes an angle to the range (−π, π].
     *
     * @param angleRad input angle in radians
     * @return equivalent angle in (−π, π]
     */
    public static double normalizeAngle(double angleRad) {
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }

    /**
     * Clamps a value between {@code min} and {@code max}.
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

