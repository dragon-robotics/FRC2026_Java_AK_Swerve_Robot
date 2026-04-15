package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class VisionConstants {
  public static final String[] APTAG_CAMERA_NAMES = {
    "AprilTagPoseEstCameraF",
    "AprilTagPoseEstCameraR",
    "AprilTagPoseEstCameraB",
    "AprilTagPoseEstCameraL"
  };

  public static final Transform3d APTAG_POSE_EST_CAM_F_POS =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-11.152),
              Units.inchesToMeters(-7.579),
              Units.inchesToMeters(20.930)),
          new Rotation3d(0, Units.degreesToRadians(-12), 0));

  public static final Transform3d APTAG_POSE_EST_CAM_R_POS =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-8.387),
              Units.inchesToMeters(-13.355),
              Units.inchesToMeters(15.931)),
          new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-90)));

  public static final Transform3d APTAG_POSE_EST_CAM_B_POS =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-9.164),
              Units.inchesToMeters(12.5),
              Units.inchesToMeters(20.839)),
          new Rotation3d(0, Units.degreesToRadians(-12), Units.degreesToRadians(180)));

  public static final Transform3d APTAG_POSE_EST_CAM_L_POS =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-8.387),
              Units.inchesToMeters(13.355),
              Units.inchesToMeters(15.931)),
          new Rotation3d(0, Units.degreesToRadians(-12), Units.degreesToRadians(90)));

  public static final Transform3d[] APTAG_POSE_EST_CAM_POSITIONS = {
    APTAG_POSE_EST_CAM_F_POS,
    APTAG_POSE_EST_CAM_R_POS,
    APTAG_POSE_EST_CAM_B_POS,
    APTAG_POSE_EST_CAM_L_POS
  };

  public static final Matrix<N3, N1> SINGLE_TAG_STDDEV = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> MULTI_TAG_STDDEV = VecBuilder.fill(0.5, 0.5, 1);
  public static final Matrix<N3, N1> DEFAULT_TAG_STDDEV = VecBuilder.fill(0.9, 0.9, 0.9);

  public static double MAX_AMBIGUITY = 0.3;
  // Z error tolerance for accepted poses. Widened to 0.5m temporarily for
  // camera transform Z debugging — tighten once transforms are confirmed correct.
  public static double MAX_Z_ERROR = 0.5;

  // ── Coplanar / Single-Tag Flip Rejection ────────────────────────────────
  // Two tags on the same hub face (coplanar) give the multi-tag PnP solver
  // NO additional rotational constraint over a single tag — both have 180°
  // flip ambiguity. These constants gate distance, ambiguity, and inter-frame
  // jumps to reject flipped poses.

  /** Maximum angle (degrees) between tag normals to be considered coplanar. */
  public static final double COPLANAR_ANGLE_THRESHOLD_DEG = 10.0;

  /**
   * Maximum distance for coplanar multi-tag observations. More generous than single-tag because
   * multiple coplanar tags improve translational accuracy even when rotational ambiguity persists.
   * Rotation is already distrusted (angular stddev = MAX_VALUE), and the inter-frame jump check is
   * the primary safety net against actual flips.
   */
  public static final double COPLANAR_MAX_DISTANCE_METERS = 7.0;

  /**
   * Maximum distance for true single-tag observations. Beyond this the PnP solver cannot reliably
   * distinguish the true pose from the 180°-flipped alternate.
   */
  public static final double SINGLE_TAG_MAX_DISTANCE_METERS = 5.0;

  /**
   * Maximum ambiguity for true single-tag observations. Tighter than general because a single tag
   * at medium range is the highest flip risk.
   */
  public static final double SINGLE_TAG_MAX_AMBIGUITY = 0.3;

  /**
   * Maximum ambiguity for coplanar multi-tag observations. Uses the same threshold as general
   * multi-tag — the distance gate and inter-frame jump check provide the primary flip protection,
   * not the ambiguity gate.
   */
  public static final double COPLANAR_MAX_AMBIGUITY = MAX_AMBIGUITY;

  // ── Timestamp-Based Flip Detection ─────────────────────────────────────
  // Instead of a fixed inter-frame jump threshold (which fails when there
  // are detection gaps), we compare vision displacement against what is
  // PHYSICALLY POSSIBLE given the elapsed time. No odometry dependency.

  /**
   * Maximum physical robot speed in m/s. Sourced from TunerConstants to stay in sync with the
   * actual drivetrain configuration.
   */
  public static final double MAX_ROBOT_SPEED_MPS =
      TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond);

  /**
   * Multiplier on top of max speed to account for encoder noise, wheel slip, and brief speed spikes
   * during collisions.
   */
  public static final double SPEED_TOLERANCE_MULTIPLIER = 1.5;

  /**
   * Minimum displacement tolerance (meters) applied regardless of dt. Handles vision jitter, small
   * corrections, and prevents false positives on the first frames after a detection gap.
   */
  public static final double BASE_JUMP_TOLERANCE_METERS = 0.5;

  /**
   * Maximum dt (seconds) used for flip detection. If the gap between observations exceeds this, dt
   * is clamped to prevent unbounded tolerance. At 3s: max = 3*4.76*1.5+0.5 = 21.9m — well above any
   * flip (2-5m) but prevents overflow after very long gaps.
   */
  public static final double MAX_FLIP_DETECTION_DT_SECONDS = 3.0;

  /** Maximum tag distance for true multi-tag (non-coplanar) observations. */
  public static final double MAX_TAG_DISTANCE = 8.0;

  // Maximum allowable discrepancy between a vision pose and the current odometry
  // estimate. Readings further than this are rejected to prevent pose snapping.
  // Set generously so normal odometry drift during auto (0.5-1.5m) doesn't
  // cause vision to be permanently rejected — vision is needed to CORRECT drift.
  public static double MAX_POSE_DISCREPANCY_METERS = 2.0;

  // If the swerve odometry has drifted this far from a high-confidence vision
  // fix, the pose estimator is hard-reset to the vision pose to recover from
  // drift. Must be less than MAX_POSE_DISCREPANCY_METERS so the reseed fires
  // before the discrepancy gate starts rejecting vision entirely.
  public static double POSE_RESEED_THRESHOLD_METERS = 1.0;

  // Minimum number of AprilTags required for a pose to be used as a reseed
  // source.
  // Multi-tag fixes are far more reliable than single-tag for hard resets.
  public static int POSE_RESEED_MIN_TAG_COUNT = 2;

  // Baseline std devs before distance/ambiguity scaling.
  // Higher = less trust in vision = less jitter.
  // At 1m, 1 tag, 0 ambiguity: linear ≈ 0.3m, angular ≈ 10deg
  public static double LINEAR_STDDEV_BASELINE = 0.3;
  public static double ANGULAR_STDDEV_BASELINE = Units.degreesToRadians(10);

  public static final double CAMERA_FOV_HORIZONTAL_DEGREES = 73.0;
  public static final double CAMERA_ASPECT_RATIO_WIDTH = 4.0;
  public static final double CAMERA_ASPECT_RATIO_HEIGHT = 3.0;

  public static final double CAMERA_FOV_VERTICAL_DEGREES =
      calculateVerticalFOV(
          CAMERA_FOV_HORIZONTAL_DEGREES, CAMERA_ASPECT_RATIO_WIDTH, CAMERA_ASPECT_RATIO_HEIGHT);

  private static double calculateVerticalFOV(
      double horizontalFOV, double aspectRatioWidth, double aspectRatioHeight) {
    double horizontalFOV_rad = Math.toRadians(horizontalFOV);
    double tan_hFOV_half = Math.tan(horizontalFOV_rad / 2.0);
    double tan_vFOV_half = tan_hFOV_half * (aspectRatioHeight / aspectRatioWidth);
    double vFOV_half_rad = Math.atan(tan_vFOV_half);
    return Math.toDegrees(vFOV_half_rad * 2.0);
  }

  public static double[] CAMERA_STDDEV_FACTORS = new double[] {1.0, 1.0};

  public static double LINEAR_STDDEV_MEGATAG2_FACTOR = 0.5;
  public static double ANGULAR_STDDEV_MEGATAG2_ANGLE_FACTOR = Double.POSITIVE_INFINITY;
}
