package frc.robot.util.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;

public final class FieldConstants {
  // The different layouts of the AprilTags on the field
  public static final AprilTagFieldLayout APTAG_FIELD_LAYOUT;

  static {
    Path defaultPath =
        Path.of(
            Filesystem.getDeployDirectory().getPath(),
            "apriltags",
            "welded",
            "2026-rebuilt-welded-no-single.json");
    AprilTagFieldLayout defaultLayout = null;
    try {
      defaultLayout = new AprilTagFieldLayout(defaultPath);
    } catch (IOException e) {
      System.err.println("!!! CRITICAL: Failed to load default AprilTag field resource!");
      DriverStation.reportError(
          "CRITICAL: Failed to load default AprilTag field resource: " + e.getMessage(), true);
      defaultLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
    APTAG_FIELD_LAYOUT = defaultLayout;
  }

  public static final double FIELD_LENGTH = APTAG_FIELD_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH = APTAG_FIELD_LAYOUT.getFieldWidth();
  public static final double FIELD_HEIGHT = Units.inchesToMeters(72);

  public static enum FieldZones {
    ALLIANCE_LEFT,
    // ALLIANCE_LEFT_TRENCH,
    // ALLIANCE_LEFT_BUMP,
    ALLIANCE_RIGHT,
    // ALLIANCE_RIGHT_TRENCH,
    // ALLIANCE_RIGHT_BUMP,
    // ALLIANCE_TOWER,
    NEUTRAL_LEFT,
    // NEUTRAL_LEFT_PURGE,
    NEUTRAL_RIGHT,
    // NEUTRAL_RIGHT_PURGE,
    OPPONENT_LEFT,
    OPPONENT_RIGHT;

    public static FieldZones fromPose(Pose2d pose, DriverStation.Alliance alliance) {
      double x = pose.getX();
      double y = pose.getY();

      // Flip the field for red since everything is calculated relatively to blue //
      double normalizedX = alliance == DriverStation.Alliance.Red ? FIELD_LENGTH - x : x;
      double normalizedY = alliance == DriverStation.Alliance.Red ? FIELD_WIDTH - y : y;

      // Check if we're left or right
      boolean isLeft = normalizedY > LinesHorizontal.CENTER;

      if (normalizedX < LinesVertical.ALLIANCE_ZONE) {
        return isLeft ? ALLIANCE_LEFT : ALLIANCE_RIGHT;
      } else if (normalizedX < LinesVertical.NEUTRAL_ZONE_FAR) {
        return isLeft ? NEUTRAL_LEFT : NEUTRAL_RIGHT;
      } else {
        return isLeft ? OPPONENT_LEFT : OPPONENT_RIGHT;
      }
    }
  }

  // Neutral zone purge left point
  // Neutral zone pass left point
  // Neutral zone outtake left point

  // Neutral zone purge right point
  // Neutral zone pass right point
  // Neutral zone outtake right point

  // Opposing alliance zone pass to neutral zone left point
  // Opposing alliance zone pass to alliance zone left point
  // Opposing alliance zone pass to neutral zone right point
  // Opposing alliance zone pass to alliance zone right point

  /**
   * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
   */
  public static class LinesVertical {
    public static final double CENTER = FIELD_LENGTH / 2.0;
    public static final double STARTING = APTAG_FIELD_LAYOUT.getTagPose(26).get().getX();
    public static final double ALLIANCE_ZONE = STARTING;
    public static final double HUB_CENTER =
        APTAG_FIELD_LAYOUT.getTagPose(26).get().getX() + Hub.WIDTH / 2.0;
    public static final double NEUTRAL_ZONE_NEAR = CENTER - Units.inchesToMeters(120);
    public static final double NEUTRAL_ZONE_FAR = CENTER + Units.inchesToMeters(120);
    public static final double OPP_HUB_CENTER =
        APTAG_FIELD_LAYOUT.getTagPose(4).get().getX() + Hub.WIDTH / 2.0;
    public static final double OPP_ALLIANCE_ZONE = APTAG_FIELD_LAYOUT.getTagPose(10).get().getX();
  }

  /**
   * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
   *
   * <p>NOTE: The field element start and end are always left to right from the perspective of the
   * alliance station
   */
  public static class LinesHorizontal {

    public static final double CENTER = FIELD_WIDTH / 2.0;

    // Right of hub
    public static final double BLUE_RIGHT_BUMP_START = Hub.BLUE_NEAR_RIGHT_CORNER.getY();
    public static final double BLUE_RIGHT_BUMP_END = BLUE_RIGHT_BUMP_START - RightBump.WIDTH;
    public static final double BLUE_RIGHT_BUMP_MIDDLE =
        (BLUE_RIGHT_BUMP_START + BLUE_RIGHT_BUMP_END) / 2.0;
    public static final double BLUE_RIGHT_TRENCH_OPEN_START =
        BLUE_RIGHT_BUMP_END - Units.inchesToMeters(12.0);
    public static final double BLUE_RIGHT_TRENCH_OPEN_END = 0;

    public static final double RED_RIGHT_BUMP_END = Hub.RED_NEAR_RIGHT_CORNER.getY();
    public static final double RED_RIGHT_BUMP_START = RED_RIGHT_BUMP_END - RightBump.WIDTH;
    public static final double RED_RIGHT_BUMP_MIDDLE =
        (RED_RIGHT_BUMP_START + RED_RIGHT_BUMP_END) / 2.0;
    public static final double RED_RIGHT_TRENCH_OPEN_START =
        RED_RIGHT_BUMP_END - Units.inchesToMeters(12.0);
    public static final double RED_RIGHT_TRENCH_OPEN_END = 0;

    // Left of hub
    public static final double BLUE_LEFT_BUMP_END = Hub.BLUE_NEAR_LEFT_CORNER.getY();
    public static final double BLUE_LEFT_BUMP_START = BLUE_LEFT_BUMP_END + LeftBump.WIDTH;
    public static final double BLUE_LEFT_BUMP_MIDDLE =
        (BLUE_LEFT_BUMP_START + BLUE_LEFT_BUMP_END) / 2.0;
    public static final double BLUE_LEFT_TRENCH_OPEN_END =
        BLUE_LEFT_BUMP_START + Units.inchesToMeters(12.0);
    public static final double BLUE_LEFT_TRENCH_OPEN_START = FIELD_WIDTH;

    public static final double RED_LEFT_BUMP_START = Hub.RED_NEAR_LEFT_CORNER.getY();
    public static final double RED_LEFT_BUMP_END = RED_LEFT_BUMP_START + LeftBump.WIDTH;
    public static final double RED_LEFT_BUMP_MIDDLE =
        (RED_LEFT_BUMP_START + RED_LEFT_BUMP_END) / 2.0;
    public static final double RED_LEFT_TRENCH_OPEN_END =
        RED_LEFT_BUMP_END + Units.inchesToMeters(12.0);
    public static final double RED_LEFT_TRENCH_OPEN_START = FIELD_WIDTH;
  }

  public static final class Hub {

    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(47.0);
    public static final double HEIGHT =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double INNER_WIDTH = Units.inchesToMeters(41.7);
    public static final double INNER_HEIGHT = Units.inchesToMeters(56.5);

    // ── Blue Alliance ──────────────────────────────────────────────────────

    public static final Translation3d BLUE_TOP_CENTER_POINT =
        new Translation3d(
            APTAG_FIELD_LAYOUT.getTagPose(26).get().getX() + WIDTH / 2.0,
            FIELD_WIDTH / 2.0,
            HEIGHT);

    public static final Translation3d BLUE_INNER_CENTER_POINT =
        new Translation3d(
            APTAG_FIELD_LAYOUT.getTagPose(26).get().getX() + INNER_WIDTH / 2.0,
            FIELD_WIDTH / 2.0,
            INNER_HEIGHT);

    public static final Translation2d BLUE_CENTER_POSE =
        new Translation2d(
            APTAG_FIELD_LAYOUT.getTagPose(26).get().getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0);

    public static final Translation2d BLUE_NEAR_LEFT_CORNER =
        new Translation2d(
            BLUE_TOP_CENTER_POINT.getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0 + WIDTH / 2.0);
    public static final Translation2d BLUE_NEAR_RIGHT_CORNER =
        new Translation2d(
            BLUE_TOP_CENTER_POINT.getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0 - WIDTH / 2.0);
    public static final Translation2d BLUE_FAR_LEFT_CORNER =
        new Translation2d(
            BLUE_TOP_CENTER_POINT.getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0 + WIDTH / 2.0);
    public static final Translation2d BLUE_FAR_RIGHT_CORNER =
        new Translation2d(
            BLUE_TOP_CENTER_POINT.getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0 - WIDTH / 2.0);

    // Blue hub faces — sourced directly from AprilTag poses
    public static final Pose2d BLUE_NEAR_FACE = APTAG_FIELD_LAYOUT.getTagPose(26).get().toPose2d();
    public static final Pose2d BLUE_FAR_FACE = APTAG_FIELD_LAYOUT.getTagPose(20).get().toPose2d();
    public static final Pose2d BLUE_RIGHT_FACE = APTAG_FIELD_LAYOUT.getTagPose(18).get().toPose2d();
    public static final Pose2d BLUE_LEFT_FACE = APTAG_FIELD_LAYOUT.getTagPose(21).get().toPose2d();

    // ── Red Alliance ───────────────────────────────────────────────────────
    // Red hub is the field-symmetric mirror of blue:
    // X_red = FIELD_LENGTH - X_blue
    // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)

    public static final Translation3d RED_TOP_CENTER_POINT =
        new Translation3d(
            APTAG_FIELD_LAYOUT.getTagPose(10).get().getX() - WIDTH / 2.0,
            FIELD_WIDTH / 2.0,
            HEIGHT);

    public static final Translation3d RED_INNER_CENTER_POINT =
        new Translation3d(
            APTAG_FIELD_LAYOUT.getTagPose(10).get().getX() - INNER_WIDTH / 2.0,
            FIELD_WIDTH / 2.0,
            INNER_HEIGHT);

    public static final Translation2d RED_CENTER_POSE =
        new Translation2d(
            APTAG_FIELD_LAYOUT.getTagPose(10).get().getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0);

    public static final Translation2d RED_NEAR_LEFT_CORNER =
        new Translation2d(
            RED_TOP_CENTER_POINT.getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0 + WIDTH / 2.0);
    public static final Translation2d RED_NEAR_RIGHT_CORNER =
        new Translation2d(
            RED_TOP_CENTER_POINT.getX() + WIDTH / 2.0, FIELD_WIDTH / 2.0 - WIDTH / 2.0);
    public static final Translation2d RED_FAR_LEFT_CORNER =
        new Translation2d(
            RED_TOP_CENTER_POINT.getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0 + WIDTH / 2.0);
    public static final Translation2d RED_FAR_RIGHT_CORNER =
        new Translation2d(
            RED_TOP_CENTER_POINT.getX() - WIDTH / 2.0, FIELD_WIDTH / 2.0 - WIDTH / 2.0);

    // Red hub faces — sourced directly from AprilTag poses
    // Note: near/far and left/right are from the red driver station perspective
    public static final Pose2d RED_NEAR_FACE = APTAG_FIELD_LAYOUT.getTagPose(10).get().toPose2d();
    public static final Pose2d RED_FAR_FACE = APTAG_FIELD_LAYOUT.getTagPose(4).get().toPose2d();
    public static final Pose2d RED_RIGHT_FACE = APTAG_FIELD_LAYOUT.getTagPose(2).get().toPose2d();
    public static final Pose2d RED_LEFT_FACE = APTAG_FIELD_LAYOUT.getTagPose(5).get().toPose2d();
  }

  public static final class LeftBump {
    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(73.0);
    public static final double HEIGHT = Units.inchesToMeters(6.513);
    public static final double DEPTH = Units.inchesToMeters(44.4);

    // ── Blue Alliance ──────────────────────────────────────────────────────
    public static final Translation2d BLUE_NEAR_LEFT_CORNER =
        Hub.BLUE_NEAR_LEFT_CORNER.plus(new Translation2d(0.0, WIDTH));
    public static final Translation2d BLUE_NEAR_RIGHT_CORNER =
        Hub.BLUE_NEAR_RIGHT_CORNER.plus(new Translation2d(0.0, WIDTH));
    public static final Translation2d BLUE_FAR_LEFT_CORNER =
        Hub.BLUE_FAR_LEFT_CORNER.plus(new Translation2d(0.0, WIDTH));
    public static final Translation2d BLUE_FAR_RIGHT_CORNER =
        Hub.BLUE_FAR_RIGHT_CORNER.plus(new Translation2d(0.0, WIDTH));

    // ── Red Alliance ───────────────────────────────────────────────────────
    // Red bump is the field-symmetric mirror of blue:
    // X_red = FIELD_LENGTH - X_blue
    // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)
    public static final Translation2d RED_NEAR_LEFT_CORNER =
        Hub.RED_NEAR_LEFT_CORNER.plus(new Translation2d(0.0, WIDTH));
    public static final Translation2d RED_NEAR_RIGHT_CORNER =
        Hub.RED_NEAR_RIGHT_CORNER.plus(new Translation2d(0.0, WIDTH));
    public static final Translation2d RED_FAR_LEFT_CORNER =
        Hub.RED_FAR_LEFT_CORNER.plus(new Translation2d(0.0, WIDTH));
    public static final Translation2d RED_FAR_RIGHT_CORNER =
        Hub.RED_FAR_RIGHT_CORNER.plus(new Translation2d(0.0, WIDTH));
  }

  public static final class RightBump {

    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(73.0);
    public static final double HEIGHT = Units.inchesToMeters(6.513);
    public static final double DEPTH = Units.inchesToMeters(44.4);

    // ── Blue Alliance ──────────────────────────────────────────────────────
    public static final Translation2d BLUE_NEAR_LEFT_CORNER = Hub.BLUE_NEAR_RIGHT_CORNER;
    public static final Translation2d BLUE_NEAR_RIGHT_CORNER =
        Hub.BLUE_NEAR_RIGHT_CORNER.minus(new Translation2d(0.0, WIDTH));
    public static final Translation2d BLUE_FAR_LEFT_CORNER = Hub.BLUE_FAR_LEFT_CORNER;
    public static final Translation2d BLUE_FAR_RIGHT_CORNER =
        Hub.BLUE_FAR_RIGHT_CORNER.minus(new Translation2d(0.0, WIDTH));

    // ── Red Alliance ───────────────────────────────────────────────────────
    // Red bump is the field-symmetric mirror of blue:
    // X_red = FIELD_LENGTH - X_blue
    // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)
    public static final Translation2d RED_NEAR_LEFT_CORNER = Hub.RED_NEAR_RIGHT_CORNER;
    public static final Translation2d RED_NEAR_RIGHT_CORNER =
        Hub.RED_NEAR_RIGHT_CORNER.minus(new Translation2d(0.0, WIDTH));
    public static final Translation2d RED_FAR_LEFT_CORNER = Hub.RED_FAR_LEFT_CORNER;
    public static final Translation2d RED_FAR_RIGHT_CORNER =
        Hub.RED_FAR_RIGHT_CORNER.minus(new Translation2d(0.0, WIDTH));
  }

  public static final class LeftTrench {
    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(65.65);
    public static final double DEPTH = Units.inchesToMeters(47.0);
    public static final double HEIGHT = Units.inchesToMeters(40.25);
    public static final double OPENING_WIDTH = Units.inchesToMeters(50.34);
    public static final double OPENING_HEIGHT = Units.inchesToMeters(22.25);

    // ── Blue Alliance ──────────────────────────────────────────────────────
    public static final Translation3d BLUE_OPENING_TOP_LEFT =
        new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH, OPENING_HEIGHT);
    public static final Translation3d BLUE_OPENING_TOP_RIGHT =
        new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH - OPENING_WIDTH, OPENING_HEIGHT);

    // ── Red Alliance ───────────────────────────────────────────────────────
    // Red trench is the field-symmetric mirror of blue:
    // X_red = FIELD_LENGTH - X_blue
    // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)
    public static final Translation3d RED_OPENING_TOP_LEFT =
        new Translation3d(LinesVertical.HUB_CENTER, 0.0, OPENING_HEIGHT);
    public static final Translation3d RED_OPENING_TOP_RIGHT =
        new Translation3d(LinesVertical.HUB_CENTER, OPENING_WIDTH, OPENING_HEIGHT);
  }

  public static final class RightTrench {
    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(65.65);
    public static final double DEPTH = Units.inchesToMeters(47.0);
    public static final double HEIGHT = Units.inchesToMeters(40.25);
    public static final double OPENING_WIDTH = Units.inchesToMeters(50.34);
    public static final double OPENING_HEIGHT = Units.inchesToMeters(22.25);

    // ── Blue Alliance ──────────────────────────────────────────────────────
    public static final Translation3d BLUE_OPENING_TOP_LEFT =
        new Translation3d(LinesVertical.HUB_CENTER, OPENING_WIDTH, OPENING_HEIGHT);
    public static final Translation3d BLUE_OPENING_TOP_RIGHT =
        new Translation3d(LinesVertical.HUB_CENTER, 0, OPENING_HEIGHT);

    // ── Red Alliance ───────────────────────────────────────────────────────
    // Red trench is the field-symmetric mirror of blue:
    // X_red = FIELD_LENGTH - X_blue
    // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)
    public static final Translation3d RED_OPENING_TOP_LEFT =
        new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH - OPENING_WIDTH, OPENING_HEIGHT);
    public static final Translation3d RED_OPENING_TOP_RIGHT =
        new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH, OPENING_HEIGHT);
  }

  // public static final class Tower {
  // // Dimensions
  // public static final double WIDTH = Units.inchesToMeters(49.25);
  // public static final double DEPTH = Units.inchesToMeters(45.0);
  // public static final double HEIGHT = Units.inchesToMeters(78.25);
  // public static final double INNER_OPENING_WIDTH =
  // Units.inchesToMeters(32.250);
  // public static final double FRONT_FACE_X = Units.inchesToMeters(43.51);
  // public static final double UPRIGHT_HEIGHT = Units.inchesToMeters(72.1);

  // // Rung heights from the floor
  // public static final double LOW_RUNG_HEIGHT = Units.inchesToMeters(27.0);
  // public static final double MID_RUNG_HEIGHT = Units.inchesToMeters(45.0);
  // public static final double HIGH_RUNG_HEIGHT = Units.inchesToMeters(63.0);

  // // ── Blue Alliance ──────────────────────────────────────────────────────
  // public static final Translation2d BLUE_CENTER_POINT =
  // new Translation2d(FRONT_FACE_X,
  // APTAG_FIELD_LAYOUT.getTagPose(31).get().getY());
  // public static final Translation2d BLUE_LEFT_UPRIGHT =
  // new Translation2d(
  // FRONT_FACE_X,
  // (APTAG_FIELD_LAYOUT.getTagPose(31).get().getY())
  // + INNER_OPENING_WIDTH / 2
  // + Units.inchesToMeters(0.75));
  // public static final Translation2d BLUE_RIGHT_UPRIGHT =
  // new Translation2d(
  // FRONT_FACE_X,
  // (APTAG_FIELD_LAYOUT.getTagPose(31).get().getY())
  // - INNER_OPENING_WIDTH / 2
  // - Units.inchesToMeters(0.75));

  // // ── Red Alliance ───────────────────────────────────────────────────────
  // // Red trench is the field-symmetric mirror of blue:
  // // X_red = FIELD_LENGTH - X_blue
  // // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)
  // public static final Translation2d RED_CENTER_POINT =
  // new Translation2d(
  // FIELD_LENGTH - FRONT_FACE_X,
  // FIELD_WIDTH - APTAG_FIELD_LAYOUT.getTagPose(31).get().getY());
  // public static final Translation2d RED_LEFT_UPRIGHT =
  // new Translation2d(
  // FIELD_LENGTH - FRONT_FACE_X,
  // (FIELD_WIDTH - APTAG_FIELD_LAYOUT.getTagPose(31).get().getY())
  // + INNER_OPENING_WIDTH / 2
  // + Units.inchesToMeters(0.75));
  // public static final Translation2d RED_RIGHT_UPRIGHT =
  // new Translation2d(
  // FIELD_LENGTH - FRONT_FACE_X,
  // (FIELD_WIDTH - APTAG_FIELD_LAYOUT.getTagPose(31).get().getY())
  // - INNER_OPENING_WIDTH / 2
  // - Units.inchesToMeters(0.75));
  // }

  public static final class Depot {
    // Dimensions
    public static final double WIDTH = Units.inchesToMeters(42.0);
    public static final double DEPTH = Units.inchesToMeters(27.0);
    public static final double HEIGHT = Units.inchesToMeters(1.125);
    public static final double DISTANCE_FROM_CENTER_Y = Units.inchesToMeters(75.93);

    // ── Blue Alliance ───────────────────────────────────────────────────────
    public static final Translation3d BLUE_DEPOT_CENTER =
        new Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y, HEIGHT);
    public static final Translation3d BLUE_LEFT_CORNER =
        new Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y + (WIDTH / 2), HEIGHT);
    public static final Translation3d BLUE_RIGHT_CORNER =
        new Translation3d(DEPTH, (FIELD_WIDTH / 2) + DISTANCE_FROM_CENTER_Y - (WIDTH / 2), HEIGHT);

    // ── Red Alliance ───────────────────────────────────────────────────────
    // Red depot is the field-symmetric mirror of blue:
    // X_red = FIELD_LENGTH - X_blue
    // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)
    public static final Translation3d RED_DEPOT_CENTER =
        new Translation3d(FIELD_LENGTH - DEPTH, (FIELD_WIDTH / 2) - DISTANCE_FROM_CENTER_Y, HEIGHT);
    public static final Translation3d RED_LEFT_CORNER =
        new Translation3d(
            FIELD_LENGTH - DEPTH, (FIELD_WIDTH / 2) - DISTANCE_FROM_CENTER_Y + (WIDTH / 2), HEIGHT);
    public static final Translation3d RED_RIGHT_CORNER =
        new Translation3d(
            FIELD_LENGTH - DEPTH, (FIELD_WIDTH / 2) - DISTANCE_FROM_CENTER_Y - (WIDTH / 2), HEIGHT);
  }

  // public static final class Outpost {
  // // Dimensions
  // public static final double WIDTH = Units.inchesToMeters(31.8);
  // public static final double OPENING_DISTANCE_FROM_FLOOR =
  // Units.inchesToMeters(28.1);
  // public static final double HEIGHT = Units.inchesToMeters(7.0);

  // // ── Blue Alliance ───────────────────────────────────────────────────────
  // public static final Translation2d BLUE_CENTER_POINT =
  // new Translation2d(0, APTAG_FIELD_LAYOUT.getTagPose(29).get().getY());

  // // ── Red Alliance ───────────────────────────────────────────────────────
  // // Red outpost is the field-symmetric mirror of blue:
  // // X_red = FIELD_LENGTH - X_blue
  // // Y_red = FIELD_WIDTH - Y_blue (left/right labels swap across centerline)
  // public static final Translation2d RED_CENTER_POINT =
  // new Translation2d(
  // FIELD_LENGTH, FIELD_WIDTH - APTAG_FIELD_LAYOUT.getTagPose(29).get().getY());
  // }
}
