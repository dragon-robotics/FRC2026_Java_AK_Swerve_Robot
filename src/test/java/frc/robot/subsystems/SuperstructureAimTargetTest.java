package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.constants.FieldConstants;
import frc.robot.util.constants.FieldConstants.FieldZones;
import org.junit.jupiter.api.Test;

class SuperstructureAimTargetTest {

  @Test
  void redAllianceFacingGeometricTargetIsAligned() {
    Translation2d target = new Translation2d(1.0, 0.0);
    Pose2d poseFacingGeometricTarget = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));

    double geometricHeading =
        Superstructure.resolveGeometricTargetHeadingRadians(poseFacingGeometricTarget, target);
    double headingErrorRad =
        poseFacingGeometricTarget.getRotation().getRadians() - geometricHeading;
    headingErrorRad = Math.IEEEremainder(headingErrorRad, 2.0 * Math.PI);
    double headingErrorDeg = Math.abs(Math.toDegrees(headingErrorRad));

    assertTrue(headingErrorDeg < 1e-9);
    assertTrue(Superstructure.isHeadingAlignedToTarget(poseFacingGeometricTarget, target, 5.0));
  }

  @Test
  void geometricAlignmentIsSameForBlueAndRed() {
    Translation2d target = new Translation2d(1.0, 0.0);

    Pose2d poseFacingTarget = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    Pose2d poseFacingAway = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180.0));

    assertTrue(Superstructure.isHeadingAlignedToTarget(poseFacingTarget, target, 5.0));
    assertFalse(Superstructure.isHeadingAlignedToTarget(poseFacingAway, target, 5.0));
  }

  @Test
  void operatorPerspectiveHeadingStillDiffersByPiBetweenBlueAndRed() {
    Pose2d pose = new Pose2d(2.0, 3.0, Rotation2d.kZero);
    Translation2d target = new Translation2d(6.0, 7.0);

    double blueTarget =
        Superstructure.resolveOperatorPerspectiveTargetHeadingRadians(
            pose, target, DriverStation.Alliance.Blue);
    double redTarget =
        Superstructure.resolveOperatorPerspectiveTargetHeadingRadians(
            pose, target, DriverStation.Alliance.Red);

    double delta = Math.IEEEremainder(redTarget - blueTarget, 2.0 * Math.PI);
    assertEquals(Math.PI, Math.abs(delta), 1e-9);
  }

  @Test
  void redRightAimPointsMirrorRedLeftAcrossHorizontalCenterline() {
    assertEquals(
        FieldConstants.FIELD_WIDTH - FieldConstants.AimPoints.RED_LEFT_PURGE_POINT.getY(),
        FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT.getY());
    assertEquals(
        FieldConstants.FIELD_WIDTH - FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT.getY(),
        FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT.getY());

    assertNotEquals(
        FieldConstants.AimPoints.RED_LEFT_PURGE_POINT,
        FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT);
    assertNotEquals(
        FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT,
        FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT);
  }

  @Test
  void redAllianceNeutralZonesUseDirectLeftRightAimPoints() {
    assertEquals(
        FieldConstants.AimPoints.RED_LEFT_SHOOT_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_LEFT_SHOOT, DriverStation.Alliance.Red));
    assertEquals(
        FieldConstants.AimPoints.RED_RIGHT_SHOOT_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_RIGHT_SHOOT, DriverStation.Alliance.Red));
    assertEquals(
        FieldConstants.AimPoints.RED_LEFT_PURGE_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_LEFT_PURGE, DriverStation.Alliance.Red));
    assertEquals(
        FieldConstants.AimPoints.RED_RIGHT_PURGE_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_RIGHT_PURGE, DriverStation.Alliance.Red));
  }

  @Test
  void blueAllianceNeutralZonesKeepStandardLeftRightAimPoints() {
    assertEquals(
        FieldConstants.AimPoints.BLUE_LEFT_SHOOT_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_LEFT_SHOOT, DriverStation.Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.BLUE_RIGHT_SHOOT_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_RIGHT_SHOOT, DriverStation.Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.BLUE_LEFT_PURGE_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_LEFT_PURGE, DriverStation.Alliance.Blue));
    assertEquals(
        FieldConstants.AimPoints.BLUE_RIGHT_PURGE_POINT,
        Superstructure.resolveAimTargetForZone(
            true, FieldZones.NEUTRAL_RIGHT_PURGE, DriverStation.Alliance.Blue));
  }

  @Test
  void defaultsToAllianceHubWhenZoneUnknownOrUnconfirmed() {
    Translation2d redHub = FieldConstants.Hub.RED_CENTER_POSE;
    Translation2d blueHub = FieldConstants.Hub.BLUE_CENTER_POSE;

    assertEquals(
        redHub,
        Superstructure.resolveAimTargetForZone(
            false, FieldZones.NEUTRAL_LEFT_SHOOT, DriverStation.Alliance.Red));
    assertEquals(
        redHub, Superstructure.resolveAimTargetForZone(true, null, DriverStation.Alliance.Red));
    assertEquals(
        blueHub,
        Superstructure.resolveAimTargetForZone(
            false, FieldZones.NEUTRAL_RIGHT_PURGE, DriverStation.Alliance.Blue));
    assertEquals(
        blueHub, Superstructure.resolveAimTargetForZone(true, null, DriverStation.Alliance.Blue));
  }
}
