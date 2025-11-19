package frc.team4276.frc2025.field;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team4276.lib.AllianceFlipUtil;

public class FieldConstants {
  public static final AprilTagFieldLayout apriltagLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final double fieldLength = Units.inchesToMeters(690.875958);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final Translation2d fieldCenter = new Translation2d(Units.inchesToMeters(345.437979),
      Units.inchesToMeters(158.5));

  public static final double reefToFieldCenter = 4.284788;

  public static final Translation2d blueReefCenter = fieldCenter.minus(new Translation2d(reefToFieldCenter, 0.0));

  public static final double tagToReef = Units.inchesToMeters(6.468853);
  public static final double reefCenterToTag = Units.inchesToMeters(32.746);

  public static final double bumperToRobotCenter = Units.inchesToMeters(18.625);
  public static final double autoLineupOffset = Units.inchesToMeters(40.0);
  public static final double teleopLineupOffset = Units.inchesToMeters(12.0);
  public static final double scoringOffset = Units.inchesToMeters(0.0);
  public static final double lateralL1Offset = Units.inchesToMeters(5.0);
  public static final double algaePickupOffset = Units.inchesToMeters(0.0);
  public static final double clearReefOffset = Units.inchesToMeters(13.0);

  public static Optional<ReefSide> getSideFromTagId(int id) {
    return switch (id) {
      case 6 -> Optional.of(ReefSide.KL);
      case 7 -> Optional.of(ReefSide.AB);
      case 8 -> Optional.of(ReefSide.CD);
      case 9 -> Optional.of(ReefSide.EF);
      case 10 -> Optional.of(ReefSide.GH);
      case 11 -> Optional.of(ReefSide.IJ);

      case 17 -> Optional.of(ReefSide.CD);
      case 18 -> Optional.of(ReefSide.AB);
      case 19 -> Optional.of(ReefSide.KL);
      case 20 -> Optional.of(ReefSide.IJ);
      case 21 -> Optional.of(ReefSide.GH);
      case 22 -> Optional.of(ReefSide.EF);

      default -> Optional.empty();
    };
  }

  public static boolean isReefTag(int tagId) {
    return (tagId >= 6 && tagId <= 11) || (tagId >= 17 && tagId <= 22);
  }

  public static enum ScoringSide {
    LEFT,
    RIGHT,
    BOTH
  }

  public enum ReefSide {
    AB,
    CD,
    EF,
    GH,
    IJ,
    KL;

    public Pose2d getLeftScorePose() {
      return getCoralScorePose(this, ScoringSide.LEFT);
    }

    public Pose2d getRightScorePose() {
      return getCoralScorePose(this, ScoringSide.RIGHT);
    }

    public Pose2d getLeftReefClearPose() {
      return getClearReefPose(this, ScoringSide.LEFT);
    }

    public Pose2d getRightReefClearPose() {
      return getClearReefPose(this, ScoringSide.RIGHT);
    }

    public Pose2d getLeftAlgaePickupPose() {
      return getAlgaePickupPose(this, ScoringSide.LEFT);
    }

    public Pose2d getRightAlgaePickupPose() {
      return getAlgaePickupPose(this, ScoringSide.RIGHT);
    }
  }

  public static Optional<Pose2d> getCoralScorePose(int id, ScoringSide side) {
    var reefSide = getSideFromTagId(id);
    if (reefSide.isPresent()) {
      return Optional.of(getCoralScorePose(reefSide.get(), side));
    }
    return Optional.empty();
  }

  public static Pose2d getAutoLineupPose(ReefSide reefSide, ScoringSide side) {
    return getReefReferencePose(reefSide, side, autoLineupOffset);
  }

  public static Pose2d getTeleopLineupPose(ReefSide reefSide, ScoringSide side) {
    return getReefReferencePose(reefSide, side, teleopLineupOffset);
  }

  public static Pose2d getCoralScorePose(ReefSide reefSide, ScoringSide side) {
    return getReefReferencePose(reefSide, side, scoringOffset);
  }

  public static Pose2d getL1TeleopLineupPose(ReefSide reefSide, ScoringSide side) {
    return getReefReferencePose(reefSide, side, teleopLineupOffset, lateralL1Offset);
  }

  public static Pose2d getL1CoralScorePose(ReefSide reefSide, ScoringSide side) {
    return getReefReferencePose(reefSide, side, scoringOffset, lateralL1Offset);
  }

  public static Pose2d getClearReefPose(ReefSide reefSide, ScoringSide side) {
    return getReefReferencePose(reefSide, side, clearReefOffset);
  }

  public static Pose2d getAlgaePickupPose(ReefSide reefSide, ScoringSide side) {
    return getReefReferencePose(reefSide, side, algaePickupOffset);
  }

  private static Pose2d getReefReferencePose(ReefSide reefSide, ScoringSide side, double offset) {
    return getReefReferencePose(reefSide, side, offset, 0.0);
  }

  private static Pose2d getReefReferencePose(
      ReefSide reefSide, ScoringSide side, double offset, double lateralOffset) {
    var angle = Rotation2d.fromDegrees(reefSide.ordinal() * 60);

    var reefToPose = new Translation2d(
        -1.0 * (reefCenterToTag + bumperToRobotCenter + offset),
        (tagToReef + lateralOffset)
            * ((reefSide.ordinal() < 2 || reefSide.ordinal() == 5)
                ? (side == ScoringSide.LEFT ? 1 : -1)
                : (side == ScoringSide.LEFT ? -1 : 1)));

    return AllianceFlipUtil.apply(
        new Pose2d(blueReefCenter.plus(reefToPose.rotateBy(angle)), angle));
  }

  public static boolean getIsLeftScoringRelativeToRobot(ReefSide reefSide, ScoringSide side) {
    return ((reefSide.ordinal() < 2 || reefSide.ordinal() == 5)
        ? (side == ScoringSide.LEFT)
        : (side == ScoringSide.RIGHT));
  }
}
