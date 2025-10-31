package frc.team4276.frc2025.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.subsystems.SuperstructureConstants.ScoringSide;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.path.PathUtil;
import java.util.Map;
import java.util.Optional;

public class FieldConstants {
  public static final int aprilTagCount = 22;
  public static final AprilTagFieldLayout apriltagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static int getTagIdFromSide(ReefSide side) {
    return switch (side) {
      case AB -> AllianceFlipUtil.shouldFlip() ? 7 : 18;
      case CD -> AllianceFlipUtil.shouldFlip() ? 8 : 17;
      case EF -> AllianceFlipUtil.shouldFlip() ? 9 : 22;
      case GH -> AllianceFlipUtil.shouldFlip() ? 10 : 21;
      case IJ -> AllianceFlipUtil.shouldFlip() ? 11 : 20;
      case KL -> AllianceFlipUtil.shouldFlip() ? 6 : 19;
    };
  }

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

  public static Pose3d getTagPose(int id) {
    if (id < 1 || id > 22) {
      throw new IllegalArgumentException("id must be between 1 and 22");
    }

    return apriltagLayout
        .getTagPose(id)
        .orElseThrow(
            () -> {
              final String message = String.format("getTagPose called for unexpected tag %d", id);
              return new RuntimeException(message);
            });
  }

  public static final Map<Pose2d, Integer> blueAlliancePoseToTagIDsMap =
      Map.of(
          getTagPose(21).toPose2d(), 21,
          getTagPose(20).toPose2d(), 20,
          getTagPose(19).toPose2d(), 19,
          getTagPose(18).toPose2d(), 18,
          getTagPose(17).toPose2d(), 17,
          getTagPose(22).toPose2d(), 22);

  public static final Map<Pose2d, Integer> redAlliancePoseToTagIDsMap =
      Map.of(
          getTagPose(6).toPose2d(), 6,
          getTagPose(7).toPose2d(), 7,
          getTagPose(8).toPose2d(), 8,
          getTagPose(9).toPose2d(), 9,
          getTagPose(10).toPose2d(), 10,
          getTagPose(11).toPose2d(), 11);

  public static final Map<Rotation2d, Integer> redAllianceAngleToTagIDsMap =
      Map.of(
          Rotation2d.fromDegrees(-60),
          9,
          Rotation2d.fromDegrees(-120),
          8,
          Rotation2d.k180deg,
          7,
          Rotation2d.fromDegrees(120),
          6,
          Rotation2d.fromDegrees(60),
          1,
          Rotation2d.kZero,
          1);

  public static final Map<Rotation2d, Integer> blueAllianceAngleToTagIDsMap =
      Map.of(
          Rotation2d.fromDegrees(-60),
          12,
          Rotation2d.fromDegrees(-120),
          27,
          Rotation2d.k180deg,
          28,
          Rotation2d.fromDegrees(120),
          29,
          Rotation2d.fromDegrees(60),
          10,
          Rotation2d.kZero,
          11);

  public static final double fieldLength = Units.inchesToMeters(690.875958);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final Translation2d fieldCenter =
      new Translation2d(Units.inchesToMeters(345.437979), Units.inchesToMeters(158.5));

  public static final double reefToFieldCenter = 4.284788;

  public static final Translation2d blueReefCenter =
      fieldCenter.minus(new Translation2d(reefToFieldCenter, 0.0));

  public static final double tagToReef = Units.inchesToMeters(6.468853);
  public static final double reefCenterToTag = Units.inchesToMeters(32.746);

  public static final double bumperToRobotCenter = Units.inchesToMeters(18.625);
  public static final double autoLineupOffset = Units.inchesToMeters(40.0);
  public static final double teleopLineupOffset = Units.inchesToMeters(12.0);
  public static final double scoringOffset = Units.inchesToMeters(1.0);
  public static final double algaePickupOffset = Units.inchesToMeters(0.0);
  public static final double clearReefOffset = Units.inchesToMeters(13.0);

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

  public static Pose2d getClearReefPose(ReefSide reefSide, ScoringSide side) {
    return getReefReferencePose(reefSide, side, clearReefOffset);
  }

  public static Pose2d getAlgaePickupPose(ReefSide reefSide, ScoringSide side) {
    return getReefReferencePose(reefSide, side, algaePickupOffset);
  }

  private static Pose2d getReefReferencePose(ReefSide reefSide, ScoringSide side, double offset) {
    var angle = Rotation2d.fromDegrees(reefSide.ordinal() * 60);

    var reefToPose =
        new Translation2d(
            -1.0 * (reefCenterToTag + bumperToRobotCenter + offset),
            tagToReef
                * ((reefSide.ordinal() < 2 || reefSide.ordinal() == 5)
                    ? (side == ScoringSide.LEFT ? 1 : -1)
                    : (side == ScoringSide.LEFT ? -1 : 1)));

    return AllianceFlipUtil.apply(
        new Pose2d(blueReefCenter.plus(reefToPose.rotateBy(angle)), angle));
  }

  public static boolean getIsLeftScoringRelativeToRobot(ReefSide reefSide, ScoringSide side) {
    return ((reefSide.ordinal() < 2 || reefSide.ordinal() == 5)
        ? (side == ScoringSide.LEFT)
        : (side != ScoringSide.LEFT));
  }

  public static final Pose2d blueProcessorSideStart =
      new Pose2d(7.1415, 1.905, Rotation2d.kCCW_90deg);
  public static final Pose2d blueJITBProcessorSideStart =
      new Pose2d(7.1415, 0.815, Rotation2d.kZero);

  public static final Pose2d blueOutsideStationIntake =
      new Pose2d(1.55, 0.72, Rotation2d.fromDegrees(55));
  public static final Pose2d blueInsideStationIntake =
      new Pose2d(0.69, 1.33, Rotation2d.fromDegrees(55));

  public static Pose2d flippablePose(Pose2d pose, boolean isBargeSide) {
    if (isBargeSide) {
      return PathUtil.mirrorLengthwise(AllianceFlipUtil.apply(pose));
    }

    return AllianceFlipUtil.apply(pose);
  }
}
