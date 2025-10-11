package frc.team4276.frc2025.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.subsystems.SuperstructureConstants.ScoringSide;
import frc.team4276.util.AllianceFlipUtil;
import java.util.Optional;

public class FieldConstants {
  public static final int aprilTagCount = 22;
  public static final AprilTagFieldLayout apriltagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final double fieldLength = Units.inchesToMeters(690.875958);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final Translation2d fieldCenter =
      new Translation2d(Units.inchesToMeters(345.437979), Units.inchesToMeters(158.5));

  public static final double reefToFieldCenter = 4.284788;

  public static final Pose2d blueReefCenter =
      new Pose2d(fieldCenter.minus(new Translation2d(reefToFieldCenter, 0.0)), Rotation2d.kZero);

  public static final double clearReefOffset = Units.inchesToMeters(13.0);
  public static final double scoringOffset = Units.inchesToMeters(18.625);
  public static final double algaePickupOffset = Units.inchesToMeters(1.0);
  public static final double reefCenterToTag = Units.inchesToMeters(32.746);
  public static final double tagToReef = Units.inchesToMeters(6.468853);

  public static final Translation2d reefToLeftAlign =
      new Translation2d(-1.0 * (reefCenterToTag + scoringOffset + clearReefOffset), tagToReef);
  public static final Translation2d reefToRightAlign =
      reefToLeftAlign.plus(new Translation2d(0.0, -2.0 * tagToReef));

  public static final Translation2d reefToLeftScore =
      new Translation2d(-1.0 * (reefCenterToTag + scoringOffset), tagToReef);
  public static final Translation2d reefToRightScore =
      reefToLeftScore.plus(new Translation2d(0.0, -2.0 * tagToReef));

  public static final Translation2d reefToLeftAlgaePickup =
      new Translation2d(-1.0 * (reefCenterToTag + scoringOffset + algaePickupOffset), tagToReef);
  public static final Translation2d reefToRightAlgaePickup =
      reefToLeftScore.plus(new Translation2d(0.0, -2.0 * tagToReef));

  public static final Pose2d[] blueReefToScore = new Pose2d[12];
  public static final Pose2d[] blueReefToAlign = new Pose2d[12];
  public static final Pose2d[] blueReefToAlgaePickup = new Pose2d[12];

  static {
    for (int i = 0; i < 6; i++) {
      var angle = Rotation2d.fromDegrees(i * 60);
      blueReefToScore[i * 2] =
          blueReefCenter.plus(new Transform2d(reefToLeftScore.rotateBy(angle), angle));
      blueReefToAlign[i * 2] =
          blueReefCenter.plus(new Transform2d(reefToLeftAlign.rotateBy(angle), angle));
      blueReefToAlgaePickup[i * 2] =
          blueReefCenter.plus(new Transform2d(reefToLeftAlgaePickup.rotateBy(angle), angle));
      blueReefToScore[i * 2 + 1] =
          blueReefCenter.plus(new Transform2d(reefToRightScore.rotateBy(angle), angle));
      blueReefToAlign[i * 2 + 1] =
          blueReefCenter.plus(new Transform2d(reefToRightAlign.rotateBy(angle), angle));
      blueReefToAlgaePickup[i * 2] =
          blueReefCenter.plus(new Transform2d(reefToRightAlgaePickup.rotateBy(angle), angle));
    }
  }

  public enum Reef {
    A(blueReefToScore[0], blueReefToAlign[0], blueReefToAlgaePickup[0]),
    B(blueReefToScore[1], blueReefToAlign[1], blueReefToAlgaePickup[1]),
    C(blueReefToScore[2], blueReefToAlign[2], blueReefToAlgaePickup[2]),
    D(blueReefToScore[3], blueReefToAlign[3], blueReefToAlgaePickup[3]),
    E(blueReefToScore[4], blueReefToAlign[4], blueReefToAlgaePickup[4]),
    F(blueReefToScore[5], blueReefToAlign[5], blueReefToAlgaePickup[5]),
    G(blueReefToScore[6], blueReefToAlign[6], blueReefToAlgaePickup[6]),
    H(blueReefToScore[7], blueReefToAlign[7], blueReefToAlgaePickup[7]),
    I(blueReefToScore[8], blueReefToAlign[8], blueReefToAlgaePickup[8]),
    J(blueReefToScore[9], blueReefToAlign[9], blueReefToAlgaePickup[9]),
    K(blueReefToScore[10], blueReefToAlign[10], blueReefToAlgaePickup[10]),
    L(blueReefToScore[11], blueReefToAlign[11], blueReefToAlgaePickup[11]);

    private final Pose2d score;
    private final Pose2d align;
    private final Pose2d algaePickup;

    private Reef(Pose2d score, Pose2d align, Pose2d algaePickup) {
      this.score = score;
      this.align = align;
      this.algaePickup = algaePickup;
    }

    public Pose2d getScore() {
      return AllianceFlipUtil.apply(score);
    }

    public Pose2d getBlueScore() {
      return score;
    }

    public Pose2d getRedScore() {
      return AllianceFlipUtil.flip(score);
    }

    public Pose2d getAlign() {
      return AllianceFlipUtil.apply(align);
    }

    public Pose2d getBlueAlign() {
      return align;
    }

    public Pose2d getRedAlign() {
      return AllianceFlipUtil.flip(align);
    }

    public Pose2d getAlgaePickup() {
      return AllianceFlipUtil.apply(algaePickup);
    }

    public Pose2d getBlueAlgaePickup() {
      return algaePickup;
    }

    public Pose2d getRedAlgaePickup() {
      return AllianceFlipUtil.flip(algaePickup);
    }
  }

  public enum ReefSide {
    AB(Reef.A, Reef.B),
    CD(Reef.C, Reef.D),
    EF(Reef.F, Reef.E),
    GH(Reef.H, Reef.G),
    IJ(Reef.J, Reef.I),
    KL(Reef.K, Reef.L);

    private final Reef leftReef; // Left
    private final Reef rightReef; // Right

    private ReefSide(Reef firstReef, Reef secondReef) {
      this.leftReef = firstReef;
      this.rightReef = secondReef;
    }

    public Reef getLeftReef() {
      return leftReef;
    }

    public Reef getRightReef() {
      return rightReef;
    }
  }

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

  public static Optional<Pose2d> getCoralScorePose(int id, ScoringSide side) {
    var reefSide = getSideFromTagId(id);
    if (reefSide.isPresent()) {
      return Optional.of(getCoralScorePose(reefSide.get(), side));
    }
    return Optional.empty();
  }

  public static Pose2d getCoralScorePose(ReefSide reefSide, ScoringSide side) {
    return getReefReferencePose(reefSide, side, scoringOffset);
  }

  public static Pose2d getClearReefPose(ReefSide reefSide, ScoringSide side) {
    return getReefReferencePose(reefSide, side, scoringOffset + clearReefOffset);
  }

  public static Pose2d getAlgaePickupPose(ReefSide reefSide, ScoringSide side) {
    return getReefReferencePose(reefSide, side, scoringOffset + algaePickupOffset);
  }

  private static Pose2d getReefReferencePose(ReefSide reefSide, ScoringSide side, double offset) {
    var angle = Rotation2d.fromDegrees(reefSide.ordinal() * 60);

    var reefToPose =
        new Translation2d(
            -1.0 * (reefCenterToTag + offset),
            tagToReef
                * (side == ScoringSide.LEFT
                    ? ((reefSide.ordinal() < 3 || reefSide.ordinal() == 5) ? 1 : -1)
                    : ((reefSide.ordinal() > 2 && reefSide.ordinal() < 5) ? 1 : -1)));

    return AllianceFlipUtil.apply(
        blueReefCenter.plus(new Transform2d(reefToPose.rotateBy(angle), angle)));
  }

  public static boolean isReefTag(int tagId) {
    return (tagId >= 6 && tagId <= 11) || (tagId >= 17 && tagId <= 22);
  }

  public static final Pose2d blueProcessorSideStart =
      new Pose2d(7.1415, 1.905, Rotation2d.kCCW_90deg);
  public static final Pose2d blueJITBProcessorSideStart =
      new Pose2d(7.1415, 0.815, Rotation2d.kZero);

  public static final Pose2d blueOutsideStationIntake =
      new Pose2d(1.55, 0.72, Rotation2d.fromDegrees(55));
  public static final Pose2d blueInsideStationIntake =
      new Pose2d(0.69, 1.33, Rotation2d.fromDegrees(55));
}
