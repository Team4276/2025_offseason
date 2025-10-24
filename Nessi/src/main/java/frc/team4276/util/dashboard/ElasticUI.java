package frc.team4276.util.dashboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.dashboard.Elastic.Notification;
import frc.team4276.util.dashboard.Elastic.Notification.NotificationLevel;
import java.util.function.Supplier;

public class ElasticUI {
  private ElasticUI() {}

  private static final Timer rainbowTimer = new Timer();
  private static final String[] rainbow;
  private static int offset = 0;

  static {
    rainbowTimer.restart();

    rainbow =
        new String[] {
          new Color8Bit(Color.kRed).toHexString(),
          new Color8Bit(Color.kRed).toHexString(),
          new Color8Bit(Color.kOrange).toHexString(),
          new Color8Bit(Color.kOrange).toHexString(),
          new Color8Bit(Color.kYellow).toHexString(),
          new Color8Bit(Color.kYellow).toHexString(),
          new Color8Bit(Color.kGreen).toHexString(),
          new Color8Bit(Color.kGreen).toHexString(),
          new Color8Bit(Color.kBlue).toHexString(),
          new Color8Bit(Color.kBlue).toHexString(),
          new Color8Bit(Color.kPurple).toHexString(),
          new Color8Bit(Color.kPurple).toHexString()
        };
  }

  public static void update() {
    if (rainbowTimer.advanceIfElapsed(0.06)) {
      if (offset >= rainbow.length) {
        offset = 0;
      }
      var colors = new String[rainbow.length];
      for (int i = 0; i < rainbow.length; i++) {
        int index = (int) MathUtil.inputModulus(i + offset, 0, rainbow.length - 1);

        colors[i] = rainbow[index];
      }
      SmartDashboard.putStringArray("Rainbow", colors);
      offset++;
    }

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public static void putSwerveDrive(
      Supplier<SwerveModuleState[]> state, Supplier<Rotation2d> angle) {
    SmartDashboard.putData(
        "Swerve Drive",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty(
                "Front Left Angle", () -> state.get()[0].angle.getRadians(), null);
            builder.addDoubleProperty(
                "Front Left Velocity", () -> state.get()[0].speedMetersPerSecond, null);

            builder.addDoubleProperty(
                "Front Right Angle", () -> state.get()[1].angle.getRadians(), null);
            builder.addDoubleProperty(
                "Front Right Velocity", () -> state.get()[1].speedMetersPerSecond, null);

            builder.addDoubleProperty(
                "Back Left Angle", () -> state.get()[2].angle.getRadians(), null);
            builder.addDoubleProperty(
                "Back Left Velocity", () -> state.get()[2].speedMetersPerSecond, null);

            builder.addDoubleProperty(
                "Back Right Angle", () -> state.get()[3].angle.getRadians(), null);
            builder.addDoubleProperty(
                "Back Right Velocity", () -> state.get()[3].speedMetersPerSecond, null);

            builder.addDoubleProperty(
                "Robot Angle", () -> AllianceFlipUtil.apply(angle.get()).getRadians(), null);
          }
        });
  }

  public static void putPoseEstimate(Supplier<Pose2d> estimatedPose) {
    SmartDashboard.putData(
        "Field",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Field");

            builder.addDoubleArrayProperty(
                "Robot",
                () ->
                    new double[] {
                      estimatedPose.get().getX()
                      // - (fieldLength / 2)
                      ,
                      estimatedPose.get().getY()
                      // - (fieldWidth / 2)
                      ,
                      estimatedPose.get().getRotation().getDegrees()
                    },
                null);
          }
        });
  }

  private static Notification autoEndNotification =
      new Notification(NotificationLevel.INFO, "AUTO FINISHED", "n/a seconds", 5000);

  public static void sendAutoEndNotification(double time) {
    Elastic.sendNotification(autoEndNotification.withDescription(time + "seconds!"));
  }
}
