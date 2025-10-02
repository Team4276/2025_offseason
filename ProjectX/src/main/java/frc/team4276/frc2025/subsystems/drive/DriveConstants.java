package frc.team4276.frc2025.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.Ports;

public class DriveConstants {
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(26.0);
  public static final double wheelBase = Units.inchesToMeters(26.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);

  public static final double maxSpeed = 5.3;
  public static final double maxAccel = 6.4;
  public static final double maxAngularSpeed = 11.4;
  public static final double maxAngularAccel = 33.9;

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(1.52507);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0318);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.034);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(2.5579);

  public static final Rotation2d frontLeftZeroHelperRotation = Rotation2d.kCCW_90deg;
  public static final Rotation2d frontRightZeroHelperRotation = Rotation2d.kZero;
  public static final Rotation2d backLeftZeroHelperRotation = Rotation2d.k180deg;
  public static final Rotation2d backRightZeroHelperRotation = Rotation2d.kCW_90deg;

  // Device CAN IDs
  public static final int frontLeftDriveCanId = Ports.FRONT_LEFT_DRIVE;
  public static final int frontRightDriveCanId = Ports.FRONT_RIGHT_DRIVE;
  public static final int backLeftDriveCanId = Ports.BACK_LEFT_DRIVE;
  public static final int backRightDriveCanId = Ports.BACK_RIGHT_DRIVE;

  public static final int frontLeftTurnCanId = Ports.FRONT_LEFT_TURN;
  public static final int frontRightTurnCanId = Ports.FRONT_RIGHT_TURN;
  public static final int backLeftTurnCanId = Ports.BACK_LEFT_TURN;
  public static final int backRightTurnCanId = Ports.BACK_RIGHT_TURN;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
  public static final double drivingMotorPinionTeeth = 13.0;
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (drivingMotorPinionTeeth * 15.0); // MAXSwerve
  // with
  // x pinion teeth
  // and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);
  public static final double maxSteerVelocity =
      driveGearbox.freeSpeedRadPerSec / driveMotorReduction;

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM
  // ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.005;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.177;
  public static final double driveKv =
      12.0 / (driveGearbox.freeSpeedRadPerSec / driveMotorReduction);
  public static final double driveSimP = 1.0;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.028;
  public static final double driveSimKv = 0.1;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.5;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 56.699;
  public static final double robotMOI = 5.267513460399;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig driveConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeed,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  public static final double ffkT = 1.0 / DCMotor.getNeoVortex(1).KtNMPerAmp;
}
