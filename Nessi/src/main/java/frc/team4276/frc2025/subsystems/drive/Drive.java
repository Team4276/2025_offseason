package frc.team4276.frc2025.subsystems.drive;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.RobotState;
import frc.team4276.util.AllianceFlipUtil;
import frc.team4276.util.dashboard.ElasticUI;
import frc.team4276.util.dashboard.LoggedTunableNumber;
import frc.team4276.util.dashboard.LoggedTunablePID;
import frc.team4276.util.hid.JoystickOutputController;
import frc.team4276.util.ios.GyroIO;
import frc.team4276.util.ios.GyroIOInputsAutoLogged;
import frc.team4276.util.ios.ModuleIO;
import frc.team4276.util.swerve.SwerveSetpointGenerator;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public enum WantedState {
    TELEOP,
    TRAJECTORY,
    HEADING_ALIGN,
    AUTO_ALIGN,
    IDLE,
    CHARACTERIZATION
  }

  public enum SystemState {
    TELEOP,
    TRAJECTORY,
    HEADING_ALIGN,
    AUTO_ALIGN,
    IDLE,
    CHARACTERIZATION
  }

  private WantedState wantedState = WantedState.TELEOP;
  private SystemState systemState = SystemState.TELEOP;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private BooleanSupplier gyroCalibrationSwitch;
  private final Timer gyroCalibrationTimer = new Timer();

  private SwerveModulePosition[] lastModulePositions = null;
  private double lastTime = 0.0;

  public enum DriveSpeedScalar {
    NORMAL(1.0, 0.65),
    CLIMB(0.5, 0.65),
    DEMO(0.1, 0.1);

    private final double linearVelocityScalar;
    private final double angularVelocityScalar;

    private DriveSpeedScalar(double linearVelocityScalar, double angularVelocityScalar) {
      this.linearVelocityScalar = linearVelocityScalar;
      this.angularVelocityScalar = angularVelocityScalar;
    }

    public double linearVelocityScalar() {
      return linearVelocityScalar;
    }

    public double angularVelocityScalar() {
      return angularVelocityScalar;
    }
  }

  private final JoystickOutputController controller;
  private DriveSpeedScalar driveSpeedScalar =
      Constants.isDemo ? DriveSpeedScalar.DEMO : DriveSpeedScalar.NORMAL;

  private final SwerveSetpointGenerator swerveSetpointGenerator =
      new SwerveSetpointGenerator(driveConfig, maxSteerVelocity);
  private SwerveSetpoint prevSetpoint;

  private final LoggedTunablePID teleopAutoAlignController =
      new LoggedTunablePID(3.0, 0, 0.1, 0.01, "Drive/AutoAlign/TeleopTranslation");
  private final LoggedTunablePID autoAutoAlignController =
      new LoggedTunablePID(2.0, 0, 0.0, 0.01, "Drive/AutoAlign/AutoTranslation");
  private final LoggedTunablePID headingAlignController =
      new LoggedTunablePID(5.0, 0, 0, Math.toRadians(1.0), "Drive/HeadingAlign");

  private Pose2d desiredAutoAlignPose = Pose2d.kZero;
  private final double autoAlignStaticFrictionConstant = maxVelocityMPS * 0.02;

  private Rotation2d desiredHeadingAlignRotation = Rotation2d.kZero;

  private double maxAutoAlignDriveTranslationOutput = maxVelocityMPS * 0.67;
  private double maxAutoAlignDriveRotationOutput = maxAngularVelocity * 0.67;

  private final LoggedTunablePID trajectoryXController =
      new LoggedTunablePID(4.0, 0, 0, 0.1, "Drive/Trajectory/Translation");
  private final LoggedTunablePID trajectoryYController =
      new LoggedTunablePID(4.0, 0, 0, 0.1, "Drive/Trajectory/Translation");
  private final LoggedTunablePID trajectoryThetaController =
      new LoggedTunablePID(3.0, 0, 0, Math.toRadians(1.0), "Drive/Trajectory/Rotation");

  private final LoggedTunableNumber maxError =
      new LoggedTunableNumber("Drive/Trajectory/maxError", 0.75);

  private Trajectory<SwerveSample> choreoTrajectory;
  private SwerveSample sampledTrajectoryState;

  private double startTime = 0.0;
  private double timeOffset = 0.0;

  private boolean enableTrajectoryFeedforward = false;
  private List<Vector<N2>> moduleForces =
      List.of(
          VecBuilder.fill(0.0, 0.0),
          VecBuilder.fill(0.0, 0.0),
          VecBuilder.fill(0.0, 0.0),
          VecBuilder.fill(0.0, 0.0));
  // private final double[] dummyForces = { 0.0, 0.0, 0.0, 0.0 };

  public Drive(
      JoystickOutputController controller,
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.controller = controller;
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    prevSetpoint =
        new SwerveSetpoint(new ChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));

    ElasticUI.putSwerveDrive(
        () -> getModuleStates(), () -> RobotState.getInstance().getEstimatedPose().getRotation());

    trajectoryThetaController.enableContinuousInput(-Math.PI, Math.PI);
    headingAlignController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      // Stop moving when disabled
      for (var module : modules) {
        module.stop();
      }

      // Log empty setpoint states when disabled
      Logger.recordOutput("Drive/SwerveStates/OptimizedSetpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/Torques", new SwerveModuleState[] {});
      if (Constants.isTuning) {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
          states[i] = modules[i].getZeroHelperModuleState();
        }
        Logger.recordOutput("Drive/SwerveStates/ZeroHelper", states);
      }

      if (gyroCalibrationSwitch.getAsBoolean()) {
        gyroCalibrationTimer.restart();
      }

      if (gyroCalibrationTimer.isRunning() && gyroCalibrationTimer.get() > 1.0) {
        gyroIO.recalibrate();
        gyroCalibrationTimer.stop();
      }
    }

    updateOdom();

    systemState = handleStateTransition();
    Logger.recordOutput("Drive/SystemState", systemState);
    Logger.recordOutput("Drive/DesiredState", wantedState);
    applyState();

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && !Constants.isSim);
  }

  private void updateOdom() {
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
      }

      boolean includeMeasurement = true;
      if (lastModulePositions != null) {

        double dt = sampleTimestamps[i] - lastTime;
        for (int j = 0; j < modules.length; j++) {
          double velocity =
              (modulePositions[j].distanceMeters - lastModulePositions[j].distanceMeters) / dt;
          double omega =
              modulePositions[j].angle.minus(lastModulePositions[j].angle).getRadians() / dt;
          // Check if delta is too large
          if (Math.abs(omega) > DriveConstants.maxVelocityMPS * 1.5
              || Math.abs(velocity) > DriveConstants.maxAngularVelocity * 1.5) {
            includeMeasurement = false;
            break;
          }
        }
      }

      // If delta isn't too large we can include the measurement.
      if (includeMeasurement) {
        lastModulePositions = modulePositions;
        RobotState.getInstance()
            .addOdometryObservation(
                sampleTimestamps[i],
                gyroInputs.connected ? gyroInputs.yawPosition : null,
                modulePositions);
        lastTime = sampleTimestamps[i];
        RobotState.getInstance().addDriveSpeeds(kinematics.toChassisSpeeds(getModuleStates()));
      }
    }
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case TELEOP -> SystemState.TELEOP;
      case TRAJECTORY -> {
        if (systemState != SystemState.TRAJECTORY) {
          resetTrajectoryTimer();
        }

        sampledTrajectoryState = choreoTrajectory.sampleAt(getTrajectoryTime(), false).get();

        yield SystemState.TRAJECTORY;
      }
      case HEADING_ALIGN -> SystemState.HEADING_ALIGN;
      case AUTO_ALIGN -> SystemState.AUTO_ALIGN;
      case CHARACTERIZATION -> SystemState.CHARACTERIZATION;
      default -> SystemState.IDLE;
    };
  }

  private void applyState() {
    ChassisSpeeds requestedSpeeds = new ChassisSpeeds();

    SwerveModuleState[] setpointTorques =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };

    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();

    Logger.recordOutput("RobotState/EstimatedPose", currentPose);
    Logger.recordOutput(
        "RobotState/EstimatedOdomPose", RobotState.getInstance().getEstimatedOdomPose());

    switch (systemState) {
      default:
        break;

      case TELEOP:
        requestedSpeeds = getJoystickRequestedSpeeds();

        break;

      case TRAJECTORY:
        if (sampledTrajectoryState
                .getPose()
                .getTranslation()
                .getDistance(currentPose.getTranslation())
            > maxError.getAsDouble()) {
          timeOffset += 0.02;

          // sampledTrajectoryState = new SwerveSample(
          // sampledTrajectoryState.getTimestamp(),
          // sampledTrajectoryState.x,
          // sampledTrajectoryState.y,
          // sampledTrajectoryState.heading,
          // sampledTrajectoryState.vx,
          // sampledTrajectoryState.vy,
          // sampledTrajectoryState.omega,
          // sampledTrajectoryState.ax,
          // sampledTrajectoryState.ay,
          // sampledTrajectoryState.alpha,
          // dummyForces,
          // dummyForces);
        }

        RobotState.getInstance().setTrajectorySetpoint(sampledTrajectoryState.getPose());

        requestedSpeeds = sampledTrajectoryState.getChassisSpeeds();

        requestedSpeeds.vxMetersPerSecond +=
            trajectoryXController.calculate(
                0.0, sampledTrajectoryState.x - currentPose.getTranslation().getX());
        requestedSpeeds.vyMetersPerSecond +=
            trajectoryYController.calculate(
                0.0, sampledTrajectoryState.y - currentPose.getTranslation().getY());
        requestedSpeeds.omegaRadiansPerSecond +=
            trajectoryThetaController.calculate(
                0.0,
                MathUtil.angleModulus(
                    sampledTrajectoryState
                        .getPose()
                        .getRotation()
                        .minus(currentPose.getRotation())
                        .getRadians()));

        if (enableTrajectoryFeedforward) {
          moduleForces = new ArrayList<>();
          for (int i = 0; i < 4; i++) {
            moduleForces.add(
                VecBuilder.fill(
                    sampledTrajectoryState.moduleForcesX()[i],
                    sampledTrajectoryState.moduleForcesY()[i]));
          }
        }

        Logger.recordOutput("Drive/Trajectory/SetpointPose", sampledTrajectoryState.getPose());
        Logger.recordOutput(
            "Drive/Trajectory/SetpointSpeeds", sampledTrajectoryState.getChassisSpeeds());
        Logger.recordOutput("Drive/Trajectory/TrajectoryTime", getTrajectoryTime());

        break;

      case HEADING_ALIGN:
        requestedSpeeds = getJoystickRequestedSpeeds();

        double thetaError =
            MathUtil.angleModulus(
                currentPose.getRotation().minus(desiredHeadingAlignRotation).getRadians());
        requestedSpeeds.omegaRadiansPerSecond = headingAlignController.calculate(thetaError, 0.0);

        if (headingAlignController.atSetpoint()) {
          requestedSpeeds.omegaRadiansPerSecond = 0.0;
        }

        Logger.recordOutput("Drive/HeadingAlign/TargetHeading", desiredHeadingAlignRotation);
        Logger.recordOutput("Drive/HeadingAlign/Error", thetaError);
        Logger.recordOutput("Drive/HeadingAlign/Output", requestedSpeeds.omegaRadiansPerSecond);

        break;

      case AUTO_ALIGN:
        Translation2d translationError =
            desiredAutoAlignPose.getTranslation().minus(currentPose.getTranslation());
        double translationLinearError = translationError.getNorm();
        double translationLinearOutput;

        if (translationLinearError < teleopAutoAlignController.getErrorTolerance()) {
          translationLinearOutput = 0.0;

        } else if (DriverStation.isAutonomous()) {
          translationLinearOutput =
              Math.abs(autoAutoAlignController.calculate(translationLinearError, 0.0))
                  + autoAlignStaticFrictionConstant;

        } else {
          translationLinearOutput =
              Math.abs(teleopAutoAlignController.calculate(translationLinearError, 0.0))
                  + autoAlignStaticFrictionConstant;
        }

        translationLinearOutput =
            Math.min(translationLinearOutput, maxAutoAlignDriveTranslationOutput);

        double vx = translationLinearOutput * translationError.getAngle().getCos();
        double vy = translationLinearOutput * translationError.getAngle().getSin();

        double autoAlignThetaError =
            MathUtil.angleModulus(
                currentPose.getRotation().minus(desiredAutoAlignPose.getRotation()).getRadians());
        double omega =
            Math.min(
                headingAlignController.calculate(autoAlignThetaError, 0.0),
                maxAutoAlignDriveRotationOutput);

        if (headingAlignController.atSetpoint()) {
          omega = 0.0;
        }

        requestedSpeeds = new ChassisSpeeds(vx, vy, omega);

        Logger.recordOutput("Drive/AutoAlign/Target", desiredAutoAlignPose);
        Logger.recordOutput("Drive/AutoAlign/TranslationError", translationLinearError);
        Logger.recordOutput("Drive/AutoAlign/ThetaError", autoAlignThetaError);
        Logger.recordOutput("Drive/AutoAlign/Output", requestedSpeeds);
        Logger.recordOutput("Drive/AutoAlign/LinearOutput", translationLinearOutput);

        break;

      case CHARACTERIZATION:
        break;
    }

    requestedSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(requestedSpeeds, currentPose.getRotation());

    SwerveModuleState[] setpointStates;
    ChassisSpeeds setpointSpeeds;

    if (systemState == SystemState.AUTO_ALIGN) {
      prevSetpoint = swerveSetpointGenerator.generateSetpoint(prevSetpoint, requestedSpeeds, 0.02);
      setpointSpeeds = prevSetpoint.robotRelativeSpeeds();
      setpointStates = prevSetpoint.moduleStates();

    } else {
      setpointSpeeds = ChassisSpeeds.discretize(requestedSpeeds, 0.02);
      setpointStates = kinematics.toSwerveModuleStates(setpointSpeeds);
    }

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drive/RequestedSpeeds", requestedSpeeds);
    Logger.recordOutput("Drive/SetpointSpeeds", setpointSpeeds);
    Logger.recordOutput(
        "Drive/SwerveStates/UnoptimizedSetpoints",
        kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(requestedSpeeds, 0.02)));
    Logger.recordOutput("Drive/SwerveStates/OptimizedSetpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveStates/Torques", setpointTorques);
  }

  private ChassisSpeeds getJoystickRequestedSpeeds() {
    double linearMagnitude =
        Math.hypot(-controller.getLeftWithDeadband().y, -controller.getLeftWithDeadband().x);

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    Translation2d linearVelocity = Translation2d.kZero;

    if (linearMagnitude > 1e-6) {
      linearVelocity =
          new Translation2d(
                  linearMagnitude,
                  new Rotation2d(
                      controller.getLeftWithDeadband().y, controller.getLeftWithDeadband().x))
              .times(driveSpeedScalar.linearVelocityScalar);
    }

    // Square rotation value for more precise control
    double omega =
        Math.copySign(
                controller.getRightWithDeadband().x * controller.getRightWithDeadband().x,
                -controller.getRightWithDeadband().x)
            * driveSpeedScalar.angularVelocityScalar;

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(
            linearVelocity.getX() * DriveConstants.maxVelocityMPS,
            linearVelocity.getY() * DriveConstants.maxVelocityMPS,
            omega * DriveConstants.maxAngularVelocity),
        AllianceFlipUtil.apply(Rotation2d.k180deg));
  }

  private double getTrajectoryTime() {
    return Timer.getTimestamp() - startTime - timeOffset;
  }

  private void resetTrajectoryTimer() {
    startTime = Timer.getTimestamp();
    timeOffset = 0.0;
  }

  public boolean isTrajectoryFinished() {
    return getTrajectoryTime() > choreoTrajectory.getTotalTime();
  }

  public boolean isAtAutoAlignPose() {
    return isAtPose(desiredAutoAlignPose);
  }

  public boolean isAtPose(Pose2d pose) {
    return isAtTranslation(pose.getTranslation()) && isHeadingAligned();
  }

  public boolean isAtTranslation(Translation2d trans) {
    return isAtTranslation(trans, teleopAutoAlignController.getErrorTolerance());
  }

  public boolean isAtTranslation(double tolerance) {
    return isAtTranslation(desiredAutoAlignPose.getTranslation(), tolerance);
  }

  public boolean isAtTranslation(Translation2d trans, double tolerance) {
    return RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(trans)
        < tolerance;
  }

  public boolean isHeadingAligned() {
    return isAtHeading(desiredHeadingAlignRotation, headingAlignController.getErrorTolerance());
  }

  public boolean isAtHeading() {
    return isAtHeading(desiredHeadingAlignRotation, headingAlignController.getErrorTolerance());
  }

  public boolean isAtHeading(Rotation2d heading, double tolerance) {
    return Math.abs(
            RobotState.getInstance().getEstimatedPose().getRotation().minus(heading).getRadians())
        < tolerance;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void setDriveSpeedScalar(DriveSpeedScalar driveSpeedScalar) {
    this.driveSpeedScalar = driveSpeedScalar;
  }

  public void setChoreoTrajectory(Trajectory<SwerveSample> trajectory) {
    setWantedState(WantedState.TRAJECTORY);
    choreoTrajectory = trajectory;
  }

  public void setAutoAlignPose(Pose2d pose) {
    setWantedState(WantedState.AUTO_ALIGN);
    desiredAutoAlignPose = pose;
  }

  public void setHeadingAlignRotation(Rotation2d rotation) {
    setWantedState(WantedState.HEADING_ALIGN);
    desiredHeadingAlignRotation = rotation;
  }

  public void setGyroCalibrationSwitch(BooleanSupplier calibrationSwitch) {
    this.gyroCalibrationSwitch = calibrationSwitch;
  }
}
