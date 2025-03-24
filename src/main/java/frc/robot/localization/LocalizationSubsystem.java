package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.MathHelpers;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.results.TagResult;
import java.util.ArrayList;
import java.util.List;

public class LocalizationSubsystem extends StateMachine<LocalizationState> {
  private static final Vector<N3> VISION_STD_DEVS =
      VecBuilder.fill(
          RobotConfig.get().vision().xyStdDev(),
          RobotConfig.get().vision().xyStdDev(),
          RobotConfig.get().vision().thetaStdDev());
  private final ImuSubsystem imu;
  private final VisionSubsystem vision;
  private final SwerveSubsystem swerve;
  private List<TagResult> latestResult = new ArrayList<>();

  public LocalizationSubsystem(ImuSubsystem imu, VisionSubsystem vision, SwerveSubsystem swerve) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.imu = imu;
    this.vision = vision;

    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      SmartDashboard.putData(
          "Debug/ResetGyroTo180",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(180))).ignoringDisable(true));
      SmartDashboard.putData(
          "Debug/ResetGyroTo0",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(0))).ignoringDisable(true));
    }
  }

  @Override
  protected void collectInputs() {
    latestResult = vision.getTagResult();
  }

  public Pose2d getPose() {
    return swerve.getDrivetrainState().Pose;
  }

  public Pose2d getPose(double timestamp) {
    var newTimestamp = Utils.fpgaToCurrentTime(timestamp);
    return swerve.drivetrain.samplePoseAt(newTimestamp).orElseGet(this::getPose);
  }

  public Pose2d getLookaheadPose(double lookahead) {
    return MathHelpers.poseLookahead(getPose(), swerve.getFieldRelativeSpeeds(), lookahead);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    for (var results : latestResult) {
      Pose2d visionPose = results.pose();

      double visionTimestamp = Utils.fpgaToCurrentTime(results.timestamp());
      if (DriverStation.isDisabled()) {
        resetPose(visionPose);
      }
      swerve.drivetrain.addVisionMeasurement(visionPose, visionTimestamp, VISION_STD_DEVS);
    }

    DogLog.log("Localization/EstimatedPose", getPose());
  }

  private void resetGyro(Rotation2d gyroAngle) {
    swerve.drivetrain.resetRotation(gyroAngle);
  }

  public void resetPose(Pose2d estimatedPose) {
    // Reset the gyro when requested in teleop
    // Otherwise, if we are in auto, only reset it if we aren't already at the correct heading
    if (DriverStation.isTeleop()
        || !MathUtil.isNear(
            estimatedPose.getRotation().getDegrees(), imu.getRobotHeading(), 1.5, -180, 180)) {
      imu.setAngle(estimatedPose.getRotation().getDegrees());
    }

    swerve.drivetrain.resetPose(estimatedPose);
  }

  public Command getZeroCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees((FmsSubsystem.isRedAlliance() ? 180 : 0))));
  }
}
