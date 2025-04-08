package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.FeatureFlags;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.vision.results.GamePieceResult;
import frc.robot.vision.results.TagResult;
import java.util.Optional;
import java.util.OptionalDouble;

public class VisionSubsystem extends StateMachine<VisionState> {
  private static final double REEF_CLOSEUP_DISTANCE = 0.7;
  private final Debouncer seeingTagDebouncer = new Debouncer(1.0, DebounceType.kFalling);
  private final Debouncer seeingTagForPoseResetDebouncer =
      new Debouncer(5.0, DebounceType.kFalling);

  private final ImuSubsystem imu;
  private final Limelight leftBackLimelight;
  private final Limelight leftFrontLimelight;
  private final Limelight rightLimelight;
  private final Limelight gamePieceDetectionLimelight;

  private Optional<TagResult> leftBackTagResult = Optional.empty();
  private Optional<TagResult> leftFrontTagResult = Optional.empty();
  private Optional<TagResult> rightTagResult = Optional.empty();

  private double robotHeading;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;

  private boolean hasSeenTag = false;
  private boolean seeingTag = false;
  private boolean seeingTagDebounced = false;
  private boolean seenTagRecentlyForReset = true;

  public VisionSubsystem(
      ImuSubsystem imu,
      Limelight leftBackLimelight,
      Limelight leftFrontLimelight,
      Limelight rightLimelight,
      Limelight gamePieceDetectionLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.leftBackLimelight = leftBackLimelight;
    this.leftFrontLimelight = leftFrontLimelight;
    this.rightLimelight = rightLimelight;
    this.gamePieceDetectionLimelight = gamePieceDetectionLimelight;
  }

  @Override
  protected void collectInputs() {
    angularVelocity = imu.getRobotAngularVelocity();

    leftBackTagResult = leftBackLimelight.getTagResult();
    leftFrontTagResult = leftFrontLimelight.getTagResult();
    rightTagResult = rightLimelight.getTagResult();

    if (leftBackTagResult.isPresent()
        || leftFrontTagResult.isPresent()
        || rightTagResult.isPresent()) {
      hasSeenTag = true;
      seeingTag = true;
    } else {
      seeingTag = false;
    }
    seeingTagDebounced = seeingTagDebouncer.calculate(seeingTag);
    if (DriverStation.isDisabled()) {
      seenTagRecentlyForReset = true;
    } else {
      seenTagRecentlyForReset = seeingTagForPoseResetDebouncer.calculate(seeingTag);
    }
  }

  public void setEstimatedPoseAngle(double robotHeading) {
    this.robotHeading = robotHeading;
  }

  public Optional<TagResult> getLeftBackTagResult() {
    return leftBackTagResult;
  }

  public Optional<TagResult> getLeftFrontTagResult() {
    return leftFrontTagResult;
  }

  public Optional<TagResult> getRightTagResult() {
    return rightTagResult;
  }

  public boolean seeingTagDebounced() {
    return seeingTagDebounced;
  }

  public boolean seenTagRecentlyForReset() {
    return seenTagRecentlyForReset;
  }

  public boolean seeingTag() {
    return seeingTag;
  }

  public boolean hasSeenTag() {
    return hasSeenTag;
  }

  public void setState(VisionState state) {
    setStateFromRequest(state);
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        leftBackLimelight.setState(LimelightState.TAGS);
        leftFrontLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);
        gamePieceDetectionLimelight.setState(LimelightState.CORAL);
      }
      case CLOSEST_REEF_TAG -> {
        leftBackLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        leftFrontLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        rightLimelight.setState(LimelightState.CLOSEST_REEF_TAG);
        gamePieceDetectionLimelight.setState(LimelightState.CORAL);
      }
      case CORAL_DETECTION -> {
        leftBackLimelight.setState(LimelightState.TAGS);
        leftFrontLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);
        gamePieceDetectionLimelight.setState(LimelightState.CORAL);
      }
      case HANDOFF -> {
        leftBackLimelight.setState(LimelightState.TAGS);
        leftFrontLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);
        gamePieceDetectionLimelight.setState(LimelightState.HELD_CORAL);
      }
      case ALGAE_DETECTION -> {
        leftBackLimelight.setState(LimelightState.OFF);
        leftFrontLimelight.setState(LimelightState.OFF);
        rightLimelight.setState(LimelightState.ALGAE);
        gamePieceDetectionLimelight.setState(LimelightState.OFF);
      }
    }
  }

  public Optional<GamePieceResult> getLollipopVisionResult() {
    return rightLimelight.getAlgaeResult();
  }

  public OptionalDouble getHandoffOffsetTx() {
    return gamePieceDetectionLimelight.handoffTx();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    leftBackLimelight.sendImuData(robotHeading, angularVelocity, 0.0, 0.0, 0.0, 0.0);
    leftFrontLimelight.sendImuData(robotHeading, angularVelocity, 0.0, 0.0, 0.0, 0.0);
    rightLimelight.sendImuData(robotHeading, angularVelocity, 0.0, 0.0, 0.0, 0.0);

    if (FeatureFlags.CAMERA_POSITION_CALIBRATION.getAsBoolean()) {
      setStateFromRequest(VisionState.TAGS);
      leftBackLimelight.logCameraPositionCalibrationValues();
      leftFrontLimelight.logCameraPositionCalibrationValues();
      rightLimelight.logCameraPositionCalibrationValues();
      gamePieceDetectionLimelight.logCameraPositionCalibrationValues();
    }

    DogLog.log("Vision/SeeingTag", seeingTag);
    DogLog.log("Vision/SeeingTagLast5Seconds", seenTagRecentlyForReset);
  }

  public void setClosestScoringReefAndPipe(int tagID) {
    leftFrontLimelight.setClosestScoringReefTag(tagID);
    rightLimelight.setClosestScoringReefTag(tagID);
    leftBackLimelight.setClosestScoringReefTag(tagID);
  }

  public boolean isAnyCameraOffline() {
    return leftBackLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || leftFrontLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || rightLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || gamePieceDetectionLimelight.getCameraHealth() == CameraHealth.OFFLINE;
  }

  public boolean isAnyLeftScoringTagLimelightOnline() {
    return leftBackLimelight.isOnlineForTags() || leftFrontLimelight.isOnlineForTags();
  }

  public boolean isAnyRightScoringTagLimelightOnline() {
    return rightLimelight.isOnlineForTags();
  }

  public boolean isAnyTagLimelightOnline() {
    return leftBackLimelight.isOnlineForTags()
        || leftFrontLimelight.isOnlineForTags()
        || rightLimelight.isOnlineForTags();
  }
}
