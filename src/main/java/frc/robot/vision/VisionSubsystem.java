package frc.robot.vision;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.auto_align.ReefPipe;
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
  private final Debouncer hasSeenTagDisabledDebouncer = new Debouncer(0.5, DebounceType.kFalling);
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
  private ReefPipe reefPipe;

  private boolean hasSeenTag = false;

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
    robotHeading = imu.getRobotHeading();
    angularVelocity = imu.getRobotAngularVelocity();
    pitch = imu.getPitch();
    pitchRate = imu.getPitchRate();
    roll = imu.getRoll();
    rollRate = imu.getRollRate();

    if (getState() == VisionState.CLOSEST_REEF_TAG_CLOSEUP) {
      switch (reefPipe) {
        case PIPE_A, PIPE_C, PIPE_E, PIPE_G, PIPE_I, PIPE_K -> {
          if (leftBackLimelight.getCameraHealth() != CameraHealth.OFFLINE) {
            leftFrontLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
            leftFrontLimelight.setState(LimelightState.OFF);
          }
        }
        case PIPE_B, PIPE_D, PIPE_F, PIPE_H, PIPE_J, PIPE_L -> {
          if (leftFrontLimelight.getCameraHealth() != CameraHealth.OFFLINE) {
            leftFrontLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
            leftBackLimelight.setState(LimelightState.OFF);
          }
        }
      }
    }

    leftBackTagResult = leftBackLimelight.getTagResult();
    leftFrontTagResult = leftFrontLimelight.getTagResult();
    rightTagResult = rightLimelight.getTagResult();

    hasSeenTag =
        leftBackTagResult.isPresent()
            || leftFrontTagResult.isPresent()
            || rightTagResult.isPresent();
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

  public boolean hasSeenTagRecentlyDisabled() {
    return hasSeenTagDisabledDebouncer.calculate(hasSeenTag);
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
      case CLOSEST_REEF_TAG_CLOSEUP -> {
        leftBackLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
        leftFrontLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
        rightLimelight.setState(LimelightState.CLOSEST_REEF_TAG_CLOSEUP);
        gamePieceDetectionLimelight.setState(LimelightState.CORAL);
      }
      case STATION_TAGS -> {
        leftBackLimelight.setState(LimelightState.STATION_TAGS);
        leftFrontLimelight.setState(LimelightState.STATION_TAGS);
        rightLimelight.setState(LimelightState.STATION_TAGS);
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
        leftBackLimelight.setState(LimelightState.TAGS);
        leftFrontLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.ALGAE);
        gamePieceDetectionLimelight.setState(LimelightState.ALGAE);
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

    leftBackLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    leftFrontLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    rightLimelight.sendImuData(robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);

    if (FeatureFlags.CAMERA_POSITION_CALIBRATION.getAsBoolean()) {
      setStateFromRequest(VisionState.TAGS);
      leftBackLimelight.logCameraPositionCalibrationValues();
      leftFrontLimelight.logCameraPositionCalibrationValues();
      rightLimelight.logCameraPositionCalibrationValues();
      gamePieceDetectionLimelight.logCameraPositionCalibrationValues();
    }
  }

  public void setClosestScoringReefAndPipe(int tagID, ReefPipe currentScoringPipe) {
    reefPipe = currentScoringPipe;
    leftFrontLimelight.setClosestScoringReefTag(tagID);
    rightLimelight.setClosestScoringReefTag(tagID);
    leftBackLimelight.setClosestScoringReefTag(tagID);
  }

  public boolean isAnyCameraOffline() {
    return leftBackLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || leftFrontLimelight.getCameraHealth() == CameraHealth.OFFLINE
        || rightLimelight.getCameraHealth() == CameraHealth.OFFLINE;
    // || gamePieceDetectionLimelight.getCameraHealth() == CameraHealth.OFFLINE;

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

  public void updateDistanceFromReef(double distanceFromReef) {
    DogLog.log("Vision/DistanceFromReef", distanceFromReef);
    if (getState() == VisionState.CLOSEST_REEF_TAG) {
      if (distanceFromReef < REEF_CLOSEUP_DISTANCE) {
        setState(VisionState.CLOSEST_REEF_TAG_CLOSEUP);
      }
    }
  }
}
