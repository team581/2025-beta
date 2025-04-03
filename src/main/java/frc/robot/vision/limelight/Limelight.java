package frc.robot.vision.limelight;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.CameraHealth;
import frc.robot.vision.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.vision.results.GamePieceResult;
import frc.robot.vision.results.TagResult;
import java.util.Optional;
import java.util.OptionalDouble;

public class Limelight extends StateMachine<LimelightState> {
  private static final int[] VALID_APRILTAGS =
      new int[] {1, 2, 6, 7, 8, 9, 10, 11, 12, 13, 17, 18, 19, 20, 21, 22};

  private static final double IS_OFFLINE_TIMEOUT = 3;

  private final String limelightTableName;
  private final String name;
  private final LimelightModel limelightModel;

  private final Timer limelightTimer = new Timer();
  private final Timer seedImuTimer = new Timer();
  private CameraHealth cameraHealth = CameraHealth.NO_TARGETS;
  private double limelightHeartbeat = -1;

  private double lastTimestamp = 0.0;

  private Optional<TagResult> tagResult = Optional.empty();

  private Optional<GamePieceResult> coralResult = Optional.empty();
  private Optional<GamePieceResult> algaeResult = Optional.empty();

  private final int[] closestScoringReefTag = {0};

  public Limelight(String name, LimelightState initialState, LimelightModel limelightModel) {
    // TODO(jonahsnider): Make Limelight state logging work with multiple instances, not just
    // singleton
    super(SubsystemPriority.VISION, initialState);
    limelightTableName = "limelight-" + name;
    this.name = name;
    limelightTimer.start();
    this.limelightModel = limelightModel;
  }

  public void sendImuData(
      double robotHeading,
      double angularVelocity,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    LimelightHelpers.SetRobotOrientation(
        limelightTableName, robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
  }

  public void setState(LimelightState state) {
    setStateFromRequest(state);
  }

  public Optional<GamePieceResult> getCoralResult() {
    return getState() == LimelightState.CORAL ? coralResult : Optional.empty();
  }

  public Optional<GamePieceResult> getAlgaeResult() {
    return getState() == LimelightState.ALGAE ? algaeResult : Optional.empty();
  }

  public Optional<TagResult> getTagResult() {
    if (getState() != LimelightState.TAGS
        && getState() != LimelightState.CLOSEST_REEF_TAG
        && getState() != LimelightState.CLOSEST_REEF_TAG_CLOSEUP) {
      return Optional.empty();
    }

    PoseEstimate estimatePose;
    if (FeatureFlags.CONTEXT_BASED_MEGATAG_1.getAsBoolean()
        && (DriverStation.isDisabled() || getState() == LimelightState.CLOSEST_REEF_TAG_CLOSEUP)) {
      estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightTableName);
    } else {
      estimatePose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightTableName);
    }

    if (estimatePose == null) {
      return Optional.empty();
    }

    if (FeatureFlags.VISION_STALE_DATA_CHECK.getAsBoolean()) {
      var newTimestamp = estimatePose.timestampSeconds;
      if (newTimestamp == lastTimestamp) {
        return Optional.empty();
      }

      lastTimestamp = newTimestamp;
    }
    var newPose = estimatePose.pose;

    if (estimatePose.tagCount == 0) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);

      return Optional.empty();
    }
    // This prevents pose estimator from having crazy poses if the Limelight loses power
    if (newPose.getX() == 0.0 && newPose.getY() == 0.0) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);

      return Optional.empty();
    }

    DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", newPose);
    return Optional.of(new TagResult(estimatePose.pose, estimatePose.timestampSeconds));
  }

  private Optional<GamePieceResult> getRawCoralResult() {
    if (getState() != LimelightState.CORAL) {
      return Optional.empty();
    }
    var t2d = LimelightHelpers.getT2DArray(limelightTableName);
    if (t2d.length == 0) {
      return Optional.empty();
    }
    var coralTx = t2d[4];
    var coralTy = t2d[5];
    if (coralTx == 0.0 || coralTy == 0.0) {
      return Optional.empty();
    }

    DogLog.log("Vision/" + name + "/Coral/tx", coralTx);
    DogLog.log("Vision/" + name + "/Coral/ty", coralTy);

    var latency = t2d[2] + t2d[3];
    var latencySeconds = latency / 1000.0;
    var timestamp = Timer.getFPGATimestamp() - latencySeconds;

    return Optional.of(new GamePieceResult(coralTx, coralTy, timestamp));
  }

  public OptionalDouble handoffTx() {
    if (getState() != LimelightState.HELD_CORAL) {
      return OptionalDouble.empty();
    }

    var t2d = LimelightHelpers.getT2DArray(limelightTableName);

    if (t2d.length != 17) {
      return OptionalDouble.empty();
    }
    var tv = t2d[0];

    if (tv == 0) {
      return OptionalDouble.empty();
    }

    var tx = t2d[4];
    if (tx == 0.0) {
      return OptionalDouble.empty();
    }

    return OptionalDouble.empty();
  }

  private Optional<GamePieceResult> getRawAlgaeResult() {
    if (getState() != LimelightState.ALGAE) {
      return Optional.empty();
    }
    var t2d = LimelightHelpers.getT2DArray(limelightTableName);
    if (t2d.length == 0) {
      return Optional.empty();
    }
    var algaeTx = t2d[4];
    var algaeTy = t2d[5];
    if (algaeTx == 0.0 || algaeTy == 0.0) {
      return Optional.empty();
    }

    DogLog.log("Vision/" + name + "/Algae/tx", algaeTx);
    DogLog.log("Vision/" + name + "/Algae/ty", algaeTy);

    var latency = t2d[2] + t2d[3];
    var latencySeconds = latency / 1000.0;
    var timestamp = Timer.getFPGATimestamp() - latencySeconds;

    return Optional.of(new GamePieceResult(algaeTx, algaeTy, timestamp));
  }

  public void setClosestScoringReefTag(int tagID) {
    closestScoringReefTag[0] = tagID;
  }

  @Override
  protected void collectInputs() {
    tagResult = getTagResult();
    coralResult = getRawCoralResult();
    algaeResult = getRawAlgaeResult();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Vision/" + name + "/State", getState());
    LimelightHelpers.setPipelineIndex(limelightTableName, getState().pipelineIndex);
    switch (getState()) {
      case TAGS -> {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, VALID_APRILTAGS);
        updateHealth(tagResult);
      }
      case CORAL -> updateHealth(coralResult);
      case ALGAE -> updateHealth(algaeResult);
      case HELD_CORAL -> updateHealth(coralResult);
      case CLOSEST_REEF_TAG, CLOSEST_REEF_TAG_CLOSEUP -> {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, closestScoringReefTag);
        updateHealth(tagResult);
      }
    }

    // TODO: Remove once Limelights are upgraded
    LimelightHelpers.SetIMUMode(limelightTableName, 0);
    // if (limelightModel == LimelightModel.FOUR) {
    //   LimelightHelpers.SetIMUMode(limelightTableName, seedIMUTimer.hasElapsed(2.0) ? 4 : 3);
    // } else {
    //   // TODO: Can remove once we have upgraded all the Limelights
    //   LimelightHelpers.SetIMUMode(limelightTableName, 0);
    // }
  }

  @Override
  public void autonomousInit() {
    seedImuTimer.reset();
    seedImuTimer.start();
  }

  private void updateHealth(Optional<?> result) {
    var newHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightTableName, "hb");
    DogLog.log("Vision/" + name + "/Heartbeat", newHeartbeat);
    if (limelightHeartbeat != newHeartbeat) {
      limelightTimer.restart();
    }
    limelightHeartbeat = newHeartbeat;

    if (limelightTimer.hasElapsed(IS_OFFLINE_TIMEOUT) && RobotBase.isReal()) {
      cameraHealth = CameraHealth.OFFLINE;
      DogLog.logFault(limelightTableName + " is offline", AlertType.kError);
      return;
    } else {
      DogLog.clearFault(limelightTableName + " is offline");
    }

    if (!result.isEmpty()) {
      cameraHealth = CameraHealth.GOOD;
      return;
    }
    cameraHealth = CameraHealth.NO_TARGETS;
  }

  public void setBlinkEnabled(boolean enabled) {
    if (enabled) {
      LimelightHelpers.setLEDMode_ForceBlink(limelightTableName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(limelightTableName);
    }
  }

  public CameraHealth getCameraHealth() {
    DogLog.log("Vision/" + name + "/Health", cameraHealth);
    return cameraHealth;
  }

  public void logCameraPositionCalibrationValues() {
    var cameraPoseTargetSpace = LimelightHelpers.getCameraPose3d_TargetSpace(limelightTableName);
    var robotPoseTargetSpace = RobotConfig.get().vision().robotPoseRelativeToCalibration();
    var cameraRobotRelativePose =
        getRobotRelativeCameraPosition(robotPoseTargetSpace, cameraPoseTargetSpace);
    DogLog.log("CameraPositionCalibration/" + name + "/LL Right", cameraRobotRelativePose.getX());
    DogLog.log("CameraPositionCalibration/" + name + "/LL Up", cameraRobotRelativePose.getY());
    DogLog.log("CameraPositionCalibration/" + name + "/LL Forward", cameraRobotRelativePose.getZ());
    DogLog.log(
        "CameraPositionCalibration/" + name + "/LL Roll",
        Units.radiansToDegrees(cameraRobotRelativePose.getRotation().getX()));
    DogLog.log(
        "CameraPositionCalibration/" + name + "/LL Pitch",
        Units.radiansToDegrees(cameraRobotRelativePose.getRotation().getY()));
    DogLog.log(
        "CameraPositionCalibration/" + name + "/LL Yaw",
        Units.radiansToDegrees(cameraRobotRelativePose.getRotation().getZ()));
  }

  public boolean isOnlineForTags() {
    return switch (getState()) {
      case TAGS, CLOSEST_REEF_TAG, CLOSEST_REEF_TAG_CLOSEUP ->
          getCameraHealth() != CameraHealth.OFFLINE;
      default -> false;
    };
  }

  private static Pose3d getRobotRelativeCameraPosition(
      Pose3d robotPoseTargetSpace, Pose3d seenCameraPoseTargetSpace) {
    // Positive X = Right
    var cameraLeftRight = seenCameraPoseTargetSpace.getX();
    // Positive Y = Down, so flipped for common sense
    var cameraUpDown = -1 * seenCameraPoseTargetSpace.getY();
    // Positive Z = Forward
    var cameraForwardBackward = seenCameraPoseTargetSpace.getZ();
    // Pitch rotates around left right axis (x according to LL coordinate systems)
    var cameraPitch = seenCameraPoseTargetSpace.getRotation().getX();
    // Roll rotates around forward backward axis (Z according to LL coordinate systems)
    var cameraRoll = seenCameraPoseTargetSpace.getRotation().getZ();
    // Yaw rotates around up down axis (y according to LL coordinate systems)
    var cameraYaw = -1 * seenCameraPoseTargetSpace.getRotation().getY();

    var robotLeftRight = robotPoseTargetSpace.getX();
    var robotUpDown = robotPoseTargetSpace.getY();
    var robotForwardBackward = robotPoseTargetSpace.getZ();
    var robotPitch = robotPoseTargetSpace.getRotation().getY();
    var robotRoll = robotPoseTargetSpace.getRotation().getX();
    var robotYaw = robotPoseTargetSpace.getRotation().getZ();

    var right = cameraLeftRight - robotLeftRight;
    var up = cameraUpDown - robotUpDown;
    var forward = cameraForwardBackward - robotForwardBackward;
    var roll = cameraRoll - robotRoll;
    var pitch = cameraPitch - robotPitch;
    var yaw = cameraYaw - robotYaw;

    return new Pose3d(right, up, forward, new Rotation3d(roll, pitch, yaw));
  }
}
