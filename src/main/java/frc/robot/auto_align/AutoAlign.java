package frc.robot.auto_align;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto_align.tag_align.TagAlign;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.collision_avoidance.ObstructionKind;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.MathHelpers;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;

public class AutoAlign extends StateMachine<AutoAlignState> {
  private static final double TELEOP_SPEED_SCALAR = 0.3;
  private static final double MAX_CONSTRAINT = 2.5;

  private static final Translation2d CENTER_OF_REEF_RED =
      new Translation2d(Units.inchesToMeters(514.13), Units.inchesToMeters(158.5));
  private static final Translation2d CENTER_OF_REEF_BLUE =
      new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.5));

  private static final DoubleSubscriber TIME_TO_RAISE_ARM_FORWARD =
      DogLog.tunable("AutoAlign/ArmForwardRaiseTime", 0.2);
  private static final DoubleSubscriber SAFE_ARM_FORWARD_DISTANCE_FROM_REEF_SIDE =
      DogLog.tunable("AutoAlign/SafeReefDistanceArmForward", 1.3);

  public static RobotScoringSide getNetScoringSideFromRobotPose(Pose2d robotPose) {
    double robotX = robotPose.getX();
    double theta = MathHelpers.angleModulus(robotPose.getRotation().getDegrees());
    DogLog.log("Debug/Theta", theta);

    // entire field length is 17.55m
    double halfFieldLength = 17.55 / 2.0;

    // Robot is on blue side
    if (robotX < halfFieldLength) {
      if (theta > 0.0) {
        return RobotScoringSide.RIGHT;
      }
      return RobotScoringSide.LEFT;
    }

    // Robot is on red side
    if (theta > 0.0) {
      return RobotScoringSide.LEFT;
    }
    return RobotScoringSide.RIGHT;
  }

  public static Translation2d getAllianceCenterOfReef() {
    return FmsSubsystem.isRedAlliance() ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
  }

  public static RobotScoringSide getScoringSideFromRobotPose(
      Pose2d robotPose, boolean leftLimelightsOnline, boolean rightLimelightOnline) {
    if (!leftLimelightsOnline) {
      return RobotScoringSide.RIGHT;
    }
    if (!rightLimelightOnline) {
      return RobotScoringSide.LEFT;
    }
    var centerOfReef = getAllianceCenterOfReef();
    var angleToAim =
        MathUtil.angleModulus(
            Math.atan2(
                centerOfReef.getY() - robotPose.getY(), centerOfReef.getX() - robotPose.getX()));
    var errorRight =
        Math.abs(
            MathUtil.angleModulus(
                angleToAim - (robotPose.getRotation().getRadians() - (Math.PI / 2.0))));
    var errorLeft =
        Math.abs(
            MathUtil.angleModulus(
                angleToAim - (robotPose.getRotation().getRadians() + (Math.PI / 2.0))));

    if (errorRight < errorLeft) {
      return RobotScoringSide.RIGHT;
    }
    return RobotScoringSide.LEFT;
  }

  public static boolean isCloseToReefSide(
      Pose2d robotPose, Pose2d nearestReefSide, double thresholdMeters) {
    return robotPose.getTranslation().getDistance(nearestReefSide.getTranslation())
        < thresholdMeters;
  }

  public static boolean isCloseToReefSide(Pose2d robotPose, Pose2d nearestReefSide) {
    return isCloseToReefSide(robotPose, nearestReefSide, LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE);
  }

  private static final double LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE = 1.2;

  private final Debouncer isAlignedDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private final VisionSubsystem vision;
  private final LocalizationSubsystem localization;
  private final TagAlign tagAlign;
  private final SwerveSubsystem swerve;

  private ChassisSpeeds tagAlignSpeeds = new ChassisSpeeds();
  private ChassisSpeeds algaeAlignSpeeds = new ChassisSpeeds();
  private boolean isAligned = false;
  private boolean isAlignedDebounced = false;
  private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
  private ReefPipe bestReefPipe = ReefPipe.PIPE_A;
  private Pose2d usedScoringPose = Pose2d.kZero;
  private ReefSideOffset reefSideOffset = ReefSideOffset.BASE;

  public AutoAlign(
      VisionSubsystem vision, LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.AUTO_ALIGN, AutoAlignState.DEFAULT_STATE);

    this.tagAlign = new TagAlign(swerve, localization);
    this.vision = vision;
    this.localization = localization;
    this.swerve = swerve;
  }

  public void setAutoReefPipeOverride(ReefPipe override) {
    tagAlign.setPipeOveride(override);
  }

  public ReefSide getClosestReefSide() {
    return ReefSide.fromPipe(bestReefPipe);
  }

  @Override
  protected void collectInputs() {
    bestReefPipe = tagAlign.getBestPipe();
    usedScoringPose = tagAlign.getUsedScoringPose(bestReefPipe);
    isAligned = tagAlign.isAligned(bestReefPipe);
    isAlignedDebounced = isAlignedDebouncer.calculate(isAligned);
    tagAlignSpeeds = tagAlign.getPoseAlignmentChassisSpeeds(usedScoringPose);
    algaeAlignSpeeds =
        tagAlign.getPoseAlignmentChassisSpeeds(
            ReefSide.fromPipe(bestReefPipe).getPose(reefSideOffset, robotScoringSide));
    var controllerValues = swerve.getControllerValues();
    tagAlign.setControllerValues(controllerValues.getX(), controllerValues.getY());
  }

  public ObstructionKind shouldArmGoAroundToScore() {
    // Account for distance we'll be at once we finish forward motion
    var lookaheadPose = localization.getLookaheadPose(TIME_TO_RAISE_ARM_FORWARD.get());
    var lookaheadPoseDistance =
        lookaheadPose
            .getTranslation()
            .getDistance(
                bestReefPipe.getPose(ReefPipeLevel.BASE, robotScoringSide).getTranslation());
    if (lookaheadPoseDistance < SAFE_ARM_FORWARD_DISTANCE_FROM_REEF_SIDE.get()) {
      return robotScoringSide == RobotScoringSide.RIGHT
          ? ObstructionKind.RIGHT_OBSTRUCTED
          : ObstructionKind.LEFT_OBSTRUCTED;
    }
    return ObstructionKind.NONE;
  }

  public ChassisSpeeds getTagAlignSpeeds() {
    return tagAlignSpeeds;
  }

  public ChassisSpeeds getAlgaeAlignSpeeds() {
    return algaeAlignSpeeds;
  }

  public ReefPipe getBestReefPipe() {
    return bestReefPipe;
  }

  public void markPipeScored() {
    tagAlign.markScored(bestReefPipe);
  }

  public void setScoringLevel(
      ReefPipeLevel level, ReefPipeLevel preferredLevel, RobotScoringSide side) {
    robotScoringSide = side;
    tagAlign.setLevel(level, preferredLevel, side);
  }

  public void setAlgaeIntakingOffset(ReefSideOffset offset) {
    reefSideOffset = offset;
  }

  public void clearReefState() {
    tagAlign.clearReefState();
  }

  public boolean isTagAlignedDebounced() {
    return isAlignedDebounced;
  }

  public Pose2d getUsedScoringPose() {
    return usedScoringPose;
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe) {
    return tagAlign.getUsedScoringPose(pipe);
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe, ReefPipeLevel level, RobotScoringSide side) {
    if (DriverStation.isAutonomous()) {

      setScoringLevel(level, level, side);
      return getUsedScoringPose(pipe);
    }
    return usedScoringPose;
  }

  public ReefAlignState getReefAlignState() {
    if (!vision.isAnyLeftScoringTagLimelightOnline()
        && !vision.isAnyRightScoringTagLimelightOnline()) {
      return ReefAlignState.ALL_CAMERAS_DEAD;
    }

    if (vision.getLeftBackTagResult().isPresent()
        || vision.getLeftFrontTagResult().isPresent()
        || vision.getRightTagResult().isPresent()) {
      if (isAligned) {
        return ReefAlignState.HAS_TAGS_IN_POSITION;
      }

      return ReefAlignState.HAS_TAGS_WRONG_POSITION;
    }

    if (isAligned) {
      return ReefAlignState.NO_TAGS_IN_POSITION;
    }
    return ReefAlignState.NO_TAGS_WRONG_POSITION;
  }
}
