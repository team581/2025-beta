package frc.robot.auto_align;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.auto_align.tag_align.TagAlign;
import frc.robot.autos.constraints.AutoConstraintCalculator;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.collision_avoidance.ObstructionKind;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveSubsystem;
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

  public static RobotScoringSide getNetScoringSideFromRobotPose(Pose2d robotPose) {
    double robotX = robotPose.getX();
    double theta = robotPose.getRotation().getDegrees() - 90;

    // entire field length is 17.55m
    double halfFieldLength = 17.55 / 2.0;

    // Robot is on blue side
    if (halfFieldLength < robotX) {
      if (Math.abs(theta) < 90) {
        return RobotScoringSide.LEFT;
      }
      return RobotScoringSide.RIGHT;
    } // Robot is on red side
    if (Math.abs(theta) < 90) {
      return RobotScoringSide.RIGHT;
    }
    return RobotScoringSide.LEFT;
  }

  public static boolean shouldIntakeStationFront(Pose2d robotPose) {
    double theta = robotPose.getRotation().getDegrees();
    var coralStationBackwardAngle = SnapUtil.getCoralStationAngle(robotPose);

    return !MathUtil.isNear(coralStationBackwardAngle, theta, 90, -180, 180);
  }

  public static RobotScoringSide getScoringSideFromRobotPose(
      Pose2d robotPose, boolean leftLimelightsOnline, boolean rightLimelightOnline) {
    if (!leftLimelightsOnline) {
      return RobotScoringSide.RIGHT;
    }
    if (!rightLimelightOnline) {
      return RobotScoringSide.LEFT;
    }
    var centerOfReef = FmsSubsystem.isRedAlliance() ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
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

  public static boolean isStationIntakeProcessorSide(Pose2d robotPose) {
    if (robotPose.getY() > 4.025) {
      if (FmsSubsystem.isRedAlliance()) {
        // Coral station red, processor side
        return true;
      }

      // Coral station blue, non processor side
      return false;
    } else {
      if (FmsSubsystem.isRedAlliance()) {
        // Coral station red, non processor side
        return false;
      }
      // Coral station blue, processor side
      return true;
    }
  }

  public static boolean isCloseToReefSide(
      Pose2d robotPose, Pose2d nearestReefSide, double thresholdMeters) {
    return robotPose.getTranslation().getDistance(nearestReefSide.getTranslation())
        < thresholdMeters;
  }

  public static boolean isCloseToReefSide(
      Pose2d robotPose, Pose2d nearestReefSide, ChassisSpeeds robotSpeeds) {
    var linearVelocity = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
    DogLog.log("Swerve/LinearVelocity", linearVelocity);
    return isCloseToReefSide(
        robotPose,
        nearestReefSide,
        LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE_KS
            + LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE_KP * linearVelocity);
  }

  private static final double LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE_KS = 1.5;
  private static final double LINEAR_VELOCITY_TO_REEF_SIDE_DISTANCE_KP = 0.625;

  private final Debouncer isAlignedDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private final VisionSubsystem vision;
  private final LocalizationSubsystem localization;
  private final TagAlign tagAlign;
  private final SwerveSubsystem swerve;

  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();
  private ChassisSpeeds tagAlignSpeeds = new ChassisSpeeds();
  private ChassisSpeeds algaeAlignSpeeds = new ChassisSpeeds();
  private boolean isAligned = false;
  private boolean isAlignedDebounced = false;
  private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
  private ReefPipe bestReefPipe = ReefPipe.PIPE_A;
  private Pose2d usedScoringPose = Pose2d.kZero;

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

  public void setTeleopSpeeds(ChassisSpeeds speeds) {
    teleopSpeeds = speeds;
  }

  private static ChassisSpeeds constrainLinearVelocity(ChassisSpeeds speeds, double maxSpeed) {
    var options =
        new AutoConstraintOptions()
            .withMaxAngularAcceleration(0)
            .withMaxAngularVelocity(0)
            .withMaxLinearAcceleration(0)
            .withMaxLinearVelocity(maxSpeed);
    return AutoConstraintCalculator.constrainLinearVelocity(speeds, options);
  }

  public ChassisSpeeds calculateConstrainedAndWeightedSpeeds(ChassisSpeeds alignSpeeds) {
    var newTeleopSpeeds = teleopSpeeds.times(TELEOP_SPEED_SCALAR);
    var addedSpeeds = newTeleopSpeeds.plus(alignSpeeds);
    var constrainedSpeeds = constrainLinearVelocity(addedSpeeds, MAX_CONSTRAINT);
    DogLog.log("Debug/ConstrainedSpeeds", constrainedSpeeds);
    return constrainedSpeeds;
  }

  @Override
  protected void collectInputs() {
    bestReefPipe = tagAlign.getBestPipe();
    usedScoringPose = tagAlign.getUsedScoringPose(bestReefPipe);
    isAligned = tagAlign.isAligned(bestReefPipe);
    isAlignedDebounced = isAlignedDebouncer.calculate(isAligned);
    tagAlignSpeeds =
        tagAlign.getPoseAlignmentChassisSpeeds(usedScoringPose, /* forwardOnly= */ false);
    algaeAlignSpeeds =
        tagAlign.getAlgaeAlignmentSpeeds(ReefSide.fromPipe(bestReefPipe).getPose(robotScoringSide));
  }

  public ObstructionKind shouldArmScoreForward() {
    // Account for distance we'll be at once we finish forward motion
    var lookaheadPose = localization.getLookaheadPose(0.8);
    var lookaheadPoseDistance =
        lookaheadPose
            .getTranslation()
            .getDistance(
                bestReefPipe.getPose(ReefPipeLevel.BASE, robotScoringSide).getTranslation());
    if (lookaheadPoseDistance > 0.8) {
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

  public void setScoringLevel(ReefPipeLevel level, RobotScoringSide side) {
    robotScoringSide = side;
    tagAlign.setLevel(level, side);
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
    setScoringLevel(level, side);
    return getUsedScoringPose(pipe);
  }

  public void setDriverPoseOffset(Translation2d offset) {
    tagAlign.setDriverPoseOffset(offset);
  }

  public ReefAlignState getReefAlignState() {
    var tagResult = vision.getTagResult();

    if (!vision.isAnyLeftScoringTagLimelightOnline()
        && !vision.isAnyRightScoringTagLimelightOnline()) {
      return ReefAlignState.ALL_CAMERAS_DEAD;
    }

    if (tagResult.isEmpty()) {
      if (isAligned) {
        return ReefAlignState.NO_TAGS_IN_POSITION;
      }
      return ReefAlignState.NO_TAGS_WRONG_POSITION;
    }

    if (isAligned) {
      return ReefAlignState.HAS_TAGS_IN_POSITION;
    }

    return ReefAlignState.HAS_TAGS_WRONG_POSITION;
  }
}
