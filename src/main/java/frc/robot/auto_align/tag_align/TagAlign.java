package frc.robot.auto_align.tag_align;

import com.google.common.collect.ImmutableList;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefState;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import java.util.List;
import java.util.Optional;

public class TagAlign {
  private static final ImmutableList<ReefPipe> ALL_REEF_PIPES =
      ImmutableList.copyOf(List.of(ReefPipe.values()));

  private static final PIDController TAG_SIDEWAYS_PID = new PIDController(6.0, 0.0, 0.0);
  private static final PIDController TAG_FORWARD_PID = new PIDController(3.0, 0.0, 0.0);
  private static final double TAG_ALIGNMENT_FINISHED_DISTANCE_THRESHOLD = 0.05;

  private final AlignmentCostUtil alignmentCostUtil;
  private final LocalizationSubsystem localization;
  private ReefPipeLevel level = ReefPipeLevel.BASE;
  private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
  private Translation2d driverPoseOffset = Translation2d.kZero;
  private Optional<ReefPipe> reefPipeOverride = Optional.empty();

  public ReefState reefState = new ReefState();

  public TagAlign(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.localization = localization;
    alignmentCostUtil = new AlignmentCostUtil(localization, swerve, reefState, robotScoringSide);
  }

  public void setLevel(ReefPipeLevel level, RobotScoringSide side) {
    this.level = level;
    this.robotScoringSide = side;
  }

  public void setPipeOveride(ReefPipe pipe) {
    this.reefPipeOverride = Optional.of(pipe);
  }

  public void setDriverPoseOffset(Translation2d offset) {
    driverPoseOffset = offset;
  }

  public boolean isAligned(ReefPipe pipe) {
    var robotPose = localization.getPose();
    var scoringPoseFieldRelative = getUsedScoringPose(pipe);
    return robotPose.getTranslation().getDistance(scoringPoseFieldRelative.getTranslation())
        <= TAG_ALIGNMENT_FINISHED_DISTANCE_THRESHOLD;
  }

  public void markScored(ReefPipe pipe) {
    reefState.markScored(pipe, level);
  }

  public void clearReefState() {
    reefState.clear();
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe) {
    var theoreticalScoringPose = pipe.getPose(level, robotScoringSide);

    if (DriverStation.isTeleop()) {
      var offsetPose =
          new Pose2d(
              theoreticalScoringPose.getTranslation().plus(driverPoseOffset),
              theoreticalScoringPose.getRotation());
      return offsetPose;
    }

    return theoreticalScoringPose;
  }

  /** Returns the best reef pipe for scoring, based on the robot's current state. */
  public ReefPipe getBestPipe() {
    if (DriverStation.isAutonomous() && reefPipeOverride.isPresent()) {
      return reefPipeOverride.orElseThrow();
    }

    return ALL_REEF_PIPES.stream()
        .min(alignmentCostUtil.getReefPipeComparator(level))
        .orElseThrow();
  }

  public ChassisSpeeds getAlgaeAlignmentSpeeds(Pose2d usedScoringPose) {
    var robotPose = localization.getPose();
    var scoringTranslationRobotRelative =
        usedScoringPose
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));

    var goalTranslationWithP =
        new Translation2d(
            TAG_FORWARD_PID.calculate(0.0),
            TAG_SIDEWAYS_PID.calculate(scoringTranslationRobotRelative.getY()));
    var goalTranslation = goalTranslationWithP.rotateBy(robotPose.getRotation());
    var goalSpeeds = new ChassisSpeeds(goalTranslation.getX(), goalTranslation.getY(), 0.0);
    DogLog.log("AutoAlign/AlgaeAlign/GoalSpeeds", goalSpeeds);
    return goalSpeeds;
  }

  public ChassisSpeeds getPoseAlignmentChassisSpeeds(Pose2d usedScoringPose, boolean forwardOnly) {
    var robotPose = localization.getPose();

    var scoringTranslationRobotRelative =
        usedScoringPose
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));

    var goalTranslationWithP =
        new Translation2d(
            TAG_FORWARD_PID.calculate(scoringTranslationRobotRelative.getX()),
            TAG_SIDEWAYS_PID.calculate(scoringTranslationRobotRelative.getY()));
    var goalTranslation = goalTranslationWithP.rotateBy(robotPose.getRotation());

    var goalSpeeds = new ChassisSpeeds(-goalTranslation.getX(), -goalTranslation.getY(), 0.0);
    DogLog.log("AutoAlign/GoalSpeeds", goalSpeeds);
    return goalSpeeds;
  }
}
