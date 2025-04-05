package frc.robot.auto_align.tag_align;

import com.google.common.collect.ImmutableList;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefState;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import java.util.Optional;

public class TagAlign {
  private static final ImmutableList<ReefPipe> ALL_REEF_PIPES =
      ImmutableList.copyOf(ReefPipe.values());

  private static final PIDController TAG_SIDEWAYS_PID = new PIDController(5.0, 0.0, 0.0);
  private static final PIDController TAG_FORWARD_PID = new PIDController(4.5, 0.0, 0.0);
  private static final DoubleSubscriber TRANSLATION_GOOD_THRESHOLD =
      DogLog.tunable("AutoAlign/IsAlignedTranslation", 0.03);
  private static final DoubleSubscriber ROTATION_GOOD_THRESHOLD =
      DogLog.tunable("AutoAlign/IsAlignedRotation", 5.0);

  /** Ratio from joystick percentage to scoring pose offset in meters. */
  private static final double FINE_ADJUST_CONTROLLER_SCALAR = 0.3;

  private static final double PIPE_SWITCH_TIMEOUT = 1.0;

  private final AlignmentCostUtil alignmentCostUtil;
  private final LocalizationSubsystem localization;
  private static final double LAST_PIPE_SWITCH_TIMESTAMP = 0.0;

  private ReefPipeLevel pipeLevel = ReefPipeLevel.RAISING;
  private ReefPipeLevel preferedScoringLevel = ReefPipeLevel.L4;
  private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
  private Optional<ReefPipe> reefPipeOverride = Optional.empty();
  private double rawControllerXValue = 0.0;
  private double rawControllerYValue = 0.0;

  private boolean pipeSwitchActive = false;

  public ReefState reefState = new ReefState();

  public TagAlign(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.localization = localization;
    alignmentCostUtil = new AlignmentCostUtil(localization, swerve, reefState, robotScoringSide);
  }

  public void setLevel(ReefPipeLevel level, ReefPipeLevel preferredLevel, RobotScoringSide side) {
    this.robotScoringSide = side;
    alignmentCostUtil.setSide(robotScoringSide);
    this.pipeLevel = level;
    this.preferedScoringLevel = preferredLevel;
  }

  public void setPipeOveride(ReefPipe pipe) {
    this.reefPipeOverride = Optional.of(pipe);
  }

  public void setControllerValues(double controllerXValue, double controllerYValue) {
    rawControllerXValue = controllerXValue;
    rawControllerYValue = controllerYValue;
    checkControllerForSwitch();
  }

  private void checkControllerForSwitch() {
    if (pipeSwitchActive
        && (Timer.getFPGATimestamp() > LAST_PIPE_SWITCH_TIMESTAMP + PIPE_SWITCH_TIMEOUT)
        && rawControllerXValue == 0.0) {
      pipeSwitchActive = false;
    }
    if (pipeSwitchActive) {
      return;
    }
    if ((rawControllerXValue < -0.98 || rawControllerXValue > 0.98)) {
      var storedPipe = getBestPipe();
      pipeSwitchActive = true;
      ReefPipe leftPipe =
          switch (storedPipe) {
            case PIPE_A -> ReefPipe.PIPE_L;
            case PIPE_B -> ReefPipe.PIPE_A;
            case PIPE_C -> ReefPipe.PIPE_B;
            case PIPE_D -> ReefPipe.PIPE_C;
            case PIPE_E -> ReefPipe.PIPE_F;
            case PIPE_F -> ReefPipe.PIPE_G;
            case PIPE_G -> ReefPipe.PIPE_H;
            case PIPE_H -> ReefPipe.PIPE_I;
            case PIPE_I -> ReefPipe.PIPE_J;
            case PIPE_J -> ReefPipe.PIPE_K;
            case PIPE_K -> ReefPipe.PIPE_J;
            case PIPE_L -> ReefPipe.PIPE_K;
          };
      ReefPipe rightPipe =
          switch (storedPipe) {
            case PIPE_A -> ReefPipe.PIPE_B;
            case PIPE_B -> ReefPipe.PIPE_C;
            case PIPE_C -> ReefPipe.PIPE_D;
            case PIPE_D -> ReefPipe.PIPE_E;
            case PIPE_E -> ReefPipe.PIPE_D;
            case PIPE_F -> ReefPipe.PIPE_E;
            case PIPE_G -> ReefPipe.PIPE_F;
            case PIPE_H -> ReefPipe.PIPE_G;
            case PIPE_I -> ReefPipe.PIPE_H;
            case PIPE_J -> ReefPipe.PIPE_I;
            case PIPE_K -> ReefPipe.PIPE_L;
            case PIPE_L -> ReefPipe.PIPE_A;
          };

      if (rawControllerXValue < -0.98) {
        setPipeOveride(leftPipe);
      } else if (rawControllerXValue > 0.98) {
        setPipeOveride(rightPipe);
      }
    }
  }

  public boolean isAligned(ReefPipe pipe) {
    if (pipeLevel.equals(ReefPipeLevel.RAISING) || pipeLevel.equals(ReefPipeLevel.BACK_AWAY)) {
      return false;
    }
    var robotPose = localization.getPose();
    var scoringPoseFieldRelative = getUsedScoringPose(pipe);
    var translationGood =
        (robotPose.getTranslation().getDistance(scoringPoseFieldRelative.getTranslation())
            <= TRANSLATION_GOOD_THRESHOLD.get());
    var rotationGood =
        MathUtil.isNear(
            scoringPoseFieldRelative.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            ROTATION_GOOD_THRESHOLD.get(),
            -180.0,
            180.0);
    return translationGood && rotationGood;
  }

  public void markScored(ReefPipe pipe) {
    reefState.markScored(pipe, preferedScoringLevel);
  }

  public void clearReefState() {
    reefState.clear();
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe) {
    var theoreticalScoringPose = pipe.getPose(pipeLevel, robotScoringSide);

    if (DriverStation.isTeleop()) {
      var offsetPose =
          new Pose2d(theoreticalScoringPose.getTranslation(), theoreticalScoringPose.getRotation());
      return offsetPose;
    }

    return theoreticalScoringPose;
  }

  /** Returns the best reef pipe for scoring, based on the robot's current state. */
  public ReefPipe getBestPipe() {
    if ((DriverStation.isAutonomous() || pipeSwitchActive) && reefPipeOverride.isPresent()) {
      return reefPipeOverride.orElseThrow();
    }
    var level = pipeLevel;
    if (pipeLevel.equals(ReefPipeLevel.RAISING)) {
      level = preferedScoringLevel;
    }
    return ALL_REEF_PIPES.stream()
        .min(alignmentCostUtil.getReefPipeComparator(level))
        .orElseThrow();
  }

  public ChassisSpeeds getPoseAlignmentChassisSpeeds(Pose2d usedScoringPose) {
    var robotPose = localization.getPose();

    var scoringTranslationRobotRelative =
        usedScoringPose
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - usedScoringPose.getRotation().getDegrees()));

    var goalTranslationWithP =
        new Translation2d(
            TAG_SIDEWAYS_PID.calculate(scoringTranslationRobotRelative.getX()),
            TAG_FORWARD_PID.calculate(scoringTranslationRobotRelative.getY()));
    var goalTranslation = goalTranslationWithP.rotateBy(usedScoringPose.getRotation());

    var goalSpeeds = new ChassisSpeeds(-goalTranslation.getX(), -goalTranslation.getY(), 0.0);
    DogLog.log("AutoAlign/GoalSpeeds", goalSpeeds);
    return goalSpeeds;
  }
}
