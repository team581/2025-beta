package frc.robot.auto_align.field_calibration;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.arm.ArmState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.elevator.ElevatorState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.lights.LightsState;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;

/**
 * Logs useful diagnostics to validate scoring setpoints during field calibration. Enabled with the
 * "FIELD_CALIBRATION" feature flag.
 *
 * <p>FIELD_CALIBRATION has the following changes:
 *
 * <ul>
 *   <li>Lights: green & yellow lights show whether scoring align & superstructure is good
 *   <li>Elevator: in coast mode while disabled
 */
public class FieldCalibrationUtil {
  private static final double ELEVATOR_TOLERANCE = 0.5;
  // Copied from ArmSubsystem
  private static final double ARM_TOLERANCE = 2.0;
  // Copied from TagAlign
  private static final double TRANSLATION_TOLERANCE = 0.05;
  private static final double HEADING_TOLERANCE = 1;

  private static final List<ReefPipeLevel> LEVELS =
      List.of(ReefPipeLevel.L2, ReefPipeLevel.L3, ReefPipeLevel.L4);

  private static Summary createSummary(
      ElevatorState wantedElevator,
      double actualElevator,
      ArmState wantedArm,
      double actualArm,
      Pose2d wantedPose,
      Pose2d actualPose) {
    var elevatorError = wantedElevator.getHeight() - actualElevator;
    var armError = wantedArm.getAngle() - actualArm;
    var alignError = wantedPose.getTranslation().getDistance(actualPose.getTranslation());

    var elevatorState = MechanismState.OK;
    var armState = MechanismState.OK;
    var alignOk = MathUtil.isNear(0, alignError, TRANSLATION_TOLERANCE);
    var headingOk =
        MathUtil.isNear(
            wantedPose.getRotation().getDegrees(),
            actualPose.getRotation().getDegrees(),
            HEADING_TOLERANCE);

    if (elevatorError > ELEVATOR_TOLERANCE) {
      elevatorState = MechanismState.TOO_LOW;
    } else if (elevatorError < -ELEVATOR_TOLERANCE) {
      elevatorState = MechanismState.TOO_HIGH;
    }

    if (armError > ARM_TOLERANCE) {
      armState = MechanismState.TOO_LOW;
    } else if (armError < -ARM_TOLERANCE) {
      armState = MechanismState.TOO_HIGH;
    }

    return new Summary(elevatorState, armState, alignOk, headingOk);
  }

  private Summary createSummary(ReefPipeLevel level, ScoringPosition position) {
    var actualElevator = elevator.getHeight();
    var actualArm = arm.getAngle();

    var wantedElevator = branchToElevator(level);
    var wantedArm = branchToArm(level, position.side());

    var wantedPose = position.getPose(level);
    var actualPose = localization.getPose();

    return createSummary(
        wantedElevator, actualElevator, wantedArm, actualArm, wantedPose, actualPose);
  }

  private static ElevatorState branchToElevator(ReefPipeLevel level) {
    return switch (level) {
      case L2 -> ElevatorState.CORAL_SCORE_LINEUP_L2;
      case L3 -> ElevatorState.CORAL_SCORE_LINEUP_L3;
      case L4 -> ElevatorState.CORAL_SCORE_LINEUP_L4;
      default -> ElevatorState.UNTUNED;
    };
  }

  private static ArmState branchToArm(ReefPipeLevel level, RobotScoringSide side) {
    return switch (level) {
      case L2 ->
          side == RobotScoringSide.LEFT
              ? ArmState.CORAL_SCORE_LEFT_LINEUP_L2
              : ArmState.CORAL_SCORE_RIGHT_LINEUP_L2;
      case L3 ->
          side == RobotScoringSide.LEFT
              ? ArmState.CORAL_SCORE_LEFT_LINEUP_L3
              : ArmState.CORAL_SCORE_RIGHT_LINEUP_L3;
      default ->
          side == RobotScoringSide.LEFT
              ? ArmState.CORAL_SCORE_LEFT_LINEUP_L4
              : ArmState.CORAL_SCORE_RIGHT_LINEUP_L4;
    };
  }

  private final ElevatorSubsystem elevator;
  private final ArmSubsystem arm;
  private final LightsSubsystem lights;
  private final LocalizationSubsystem localization;

  public FieldCalibrationUtil(
      ElevatorSubsystem elevator,
      ArmSubsystem arm,
      LightsSubsystem lights,
      LocalizationSubsystem localization) {
    this.elevator = elevator;
    this.arm = arm;
    this.lights = lights;
    this.localization = localization;
  }

  public void log() {
    logAllScoringPositions(false, RobotScoringSide.LEFT);
    logAllScoringPositions(false, RobotScoringSide.RIGHT);
    logAllScoringPositions(true, RobotScoringSide.LEFT);
    logAllScoringPositions(true, RobotScoringSide.RIGHT);

    var anyOk = false;

    for (var level : LEVELS) {
      var prefix = "FieldCalibration/Best/" + level.toString();
      var bestScoringPosition = getBestScoringPosition(level);
      DogLog.log(prefix + "/Alliance", bestScoringPosition.isRedAlliance() ? "Red" : "Blue");
      DogLog.log(prefix + "/Pipe", bestScoringPosition.pipe());
      DogLog.log(prefix + "/Orientation", bestScoringPosition.side());
      DogLog.log(prefix + "/Pose", bestScoringPosition.getPose(level));

      var summary = createSummary(level, bestScoringPosition);

      DogLog.log(prefix + "/Summary", summary.format());

      anyOk = anyOk || summary.isOk();
    }

    // Use lights to indicate we are in any valid scoring configuration
    lights.setDisabledState(anyOk ? LightsState.SCORE_ALIGN_TAGS : LightsState.SCORE_ALIGN_NO_TAGS);
  }

  private static void logAllScoringPositions(boolean isRedAlliance, RobotScoringSide side) {
    var allianceLabel = isRedAlliance ? "Red" : "Blue";
    var sideLabel = side == RobotScoringSide.LEFT ? "Left" : "Right";

    for (var pipe : ReefPipe.values()) {
      for (var level : LEVELS) {
        DogLog.log(
            "FieldCalibration/" + allianceLabel + "/" + sideLabel + "/" + level.toString(),
            pipe.getPose(level, isRedAlliance, side));
      }
    }
  }

  private ScoringPosition getBestScoringPosition(ReefPipeLevel level) {
    var bestRedLeft = getBestScoringPipe(true, RobotScoringSide.LEFT);
    var bestRedRight = getBestScoringPipe(true, RobotScoringSide.RIGHT);
    var bestBlueLeft = getBestScoringPipe(false, RobotScoringSide.LEFT);
    var bestBlueRight = getBestScoringPipe(false, RobotScoringSide.RIGHT);

    var robotPose = localization.getPose();

    var bestPipe =
        Stream.of(bestRedLeft, bestRedRight, bestBlueLeft, bestBlueRight)
            .min(
                Comparator.comparingDouble(
                        (ScoringPosition scoringPosition) ->
                            scoringPosition
                                .getPose(level)
                                .getTranslation()
                                .getDistance(robotPose.getTranslation()))
                    .thenComparingDouble(
                        (ScoringPosition scoringPosition) ->
                            Math.abs(
                                robotPose
                                    .getRotation()
                                    .minus(scoringPosition.getPose(level).getRotation())
                                    .getRadians())))
            .orElseThrow();

    return bestPipe;
  }

  private ScoringPosition getBestScoringPipe(boolean isRedAlliance, RobotScoringSide side) {
    var robotTranslation = localization.getPose().getTranslation();

    var bestPipe =
        Arrays.stream(ReefPipe.values())
            .min(
                Comparator.comparingDouble(
                    pipe ->
                        pipe.getPose(ReefPipeLevel.L4, isRedAlliance, side)
                            .getTranslation()
                            .getDistance(robotTranslation)))
            .orElseThrow();

    return new ScoringPosition(bestPipe, side, isRedAlliance);
  }
}
