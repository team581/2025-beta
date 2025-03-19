package frc.robot.auto_align.field_calibration;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.arm.ArmState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.config.RobotConfig;
import frc.robot.elevator.ElevatorState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.lights.LightsState;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * Logs useful diagnostics to validate scoring setpoints during field calibration. Enabled with the
 * "FIELD_CALIBRATION" feature flag.
 *
 * <p>FIELD_CALIBRATION has the following changes:
 *
 * <ul>
 *   <li>Lights: green & yellow lights show whether scoring align & superstructure is good
 *   <li>Elevator: in coast mode while disabled
 *   <li>Arm: assumes it is at bottom hardstop when code starts
 */
public class FieldCalibrationUtil {
  private static final double ELEVATOR_TOLERANCE = RobotConfig.get().elevator().tolerance();
  // Copied from ArmSubsystem
  private static final double ARM_TOLERANCE = 2.0;
  // Copied from TagAlign
  private static final double TRANSLATION_TOLERANCE = 0.05;

  private static final List<ReefPipeLevel> LEVELS =
      List.of(ReefPipeLevel.L2, ReefPipeLevel.L3, ReefPipeLevel.L4);

  private static Summary createSummary(
      ElevatorState wantedElevator,
      double actualElevator,
      ArmState wantedArm,
      double actualArm,
      Translation2d wantedTranslation,
      Translation2d actualTranslation) {
    var elevatorError = wantedElevator.height - actualElevator;
    var armError = wantedArm.angle - actualArm;
    var alignError = wantedTranslation.getDistance(actualTranslation);

    var elevatorState = MechanismState.OK;
    var armState = MechanismState.OK;
    var alignOk = MathUtil.isNear(0, alignError, TRANSLATION_TOLERANCE);

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

    return new Summary(elevatorState, armState, alignOk);
  }

  private static ElevatorState branchToElevator(ReefPipeLevel level, boolean centered) {
    return switch (level) {
      case L2 ->
          centered
              ? ElevatorState.CORAL_CENTERED_L2_LINEUP
              : ElevatorState.CORAL_DISPLACED_L2_LINEUP;
      case L3 ->
          centered
              ? ElevatorState.CORAL_CENTERED_L3_LINEUP
              : ElevatorState.CORAL_DISPLACED_L3_LINEUP;
      case L4 ->
          centered
              ? ElevatorState.CORAL_CENTERED_L4_LINEUP
              : ElevatorState.CORAL_DISPLACED_L4_LINEUP;
      default -> ElevatorState.UNTUNED;
    };
  }

  private static ArmState branchToArm(ReefPipeLevel level, boolean centered) {
    return switch (level) {
      case L2 ->
          centered
              ? ArmState.CORAL_SCORE_CENTERED_LINEUP_L2
              : ArmState.CORAL_SCORE_DISPLACED_LINEUP_L2;
      case L3 ->
          centered
              ? ArmState.CORAL_SCORE_CENTERED_LINEUP_L3
              : ArmState.CORAL_SCORE_DISPLACED_LINEUP_L3;
      case L4 ->
          centered
              ? ArmState.CORAL_SCORE_CENTERED_LINEUP_L4
              : ArmState.CORAL_SCORE_DISPLACED_LINEUP_L4;
      default -> ArmState.UNTUNED;
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
    var robotTranslation = localization.getPose().getTranslation();
    var bestRedPipe =
        Arrays.stream(ReefPipe.values())
            .min(
                Comparator.comparingDouble(
                    pipe ->
                        pipe.getPose(ReefPipeLevel.L4, true)
                            .getTranslation()
                            .getDistance(robotTranslation)))
            .orElseThrow();
    var bestBluePipe =
        Arrays.stream(ReefPipe.values())
            .min(
                Comparator.comparingDouble(
                    pipe ->
                        pipe.getPose(ReefPipeLevel.L4, false)
                            .getTranslation()
                            .getDistance(robotTranslation)))
            .orElseThrow();

    DogLog.log("FieldCalibration/Red/Best/Name", bestRedPipe);
    DogLog.log("FieldCalibration/Blue/Best/Name", bestBluePipe);

    var anyOk = false;

    for (var level : LEVELS) {
      DogLog.log("FieldCalibration/Red/Best/" + level.toString(), bestRedPipe.getPose(level, true));
      DogLog.log(
          "FieldCalibration/Blue/Best/" + level.toString(), bestBluePipe.getPose(level, false));

      var redDisplacedSummary = createSummary(bestRedPipe, true, level, false);
      var redCenteredSummary = createSummary(bestRedPipe, true, level, true);
      var blueDisplacedSummary = createSummary(bestBluePipe, false, level, false);
      var blueCenteredSummary = createSummary(bestBluePipe, false, level, true);

      anyOk =
          anyOk
              || redDisplacedSummary.isOk()
              || redCenteredSummary.isOk()
              || blueDisplacedSummary.isOk()
              || blueCenteredSummary.isOk();

      DogLog.log(
          "FieldCalibration/Red/Best/" + level.toString() + "/Displaced",
          redDisplacedSummary.format());
      DogLog.log(
          "FieldCalibration/Red/Best/" + level.toString() + "/Centered",
          redCenteredSummary.format());
      DogLog.log(
          "FieldCalibration/Blue/Best/" + level.toString() + "/Displaced",
          blueDisplacedSummary.format());
      DogLog.log(
          "FieldCalibration/Blue/Best/" + level.toString() + "/Centered",
          blueCenteredSummary.format());

      // Loop through all L2-4 pipes and log the scoring poses for each
      for (var pipe : ReefPipe.values()) {
        var branchSlug = pipe.toString() + "/" + level.toString() + "/Pose";
        DogLog.log("FieldCalibration/Red/" + branchSlug, pipe.getPose(level, true));
        DogLog.log("FieldCalibration/Blue/" + branchSlug, pipe.getPose(level, false));
      }
    }

    // Use lights to indicate we are in any valid scoring configuration
    lights.setDisabledState(
        anyOk ? LightsState.SCORE_ALIGN_READY : LightsState.SCORE_ALIGN_NOT_READY);
  }

  private Summary createSummary(
      ReefPipe pipe, boolean isRedAlliance, ReefPipeLevel level, boolean centered) {
    var actualElevator = elevator.getHeight();
    var actualArm = arm.getAngle();

    var wantedElevator = branchToElevator(level, centered);
    var wantedArm = branchToArm(level, centered);

    var wantedTranslation = pipe.getPose(level, isRedAlliance).getTranslation();
    var actualTranslation = localization.getPose().getTranslation();

    return createSummary(
        wantedElevator,
        actualElevator,
        wantedArm,
        actualArm,
        wantedTranslation,
        actualTranslation);
  }
}
