package frc.robot.autos;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.auto_align.ReefPipe;
import frc.robot.elevator.CoralStation;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import java.util.List;
import java.util.stream.Stream;

public class AutoCommands {
  private final RobotCommands robotCommands;
  private final RobotManager robotManager;
  private final Subsystem[] requirements;

  public AutoCommands(RobotCommands robotCommands, RobotManager robotManager) {
    this.robotCommands = robotCommands;
    this.robotManager = robotManager;
    var requirementsList = List.of(robotManager.elevator, robotManager.arm, robotManager.intake);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command l4ScoreAndReleaseCommand() {
    return Commands.runOnce(robotManager::l4coralPlaceAndReleaseRequest, requirements)
        .andThen(
            robotManager
                .waitForStates(
                    RobotState.CORAL_DISPLACED_L4_4_RELEASE, RobotState.CORAL_CENTERED_L4_4_RELEASE)
                .withTimeout(1))
        .withName("L4ScoreAndReleaseCommand");
  }

  public Command waitThenStow() {
    return Commands.sequence(
        Commands.waitSeconds(0.25), Commands.runOnce(robotManager::forceIdleNoGp));
  }

  @Deprecated
  public Command intakeStationWithTimeoutCommand() {
    return Commands.runOnce(robotManager::intakeStationRequest, requirements)
        .andThen(
            robotManager.waitForStates(
                RobotState.SMART_STOW_1, RobotState.SMART_STOW_2, RobotState.IDLE_CORAL))
        .withTimeout(2)
        .withName("IntakeStationWithTimeoutCommand");
  }

  public Command l4WarmupCommand(ReefPipe pipe) {
    return Commands.waitUntil(
            () ->
                robotManager.getState() != RobotState.SMART_STOW_1
                    && robotManager.getState() != RobotState.SMART_STOW_2)
        .andThen(
            Commands.runOnce(
                () -> {
                  robotManager.autoAlign.setAutoReefPipeOverride(pipe);
                  robotManager.l4CoralApproachRequest();
                }));
  }

  public Command l4LineupCommand(ReefPipe pipe) {
    return Commands.runOnce(
        () -> {
          robotManager.autoAlign.setAutoReefPipeOverride(pipe);
          robotManager.l4CoralLineupRequest();
        });
  }

  public Command intakeStationWarmupCommand() {
    return Commands.runOnce(robotManager::intakeStationRequest);
  }

  public Command preloadCoralAfterRollHomed() {
    return robotManager
        .waitForRollHomedCommand()
        .andThen(Commands.runOnce(robotManager::preloadCoralRequest));
  }

  public boolean isSmartStowing() {
    return robotManager.getState() == RobotState.SMART_STOW_1
        || robotManager.getState() == RobotState.SMART_STOW_2;
  }

  /**
   * @deprecated Use {@link #hasCoral()} instead
   */
  @Deprecated
  public boolean isIdleCoral() {
    return robotManager.getState() == RobotState.IDLE_CORAL;
  }

  public boolean hasCoral() {
    return robotManager.getState() == RobotState.IDLE_CORAL;
  }

  public Command waitForBackIntakeDone() {
    if (RobotBase.isSimulation()) {
      return Commands.waitUntil(
              () -> {
                var robotPose = robotManager.localization.getPose();

                return Stream.of(CoralStation.values())
                    .anyMatch(
                        station ->
                            station
                                    .backLoadPose
                                    .getTranslation()
                                    .getDistance(robotPose.getTranslation())
                                < 0.05);
              })
          // Simulate delay from human player dropping the coral
          .andThen(Commands.waitSeconds(0.75));
    }

    return Commands.waitUntil(this::isSmartStowing);
  }

  public Command waitForFrontIntakeDone() {
    if (RobotBase.isSimulation()) {
      // Wait until aligned at the coral station
      return Commands.waitUntil(
              () -> {
                var robotPose = robotManager.localization.getPose();

                return Stream.of(CoralStation.values())
                    .anyMatch(
                        station ->
                            station
                                    .frontLoadPose
                                    .getTranslation()
                                    .getDistance(robotPose.getTranslation())
                                < 0.05);
              })
          // Simulate delay from human player dropping the coral
          .andThen(Commands.waitSeconds(0.75));
    }

    return robotManager
        .waitForState(RobotState.INTAKE_CORAL_STATION_FRONT)
        .andThen(robotManager.waitForState(RobotState.IDLE_CORAL));
  }

  public Command stowRequest() {
    return Commands.runOnce(robotManager::stowRequest);
  }

  public boolean alignedForScore() {
    return robotManager.autoAlign.isTagAlignedDebounced()
        && robotManager.imu.isFlatDebounced()
        && robotManager.elevator.atGoal()
        && robotManager.arm.atGoal();
  }

  public Command waitForAlignedForScore() {
    // return robotManager
    //     .waitForStates(
    //         RobotState.CORAL_CENTERED_L4_2_LINEUP, RobotState.CORAL_DISPLACED_L4_2_LINEUP)
    //     .andThen(Commands.waitUntil(this::alignedForScore));
    return Commands.waitUntil(this::alignedForScore).withTimeout(5.0);
  }

  public Command waitForGroundIntakeDone() {
    return robotManager
        .waitForStates(
            RobotState.INTAKE_CORAL_FLOOR_HORIZONTAL,
            RobotState.INTAKE_CORAL_FLOOR_UPRIGHT,
            RobotState.INTAKE_ASSIST_CORAL_FLOOR_HORIZONTAL)
        .andThen(robotManager.waitForState(RobotState.IDLE_CORAL));
  }
}
