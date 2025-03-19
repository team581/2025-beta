package frc.robot.robot_manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] requirements;

  public RobotCommands(RobotManager robot) {
    this.robot = robot;
    var requirementsList = List.of(robot.elevator, robot.arm, robot.intake, robot.climber);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command floorIntakeCommand() {
    return Commands.runOnce(robot::intakeFloorRequest, requirements).withName("FloorIntakeCommand");
  }

  public Command floorAssistIntakeCommand() {
    return Commands.runOnce(robot::intakeAssistFloorRequest, requirements)
        .withName("FloorIntakeCommand");
  }

  public Command confirmScoreCommand() {
    return Commands.runOnce(robot::confirmScoreRequest, requirements)
        .withName("ConfirmScoreCommand");
  }

  public Command stowCommand() {
    return Commands.runOnce(robot::stowRequest, requirements)
        .andThen(Commands.waitUntil(() -> robot.elevator.atGoal() && robot.arm.atGoal()))
        .withName("StowCommand");
  }

  public Command intakeStationCommand() {
    return Commands.runOnce(robot::intakeStationRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_CORAL))
        .withName("IntakeStationCommand");
  }

  public Command lowLineupCommand() {
    return Commands.waitUntil(robot::notSmartStowing)
        .andThen(Commands.runOnce(robot::lowLineupRequest, requirements))
        .withName("LowLineupCommand");
  }

  public Command l2LineupCommand() {
    return Commands.waitUntil(robot::notSmartStowing)
        .andThen(Commands.runOnce(robot::l2LineupRequest, requirements))
        .withName("L2LineupCommand");
  }

  public Command l3LineupCommand() {
    return Commands.waitUntil(robot::notSmartStowing)
        .andThen(Commands.runOnce(robot::l3LineupRequest, requirements))
        .withName("L3LineupCommand");
  }

  public Command highLineupCommand() {
    return Commands.waitUntil(robot::notSmartStowing)
        .andThen(Commands.runOnce(robot::highApproachRequest, requirements))
        .withName("HighLineupCommand");
  }

  public Command setAlgaeModeCommand(boolean algaeActive) {
    return Commands.runOnce(
            () -> {
              robot.setAlgaeMode(algaeActive);
            })
        .withName("SetAlgaeModeCommand");
  }

  public Command climbUpCommand() {
    return Commands.runOnce(robot::nextClimbStateRequest, requirements).withName("ClimbUpCommand");
  }

  public Command climbDownCommand() {
    return Commands.runOnce(robot::previousClimbStateRequest, requirements)
        .withName("ClimbDownCommand");
  }

  public Command unjamCommand() {
    return Commands.runOnce(robot::unjamRequest, requirements).withName("UnjamCommand");
  }

  public Command unjamStationCommand() {
    return Commands.runOnce(robot::unjamStationRequest, requirements)
        .withName("UnjamStationCommand");
  }

  public Command rehomeElevatorCommand() {
    return Commands.runOnce(robot::rehomeElevatorRequest, requirements)
        .withName("RehomeElevatorCommand");
  }

  public Command rehomeArmCommand() {
    return Commands.runOnce(robot::rehomeArmRequest, requirements).withName("RehomeArmCommand");
  }

  public Command rehomeRollCommand() {
    return Commands.runOnce(robot::rehomeRollRequest, requirements)
        .andThen(robot.waitForState(RobotState.IDLE_NO_GP))
        .withName("RehomeRollCommand");
  }
}
