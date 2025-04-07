package frc.robot.robot_manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;

public class RobotCommands {
  private final RobotManager robot;
  private final Subsystem[] rmRequirements;
  private final Subsystem[] gmRequirements;
  private final Subsystem[] bothRequirements;

  public RobotCommands(RobotManager robot) {
    this.robot = robot;
    var requirementsList = List.of(robot.elevator, robot.arm, robot.claw, robot.climber);
    rmRequirements = requirementsList.toArray(Subsystem[]::new);
    requirementsList = List.of(robot.groundManager.deploy, robot.groundManager.intake);
    gmRequirements = requirementsList.toArray(Subsystem[]::new);
    requirementsList =
        List.of(
            robot.elevator,
            robot.arm,
            robot.claw,
            robot.climber,
            robot.groundManager.deploy,
            robot.groundManager.intake);
    bothRequirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command algaeIntakeGroundCommand() {
    return Commands.runOnce(robot::intakeFloorAlgaeRequest, rmRequirements)
        .withName("AlgaeIntakeGroundCommand");
  }

  public Command floorAssistIntakeCommand() {
    return Commands.runOnce(robot::lollipopIntakeApproachRequest, rmRequirements)
        .withName("FloorIntakeCommand");
  }

  public Command highLineupCommand() {
    return Commands.runOnce(robot::highLineupRequest, bothRequirements)
        .withName("HighLineupCommand");
  }

  public Command l3LineupCommand() {
    return Commands.runOnce(robot::l3CoralApproachRequest, bothRequirements)
        .withName("L3LineupCommand");
  }

  public Command l2LineupCommand() {
    return Commands.runOnce(robot::l2CoralApproachRequest, bothRequirements)
        .withName("L2LineupCommand");
  }

  public Command lowLineupCommand() {
    return Commands.runOnce(robot::lowLineupRequest, bothRequirements).withName("LowLineupCommand");
  }

  public Command algaeReefIntakeCommand() {
    return Commands.runOnce(robot::algaeReefIntakeRequest, rmRequirements)
        .withName("AlgaeReefIntakeCommand");
  }

  public Command confirmScoreCommand() {
    return Commands.runOnce(robot::confirmScoreRequest, bothRequirements)
        .withName("ConfirmScoreCommand");
  }

  public Command stowCommand() {
    return Commands.runOnce(robot::stowRequest, bothRequirements)
        .andThen(
            Commands.waitUntil(
                () ->
                    robot.elevator.atGoal()
                        && robot.arm.atGoal()
                        && robot.groundManager.deploy.atGoal()))
        .withName("StowCommand");
  }

  public Command scoringAlignOffCommand() {
    return Commands.runOnce(robot::scoringAlignOffRequest, bothRequirements)
        .withName("ScoringAlignOffCommand");
  }

  public Command climbUpCommand() {
    return Commands.runOnce(robot::nextClimbStateRequest, rmRequirements)
        .withName("ClimbUpCommand");
  }

  public Command climbStopCommand() {
    return Commands.runOnce(robot::stopClimbStateRequest, rmRequirements)
        .withName("ClimbStopCommand");
  }

  public Command unjamCommand() {
    return Commands.runOnce(robot::unjamRequest, bothRequirements).withName("UnjamCommand");
  }

  public Command rehomeElevatorCommand() {
    return Commands.runOnce(robot::rehomeElevatorRequest, rmRequirements)
        .withName("RehomeElevatorCommand");
  }

  public Command floorIntakeCommand() {
    return Commands.runOnce(robot.groundManager::intakeRequest, gmRequirements)
        .withName("FloorIntakeCommand");
  }

  public Command rehomeDeployCommand() {
    return Commands.runOnce(robot.groundManager::rehomeDeployRequest, gmRequirements)
        .withName("RehomeDeployCommand");
  }

  public Command spinToWinCommand() {
    return Commands.runOnce(robot::spinToWinRequest, rmRequirements).withName("SpinToWinCommand");
  }

  public Command testNextLollipopCommand() {
    return Commands.runOnce(robot::testingNextLollipopRequest, rmRequirements)
        .withName("TestNextLollipopCommand");
  }

  public Command waitAlgaeFlingCommand() {
    return Commands.runOnce(robot::algaeFlingRequest, rmRequirements)
        .withName("WaitAlgaeFlingCommand");
  }

  public Command endgameStowCommand() {
    return Commands.runOnce(robot::endgameStowRequest, rmRequirements)
        .withName("EndgameStowCommand");
  }
}
