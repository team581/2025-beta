package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.auto_align.ReefPipe;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.robot_manager.ground_manager.GroundState;
import java.util.List;

public class AutoCommands {
  private static final double LOLLIPOP_LOWER_ARM_THRESHOLD = 1.0;
  private final RobotCommands robotCommands;
  private final RobotManager robotManager;
  private final Subsystem[] requirements;

  public AutoCommands(RobotCommands robotCommands, RobotManager robotManager) {
    this.robotCommands = robotCommands;
    this.robotManager = robotManager;
    var requirementsList = List.of(robotManager.elevator, robotManager.arm, robotManager.claw);
    requirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command stowRequest() {
    return Commands.runOnce(robotManager::stowRequest);
  }

  public Command preloadCoralCommand() {
    return Commands.runOnce(robotManager::preloadCoralRequest);
  }

  public boolean alignedForScore() {
    return robotManager.autoAlign.isTagAlignedDebounced()
        && robotManager.imu.isFlatDebounced()
        && robotManager.elevator.atGoal()
        && robotManager.arm.atGoal();
  }

  public Command resetPoseIfNeeded(Pose2d pose) {
    return Commands.runOnce(
        () -> {
          if (!robotManager.vision.hasSeenTag()) {
            robotManager.localization.resetPose(pose);
          }
        });
  }

  public Command intakeCoralHorizontalCommand() {
    return Commands.runOnce(robotManager.groundManager::intakeRequest);
  }

  public Command lollipopApproachCommand() {
    return Commands.runOnce(robotManager::lollipopIntakeApproachRequest);
  }

  public Command intakeLollipopCommand() {
    return Commands.runOnce(robotManager::lollipopIntakeGrabRequest);
  }

  public Command waitForIntakeDone() {
    return robotManager
        .groundManager
        .waitForState(GroundState.INTAKING)
        .andThen(robotManager.groundManager.waitForState(GroundState.IDLE_CORAL))
        .withTimeout(4);
  }

  public Command waitForGroundIntakeDone() {
    return robotManager
        .waitForState(RobotState.CORAL_INTAKE_LOLLIPOP_APPROACH)
        .andThen(robotManager.groundManager.waitForState(GroundState.INTAKING));
  }

  public Command l4WarmupCommand(ReefPipe pipe) {
    return Commands.runOnce(
        () -> {
          robotManager.autoAlign.setAutoReefPipeOverride(pipe);
          robotManager.l4CoralApproachRequest();
        });
  }

  public Command l4LineupCommand(ReefPipe pipe) {
    return Commands.runOnce(
        () -> {
          robotManager.autoAlign.setAutoReefPipeOverride(pipe);
          robotManager.highLineupRequest();
        });
  }

  public Command l4LeftReleaseCommand() {
    return Commands.runOnce(robotManager::l4CoralLeftReleaseRequest);
  }

  public Command waitForAlignedForScore() {
    if (RobotBase.isSimulation()) {
      return Commands.waitSeconds(1.0);
    }
    return Commands.waitUntil(this::alignedForScore).withTimeout(5);
  }
}
