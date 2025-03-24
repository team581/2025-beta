package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.RobotConfig;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;

public abstract class BaseAuto {
  protected final RobotManager robotManager;
  protected final Trailblazer trailblazer;
  protected final RobotCommands actions;
  protected final AutoCommands autoCommands;
  protected final AutoBlocks blocks;
  private final String autoName;
  private final Command autoCommand;

  protected BaseAuto(RobotManager robotManager, Trailblazer trailblazer) {
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
    actions = new RobotCommands(robotManager);
    autoCommands = new AutoCommands(actions, robotManager);
    blocks = new AutoBlocks(trailblazer, robotManager, autoCommands);

    var className = this.getClass().getSimpleName();
    autoName = className.substring(className.lastIndexOf('.') + 1);
    autoCommand = createFullAutoCommand();
  }

  protected abstract Pose2d getStartingPose();

  protected abstract Command createAutoCommand();

  /** Returns the name of this auto. */
  public String name() {
    return autoName;
  }

  public Command getAutoCommand() {
    return autoCommand;
  }

  private Command createFullAutoCommand() {
    TrailblazerPathLogger.markAuto(this);
    // We continuously reset the pose anyway, but doing it here should be fine
    // It's basically free as long as we aren't updating the IMU
    return createAutoCommand()
        .finallyDo(
            interrupted -> {
              // Check if we are enabled, since auto commands are cancelled during disable
              if (interrupted && DriverStation.isAutonomousEnabled()) {
                DogLog.logFault("Auto command interrupted outside teleop");

                if (RobotConfig.IS_DEVELOPMENT) {
                  throw new IllegalStateException(
                      "The auto command was interrupted while still in auto mode, is there a command requirements conflict?");
                }
              }
            })
        .withName(autoName + "Command");
  }
}
