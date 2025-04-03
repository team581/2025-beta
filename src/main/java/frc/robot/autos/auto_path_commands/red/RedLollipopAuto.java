package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.AutoBlocks;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class RedLollipopAuto extends BaseAuto {
  public RedLollipopAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R3_AND_B3.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        blocks.scorePreloadL4(Points.START_R3_AND_B3.redPose, ReefPipe.PIPE_I),
        blocks.intakeLollipop(
            new Pose2d(14.2, 2.168, Rotation2d.fromDegrees(90))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET),
            new Pose2d(15.2, 2.168, Rotation2d.fromDegrees(90))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
        blocks.scoreL4(ReefPipe.PIPE_A),
        blocks.intakeLollipop(
            new Pose2d(14.5, 3.996, Rotation2d.fromDegrees(90))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET),
            new Pose2d(15.2, 3.996, Rotation2d.fromDegrees(90))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
        blocks.scoreL4(ReefPipe.PIPE_B),
        blocks.intakeLollipop(
            new Pose2d(15.085, 5.106, Rotation2d.fromDegrees(140.5))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET),
            new Pose2d(15.513, 5.473, Rotation2d.fromDegrees(140.5))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
        blocks.scoreL4(ReefPipe.PIPE_C));
  }
}
