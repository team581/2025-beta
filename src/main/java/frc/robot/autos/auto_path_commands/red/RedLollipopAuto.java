package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.AutoBlocks;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
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
    return Points.START_R1_AND_B1.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        autoCommands.resetPoseIfNeeded(getStartingPose()),
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(new Pose2d(10.289, 0.758, Rotation2d.fromDegrees(90))),
                new AutoPoint(new Pose2d(12.242, 1.278, Rotation2d.fromDegrees(90))),
                new AutoPoint(new Pose2d(13.672, 2.019, Rotation2d.fromDegrees(90))))),
        blocks.scorePreloadL4(
            new Pose2d(13.672, 2.019, Rotation2d.fromDegrees(90)), ReefPipe.PIPE_L),
        blocks.intakeLollipop(
            new Pose2d(13.8, 2.168, Rotation2d.fromDegrees(0))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET),
            new Pose2d(15.2, 2.168, Rotation2d.fromDegrees(0))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
        blocks.scoreL4(ReefPipe.PIPE_A),
        blocks.intakeLollipop(
            new Pose2d(14.1, 3.996, Rotation2d.fromDegrees(0))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET),
            new Pose2d(15.2, 3.996, Rotation2d.fromDegrees(0))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
        blocks.scoreL4(ReefPipe.PIPE_B),
        blocks.intakeLollipop(
            new Pose2d(14.8, 5.106, Rotation2d.fromDegrees(32))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET),
            new Pose2d(15.795, 5.473, Rotation2d.fromDegrees(32))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
        blocks.scoreL4(ReefPipe.PIPE_C));
  }
}
