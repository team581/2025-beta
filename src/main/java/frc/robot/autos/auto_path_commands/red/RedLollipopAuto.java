package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.RobotScoringSide;
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
        blocks.startingPath(
            getStartingPose(),
            new AutoSegment(
                new AutoPoint(new Pose2d(10.289, 0.758, Rotation2d.fromDegrees(90.0))),
                new AutoPoint(new Pose2d(12.242, 1.278, Rotation2d.fromDegrees(60))),
                new AutoPoint(new Pose2d(13.672, 2.019, Rotation2d.fromDegrees(30))))),
        blocks.scoreL4(ReefPipe.PIPE_L, RobotScoringSide.LEFT),
        blocks.intakeLollipop(
            new Pose2d(13.7, 2.168, Rotation2d.fromDegrees(0))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET),
            new Pose2d(15.1, 2.168, Rotation2d.fromDegrees(0))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
        blocks.scoreL4(ReefPipe.PIPE_A, RobotScoringSide.LEFT),
        blocks.intakeLollipop(
            new Pose2d(13.9, 3.996, Rotation2d.fromDegrees(0))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET),
            new Pose2d(15.1, 3.996, Rotation2d.fromDegrees(0))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
        blocks.scoreL4(ReefPipe.PIPE_B, RobotScoringSide.LEFT),
        blocks.intakeLollipop(
            new Pose2d(14.8, 5.106, Rotation2d.fromDegrees(32))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET),
            new Pose2d(15.795, 5.473, Rotation2d.fromDegrees(32))
                .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
        blocks.scoreL4(ReefPipe.PIPE_C, RobotScoringSide.LEFT));
  }
}
