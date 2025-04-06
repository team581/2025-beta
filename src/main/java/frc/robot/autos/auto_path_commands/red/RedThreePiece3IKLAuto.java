package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.autos.AutoBlocks;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class RedThreePiece3IKLAuto extends BaseAuto {
  public RedThreePiece3IKLAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R2_AND_B2.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        blocks.scorePreloadL4(
            Points.START_R3_AND_B3.redPose, ReefPipe.PIPE_I, RobotScoringSide.LEFT),
        blocks.intakeGround(
            new Pose2d(15.884, 0.994, Rotation2d.fromDegrees(0))
                .transformBy(AutoBlocks.INTAKE_CORAL_GROUND_LINEUP_OFFSET),
            new Pose2d(15.884, 0.994, Rotation2d.fromDegrees(0))),
        blocks.scoreL4(ReefPipe.PIPE_K, RobotScoringSide.LEFT),
        blocks.intakeGround(
            new Pose2d(15.884, 0.994, Rotation2d.fromDegrees(0))
                .transformBy(AutoBlocks.INTAKE_CORAL_GROUND_LINEUP_OFFSET),
            new Pose2d(15.884, 0.994, Rotation2d.fromDegrees(0))),
        blocks.scoreL4(ReefPipe.PIPE_L, RobotScoringSide.LEFT));
  }
}
