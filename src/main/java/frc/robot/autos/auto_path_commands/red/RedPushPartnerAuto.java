package frc.robot.autos.auto_path_commands.red;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.robot_manager.RobotManager;

public class RedPushPartnerAuto extends BaseAuto {
  public RedPushPartnerAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R3_AND_B3.redPose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        trailblazer.followSegment(
            new AutoSegment(
                new AutoPoint(Points.START_R3_AND_B3.redPose),
                new AutoPoint(new Pose2d(9.462, 3.054, Rotation2d.fromDegrees(0))))),
        blocks.scorePreloadL4(Points.START_R3_AND_B3, ReefPipe.PIPE_I),
        blocks.intakeCoralGroundPoints(Points.GROUND_INTAKE_LEFT_STATION),
        blocks.scoreL4(ReefPipe.PIPE_J),
        blocks.intakeCoralGroundPoints(Points.GROUND_INTAKE_LEFT_STATION),
        blocks.scoreL4(ReefPipe.PIPE_K));
  }
}
