package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.PoseErrorTolerance;

public class AutoBlocks {
  /**
   * The tolerance used to determine when to end the little "backup & stow" motion we do after
   * scoring L4.
   */
  private static final PoseErrorTolerance AFTER_SCORE_POSITION_TOLERANCE =
      new PoseErrorTolerance(0.3, 10);

  /**
   * The offset used to calculate the position to go to before PIDing to the lineup pose. Ensures we
   * don't strafe into the scoring position, just a straight-on movement.
   */
  private static final Transform2d PIPE_LINEUP_OFFSET = new Transform2d(-0.6, 0, Rotation2d.kZero);

  private static final Transform2d PIPE_APPROACH_OFFSET =
      new Transform2d(-1.3, 0, Rotation2d.kZero);

  private static final Transform2d INTAKE_CORAL_GROUND_LINEUP_OFFSET =
      new Transform2d(-1.3, 0, Rotation2d.kZero);

  private static final Transform2d INTAKE_CORAL_GROUND_APPROACH_OFFSET =
      new Transform2d(-0.6, 0, Rotation2d.kZero);

  public static final AutoConstraintOptions BASE_CONSTRAINTS =
      new AutoConstraintOptions(4.7, 57, 4, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      BASE_CONSTRAINTS.withMaxLinearAcceleration(2.0);
  private static final AutoConstraintOptions LOLLIPOP_CONSTRAINTS =
      BASE_CONSTRAINTS.withMaxLinearAcceleration(2.0).withMaxLinearVelocity(1.0);

  private final Trailblazer trailblazer;
  private final RobotManager robotManager;
  private final AutoCommands autoCommands;

  public AutoBlocks(Trailblazer trailblazer, RobotManager robotManager, AutoCommands autoCommands) {
    this.trailblazer = trailblazer;
    this.robotManager = robotManager;
    this.autoCommands = autoCommands;
  }

  public Command scorePreloadL4(Pose2d startingPose, ReefPipe pipe) {
    return Commands.sequence(
        autoCommands.resetPoseIfNeeded(startingPose),
        trailblazer
            .followSegment(
                new AutoSegment(
                    BASE_CONSTRAINTS,
                    new AutoPoint(
                        robotManager
                            .autoAlign
                            .getUsedScoringPose(pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT)
                            .transformBy(PIPE_APPROACH_OFFSET),
                        autoCommands
                            .preloadCoralCommand()
                            .andThen(autoCommands.l4WarmupCommand(pipe)),
                        BASE_CONSTRAINTS),
                    new AutoPoint(
                        () ->
                            robotManager
                                .autoAlign
                                .getUsedScoringPose(pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT)
                                .transformBy(PIPE_LINEUP_OFFSET),
                        BASE_CONSTRAINTS),
                    new AutoPoint(
                        () ->
                            robotManager
                                .autoAlign
                                .getUsedScoringPose(pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT)
                                .transformBy(PIPE_LINEUP_OFFSET),
                        autoCommands.l4LineupCommand(pipe),
                        SCORING_CONSTRAINTS),
                    // Actually align to score
                    new AutoPoint(
                        () ->
                            robotManager.autoAlign.getUsedScoringPose(
                                pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT),
                        SCORING_CONSTRAINTS)),
                false)
            .withDeadline(
                autoCommands.waitForAlignedForScore().andThen(autoCommands.l4LeftReleaseCommand())),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                AFTER_SCORE_POSITION_TOLERANCE,
                // Start at the scoring position
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT),
                    Commands.waitSeconds(0.15).andThen(robotManager::stowRequest)),
                // Scoot back to the lineup position to finish the score
                new AutoPoint(
                    () ->
                        robotManager
                            .autoAlign
                            .getUsedScoringPose(pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT)
                            .transformBy(PIPE_LINEUP_OFFSET)))));
  }

  public Command scoreL4(ReefPipe pipe) {
    return Commands.sequence(
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(
                        () ->
                            robotManager
                                .autoAlign
                                .getUsedScoringPose(pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT)
                                .transformBy(PIPE_APPROACH_OFFSET),
                        autoCommands.l4WarmupCommand(pipe),
                        BASE_CONSTRAINTS),
                    new AutoPoint(
                        () ->
                            robotManager
                                .autoAlign
                                .getUsedScoringPose(pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT)
                                .transformBy(PIPE_LINEUP_OFFSET),
                        autoCommands.l4LineupCommand(pipe),
                        SCORING_CONSTRAINTS),
                    new AutoPoint(
                        () ->
                            robotManager.autoAlign.getUsedScoringPose(
                                pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT))),
                false)
            .withDeadline(
                autoCommands.waitForAlignedForScore().andThen(autoCommands.l4LeftReleaseCommand())),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                AFTER_SCORE_POSITION_TOLERANCE,
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT),
                    Commands.waitSeconds(0.15).andThen(robotManager::stowRequest)),
                new AutoPoint(
                    () ->
                        robotManager
                            .autoAlign
                            .getUsedScoringPose(pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT)
                            .transformBy(PIPE_LINEUP_OFFSET)))));
  }

  public Command intakeCoralGroundPoints(Points intakingPoint) {
    return trailblazer
        .followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                new AutoPoint(
                    intakingPoint.redPose.transformBy(INTAKE_CORAL_GROUND_APPROACH_OFFSET),
                    Commands.runOnce(robotManager.groundManager::intakeRequest)),
                new AutoPoint(intakingPoint.redPose)),
            false)
        .withDeadline(autoCommands.waitForGroundIntakeDone());
  }

  public Command intakeLollipop(Pose2d approachPoint, Pose2d defaultIntakingPoint) {
    return trailblazer
        .followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                new AutoPoint(
                    approachPoint, Commands.runOnce(() -> autoCommands.lollipopApproachCommand())),
                new AutoPoint(
                    () ->
                        robotManager.coralMap.getLollipopIntakePose().orElse(defaultIntakingPoint),
                    Commands.runOnce(() -> autoCommands.intakeLollipopCommand()),
                    LOLLIPOP_CONSTRAINTS)),
            false)
        .withTimeout(5);
  }

  public Command intakeGround(Pose2d approachPoint, Pose2d defaultIntakingPose) {
    return trailblazer
        .followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                new AutoPoint(approachPoint),
                new AutoPoint(
                    () -> robotManager.coralMap.getBestCoral().orElse(defaultIntakingPose),
                    Commands.runOnce(robotManager::lollipopIntakeApproachRequest))),
            false)
        .withDeadline(autoCommands.waitForGroundIntakeDone());
  }
}
