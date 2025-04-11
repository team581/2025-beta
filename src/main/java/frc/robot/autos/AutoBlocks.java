package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.config.RobotConfig;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.util.PoseErrorTolerance;

public class AutoBlocks {
  /**
   * The tolerance used to determine when to end the little "backup & stow" motion we do after
   * scoring L4.
   */
  private static final PoseErrorTolerance AFTER_SCORE_POSITION_TOLERANCE =
      new PoseErrorTolerance(0.3, 10);

  public static final PoseErrorTolerance APPROACH_REEF_TOLERANCE = new PoseErrorTolerance(0.6, 10);

  public static final Transform2d INTAKE_CORAL_GROUND_LINEUP_OFFSET =
      new Transform2d(-1.3, 0, Rotation2d.kZero);

  private static final Transform2d INTAKE_CORAL_GROUND_APPROACH_OFFSET =
      new Transform2d(-0.6, 0, Rotation2d.kZero);

  public static final Transform2d LOLLIPOP_OFFSET =
      new Transform2d(
          0.0,
          -Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(90));

  public static final AutoConstraintOptions BASE_CONSTRAINTS =
      new AutoConstraintOptions(3.5, 57, 2, 30);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      BASE_CONSTRAINTS.withMaxLinearAcceleration(2.0);
  private static final AutoConstraintOptions LOLLIPOP_CONSTRAINTS =
      BASE_CONSTRAINTS.withMaxLinearAcceleration(1.0).withMaxLinearVelocity(1.5);

  private final Trailblazer trailblazer;
  private final RobotManager robotManager;
  private final AutoCommands autoCommands;

  public AutoBlocks(Trailblazer trailblazer, RobotManager robotManager, AutoCommands autoCommands) {
    this.trailblazer = trailblazer;
    this.robotManager = robotManager;
    this.autoCommands = autoCommands;
  }

  public Command scorePreloadL4(Pose2d startingPose, ReefPipe pipe, RobotScoringSide scoringSide) {
    return Commands.sequence(
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
                    autoCommands
                        .preloadCoralCommand()
                        .andThen(autoCommands.l4ApproachCommand(scoringSide)),
                    BASE_CONSTRAINTS),
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
                    BASE_CONSTRAINTS),
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
                    autoCommands.l4ApproachCommand(scoringSide),
                    SCORING_CONSTRAINTS),
                // Actually align to score
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT),
                    SCORING_CONSTRAINTS))),
        //          .withDeadline(
        //
        // autoCommands.waitForAlignedForScore().andThen(autoCommands.l4LeftReleaseCommand())),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                AFTER_SCORE_POSITION_TOLERANCE,
                // Start at the scoring position
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
                    Commands.waitSeconds(0.15).andThen(robotManager::stowRequest)),
                // Scoot back to the lineup position to finish the score
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT)))));
  }

  public Command scoreL4(ReefPipe pipe, RobotScoringSide scoringSide, Command onFinish) {
    return Commands.sequence(
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(
                        () -> robotManager.autoAlign.getUsedScoringPose(pipe),
                        Commands.runOnce(() -> robotManager.autoAlign.setAutoReefPipeOverride(pipe))
                            .andThen(robotManager.waitForState(RobotState.CLAW_CORAL))
                            .andThen(autoCommands.l4ApproachCommand(scoringSide)),
                        BASE_CONSTRAINTS)),
                false)
            .withDeadline(autoCommands.waitForReleaseCommand().withTimeout(3)),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                AFTER_SCORE_POSITION_TOLERANCE,
                new AutoPoint(
                    () -> pipe.getPose(ReefPipeLevel.BACK_AWAY, scoringSide),
                    Commands.waitSeconds(0.10).andThen(onFinish)))));
  }

  public Command scoreL4(ReefPipe pipe, RobotScoringSide scoringSide) {
    return scoreL4(pipe, scoringSide, Commands.runOnce(robotManager::stowRequest));
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
        .withDeadline(autoCommands.waitForIntakeDone());
  }

  public Command scoreL3(ReefPipe pipe, RobotScoringSide scoringSide, Command onFinish) {
    return Commands.sequence(
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(
                        () -> robotManager.autoAlign.getUsedScoringPose(pipe),
                        autoCommands.l3ApproachCommand(pipe, scoringSide),
                        BASE_CONSTRAINTS)),
                false)
            .withDeadline(autoCommands.waitForReleaseCommand().withTimeout(5)),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                AFTER_SCORE_POSITION_TOLERANCE,
                new AutoPoint(
                    () -> pipe.getPose(ReefPipeLevel.BACK_AWAY, scoringSide),
                    Commands.waitSeconds(0.10).andThen(onFinish)))));
  }

  public Command scoreL3(ReefPipe pipe, RobotScoringSide scoringSide) {
    return scoreL3(pipe, scoringSide, Commands.runOnce(robotManager::stowRequest));
  }

  public Command intakeLollipop(Pose2d approachPoint, Pose2d defaultIntakingPoint) {
    return Commands.sequence(
        autoCommands.lollipopApproachCommand(),
        trailblazer
            .followSegment(new AutoSegment(BASE_CONSTRAINTS, new AutoPoint(approachPoint)), false)
            .withDeadline(autoCommands.waitForElevatorAndArmNearGoal()),
        autoCommands.intakeLollipopCommand(),
        trailblazer
            .followSegment(
                new AutoSegment(
                    BASE_CONSTRAINTS,
                    new AutoPoint(
                        () ->
                            new Pose2d(
                                robotManager
                                    .coralMap
                                    .getLollipopIntakePose()
                                    .orElse(defaultIntakingPoint)
                                    .getTranslation(),
                                defaultIntakingPoint.getRotation()),
                        LOLLIPOP_CONSTRAINTS)),
                false)
            .withDeadline(autoCommands.waitForLollipopIntakeDone()));
  }

  public Command intakeGroundForL4(Pose2d defaultIntakingPose) {
    return trailblazer
        .followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                new AutoPoint(
                    () -> robotManager.coralMap.getBestCoralPose().orElse(defaultIntakingPose),
                    Commands.runOnce(
                        () -> {
                          robotManager.groundManager.intakeRequest();
                          robotManager.l4WaitingHandoffRequest();
                        }))),
            false)
        .withDeadline(autoCommands.waitForIntakeDone());
  }
}
