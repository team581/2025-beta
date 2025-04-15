package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.constraints.AutoConstraintCalculator;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.autos.followers.PathFollower;
import frc.robot.autos.followers.PidPathFollower;
import frc.robot.autos.trackers.PathTracker;
import frc.robot.autos.trackers.pure_pursuit.PurePursuitPathTracker;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.TimestampedChassisSpeeds;

public class Trailblazer {
  /**
   * Given a point and the constraints for its parent segment, resolve the constraint options to use
   * while following that point.
   */
  private static AutoConstraintOptions resolveConstraints(
      AutoPoint point, AutoConstraintOptions segmentConstraints) {
    var constraints = point.constraints.orElse(segmentConstraints);
    return constraints;
  }

  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final PathTracker pathTracker = new PurePursuitPathTracker();
  private final PathFollower pathFollower =
      new PidPathFollower(
          new PIDController(5, 0, 0), new PIDController(5, 0, 0), new PIDController(7, 0, 0));
  private int previousAutoPointIndex = -1;
  private TimestampedChassisSpeeds previousSpeeds = new TimestampedChassisSpeeds(0);

  public Trailblazer(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.swerve = swerve;
    this.localization = localization;
  }

  public Command followSegment(AutoSegment segment) {
    return followSegment(segment, true);
  }

  public Command followSegment(AutoSegment segment, boolean shouldEnd) {
    TrailblazerPathLogger.logSegment(segment);
    var command =
        Commands.runOnce(
                () -> {
                  pathTracker.resetAndSetPoints(segment.points);
                  previousAutoPointIndex = -1;
                  DogLog.log(
                      "Autos/Trailblazer/CurrentSegment/InitialPoints",
                      segment.points.stream()
                          .map(point -> point.poseSupplier.get())
                          .toArray(Pose2d[]::new));
                })
            .alongWith(
                Commands.run(
                    () -> {
                      pathTracker.updateRobotState(
                          localization.getPose(), swerve.getFieldRelativeSpeeds());
                      var currentAutoPointIndex = pathTracker.getCurrentPointIndex();
                      var currentAutoPoint = segment.points.get(currentAutoPointIndex);
                      double distanceToSegmentEnd =
                          segment.getRemainingDistance(
                              localization.getPose(), currentAutoPointIndex);

                      var constrainedVelocityGoal =
                          getSwerveSetpoint(
                              currentAutoPoint, segment.defaultConstraints, distanceToSegmentEnd);
                      swerve.setFieldRelativeAutoSpeeds(constrainedVelocityGoal);

                      DogLog.log(
                          "Autos/Trailblazer/Tracker/CurrentPointIndex", currentAutoPointIndex);
                      if (previousAutoPointIndex != currentAutoPointIndex) {
                        // Currently tracked point has changed, trigger side effects

                        // Each of the points in (previous, current]
                        var pointsToRunSideEffectsFor =
                            segment.points.subList(
                                previousAutoPointIndex + 1, currentAutoPointIndex + 1);
                        for (var passedPoint : pointsToRunSideEffectsFor) {
                          DogLog.log(
                              "Autos/Trailblazer/Tracker/CommandTriggered",
                              passedPoint.command.getName());
                          passedPoint.command.schedule();
                        }
                        previousAutoPointIndex = currentAutoPointIndex;
                      }
                    },
                    swerve))
            .withName("FollowSegmentIndefinitely");

    if (shouldEnd) {
      return command
          .until(
              () -> segment.isFinished(localization.getPose(), pathTracker.getCurrentPointIndex()))
          .andThen(
              Commands.runOnce(
                  () -> {
                    swerve.setFieldRelativeAutoSpeeds(new ChassisSpeeds());
                  }))
          .withName("FollowSegmentUntilFinished");
    }

    return command;
  }

  private ChassisSpeeds getSwerveSetpoint(
      AutoPoint point, AutoConstraintOptions segmentConstraints, double distanceToSegmentEnd) {
    if (previousSpeeds.timestampSeconds == 0) {
      previousSpeeds = new TimestampedChassisSpeeds(Timer.getFPGATimestamp() - 0.02);
    }

    var robotPose = localization.getPose();
    var originalTargetPose = pathTracker.getTargetPose();
    var originalVelocityGoal =
        new TimestampedChassisSpeeds(
            pathFollower.calculateSpeeds(
                robotPose, swerve.getFieldRelativeSpeeds(), originalTargetPose));
    var usedConstraints = resolveConstraints(point, segmentConstraints);

    /*
    var newLinearVelocity =
        AutoConstraintCalculator.getDynamicVelocityConstraint(
            robotPose,
            endPose,
            swerve.getFieldRelativeSpeeds(),
            originalConstraints.maxLinearVelocity(),
            originalConstraints.maxLinearAcceleration());
    */

    pathFollower.setConstraints(usedConstraints);

    DogLog.log(
        "Autos/Trailblazer/Constraints/VelocityCalculation/CalculatedLinearVelocity",
        usedConstraints.maxLinearVelocity());
    DogLog.log(
        "Autos/Trailblazer/Constraints/Acceleration/CalulatedLinearAcceleration",
        usedConstraints.maxLinearAcceleration());
    DogLog.log("Autos/Trailblazer/Tracker/RawOutput", originalTargetPose);

    DogLog.log("Autos/Trailblazer/Follower/RawOutput", originalVelocityGoal);
    var constrainedVelocityGoal = originalVelocityGoal;
    if (!pathFollower.handlesConstraints()) {
      AutoConstraintCalculator.constrainVelocityGoal(
          originalVelocityGoal, previousSpeeds, usedConstraints, distanceToSegmentEnd);
    }

    DogLog.log("Autos/Trailblazer/Follower/UsedOutput", constrainedVelocityGoal);

    previousSpeeds = constrainedVelocityGoal;

    return constrainedVelocityGoal;
  }
}
