package frc.robot.autos.constraints;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.TimestampedChassisSpeeds;

public class AutoConstraintCalculator {
  private static AutoConstraintOptions lastUsedConstraints = new AutoConstraintOptions();

  public static TimestampedChassisSpeeds constrainVelocityGoal(
      TimestampedChassisSpeeds inputSpeeds,
      TimestampedChassisSpeeds previousSpeeds,
      AutoConstraintOptions options,
      double distanceToSegmentEnd) {
    ChassisSpeeds constrainedSpeeds = constrainVelocityGoal(inputSpeeds, previousSpeeds, options);

    double newLinearVelocity =
        getAccelerationBasedVelocityConstraint(
            constrainedSpeeds,
            distanceToSegmentEnd,
            options.maxLinearAcceleration(),
            options.maxLinearVelocity());
    constrainedSpeeds =
        constrainLinearVelocity(
            constrainedSpeeds, options.withMaxLinearVelocity(newLinearVelocity));
    DogLog.log(
        "Debug/finalConstrainedSpeeds",
        Math.hypot(constrainedSpeeds.vxMetersPerSecond, constrainedSpeeds.vyMetersPerSecond));
    return new TimestampedChassisSpeeds(constrainedSpeeds, inputSpeeds.timestampSeconds);
  }

  public static ChassisSpeeds constrainVelocityGoal(
      TimestampedChassisSpeeds inputSpeeds,
      TimestampedChassisSpeeds previousSpeeds,
      AutoConstraintOptions options) {
    lastUsedConstraints = options;
    var constrainedSpeeds = inputSpeeds;

    if (options.maxLinearVelocity() != 0) {
      constrainedSpeeds =
          new TimestampedChassisSpeeds(
              constrainLinearVelocity(constrainedSpeeds, options),
              constrainedSpeeds.timestampSeconds);
    }

    if (options.maxAngularVelocity() != 0) {
      constrainedSpeeds =
          new TimestampedChassisSpeeds(
              constrainRotationalVelocity(constrainedSpeeds, options),
              constrainedSpeeds.timestampSeconds);
    }

    // if (options.maxLinearAcceleration() != 0) {
    if (false) {
      constrainedSpeeds =
          new TimestampedChassisSpeeds(
              constrainLinearAcceleration(constrainedSpeeds, previousSpeeds, options),
              constrainedSpeeds.timestampSeconds);
    }

    if (options.maxAngularAcceleration() != 0) {
      constrainedSpeeds =
          new TimestampedChassisSpeeds(
              constrainRotationalAcceleration(constrainedSpeeds, previousSpeeds, options),
              constrainedSpeeds.timestampSeconds);
    }

    return constrainedSpeeds;
  }

  public static AutoConstraintOptions getLastUsedConstraints() {
    return lastUsedConstraints;
  }

  public static ChassisSpeeds constrainLinearVelocity(
      ChassisSpeeds inputSpeeds, AutoConstraintOptions options) {
    double currentLinearVelocity =
        Math.hypot(inputSpeeds.vxMetersPerSecond, inputSpeeds.vyMetersPerSecond);
    // double preserveTheta = Math.atan(inputSpeeds.vyMetersPerSecond /
    // inputSpeeds.vxMetersPerSecond);
    if (currentLinearVelocity > options.maxLinearVelocity()) {
      double clampingFactor = options.maxLinearVelocity() / currentLinearVelocity;

      return new ChassisSpeeds(
          inputSpeeds.vxMetersPerSecond * clampingFactor,
          inputSpeeds.vyMetersPerSecond * clampingFactor,
          inputSpeeds.omegaRadiansPerSecond);
    }
    return inputSpeeds;
  }

  private static ChassisSpeeds constrainRotationalVelocity(
      ChassisSpeeds inputSpeeds, AutoConstraintOptions options) {
    double currentAngularVelocity = inputSpeeds.omegaRadiansPerSecond;
    if (currentAngularVelocity > options.maxAngularVelocity()) {
      double clampingFactor = options.maxAngularVelocity() / currentAngularVelocity;
      return new ChassisSpeeds(
          inputSpeeds.vxMetersPerSecond,
          inputSpeeds.vyMetersPerSecond,
          inputSpeeds.omegaRadiansPerSecond * clampingFactor);
    }

    return inputSpeeds;
  }

  private static ChassisSpeeds constrainLinearAcceleration(
      TimestampedChassisSpeeds inputSpeeds,
      TimestampedChassisSpeeds previousSpeeds,
      AutoConstraintOptions options) {

    double inputTotalSpeed =
        Math.sqrt(
            Math.pow(inputSpeeds.vxMetersPerSecond, 2)
                + Math.pow(inputSpeeds.vyMetersPerSecond, 2));
    double previousTotalSpeed =
        Math.sqrt(
            Math.pow(previousSpeeds.vxMetersPerSecond, 2)
                + Math.pow(previousSpeeds.vyMetersPerSecond, 2));
    double unconstrainedLinearAcceleration =
        (inputTotalSpeed - previousTotalSpeed) / inputSpeeds.timestampDifference(previousSpeeds);

    if (unconstrainedLinearAcceleration < 0) {
      return inputSpeeds;
    }

    double deltaVx = inputSpeeds.vxMetersPerSecond - previousSpeeds.vxMetersPerSecond;
    double deltaVy = inputSpeeds.vyMetersPerSecond - previousSpeeds.vyMetersPerSecond;

    double constrainedLinearAcceleration =
        Math.min(unconstrainedLinearAcceleration, options.maxLinearAcceleration());

    if (unconstrainedLinearAcceleration > options.maxLinearAcceleration()) {
      double constrainedVx =
          previousSpeeds.vxMetersPerSecond
              + (deltaVx / unconstrainedLinearAcceleration) * constrainedLinearAcceleration;
      double constrainedVy =
          previousSpeeds.vyMetersPerSecond
              + (deltaVy / unconstrainedLinearAcceleration) * constrainedLinearAcceleration;

      return new ChassisSpeeds(constrainedVx, constrainedVy, inputSpeeds.omegaRadiansPerSecond);
    }
    return inputSpeeds;
  }

  public static double getAccelerationBasedVelocityConstraint(
      ChassisSpeeds currentSpeeds,
      double distanceToSegmentEnd,
      double accelerationLimit,
      double velocityConstraint) {
    double currentVelocity =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    double decelerationDistance =
        (1.0 * (currentVelocity * currentVelocity)) / (2.0 * accelerationLimit);
    double perfectVelocity =
        Math.sqrt(0.0 - (-1.0 * 2.0 * (accelerationLimit * distanceToSegmentEnd)));
    // TODO: Clean these logs up
    DogLog.log("Debug/DistanceToSegmentEnd", distanceToSegmentEnd);
    DogLog.log("Debug/currentVelocity", currentVelocity);
    DogLog.log("Debug/perfectDecelerationVelocity", perfectVelocity);
    DogLog.log("Debug/decelerationDistance", decelerationDistance);
    if (distanceToSegmentEnd > decelerationDistance) {
      return currentVelocity;
    }
    return Math.max(perfectVelocity, 0.05); // Allow you to go 2 inches per second
  }

  public static double getDynamicVelocityConstraint(
      Pose2d currentPose,
      Pose2d endWaypoint,
      ChassisSpeeds currentSpeeds,
      double oldVelocityConstraint,
      double accelerationLimit) {
    var distanceToEnd = currentPose.getTranslation().getDistance(endWaypoint.getTranslation());
    DogLog.log("Debug/DistanceToEnd", distanceToEnd);
    var currentVelocity =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    DogLog.log("Debug/CurrentVelocity", distanceToEnd);

    var timeToTraverse = distanceToEnd / currentVelocity;
    var acceleration = (accelerationLimit - currentVelocity) / timeToTraverse;
    if (Math.abs(acceleration) < accelerationLimit) {
      return oldVelocityConstraint;
    }
    var velocityConstraint = acceleration * timeToTraverse;
    var clampedConstraint = MathUtil.clamp(Math.abs(velocityConstraint), 0.5, 5.0);
    return clampedConstraint;
  }

  private static ChassisSpeeds constrainRotationalAcceleration(
      TimestampedChassisSpeeds inputSpeeds,
      TimestampedChassisSpeeds previousSpeeds,
      AutoConstraintOptions options) {

    double currentAngularSpeed = inputSpeeds.omegaRadiansPerSecond;
    double previousAngularSpeed = previousSpeeds.omegaRadiansPerSecond;

    double currentAngularAcceleration =
        currentAngularSpeed
            - previousAngularSpeed / inputSpeeds.timestampDifference(previousSpeeds);
    if (currentAngularAcceleration > options.maxAngularAcceleration()) {
      double constrainedAngularAcceleration =
          previousAngularSpeed
              + options.maxAngularAcceleration() * inputSpeeds.timestampDifference(previousSpeeds);
      return new ChassisSpeeds(
          inputSpeeds.vxMetersPerSecond,
          inputSpeeds.vyMetersPerSecond,
          constrainedAngularAcceleration);
    }
    return inputSpeeds;
  }

  private AutoConstraintCalculator() {}
}
