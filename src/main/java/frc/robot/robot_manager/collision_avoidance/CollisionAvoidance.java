package frc.robot.robot_manager.collision_avoidance;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class CollisionAvoidance {
  private static final List<CollisionBox> ALL_COLLISION_BOXES = List.of(CollisionBox.values());
  private static final double ARM_LENGTH = 23.092038;
  private static final Rotation2d offsetAngle = Rotation2d.fromDegrees(-33.0);

  public static Optional<SuperstructurePosition> plan(
      SuperstructurePosition current, SuperstructurePosition goal) {
    var goalPoint = getGoalPoint(current, goal);

    return goalPoint;
  }

  static CollisionBox getZone(SuperstructurePosition current) {
    var currentPoint = positionToTranslation(current);

    for (var box : ALL_COLLISION_BOXES) {
      if (box.bounds.contains(currentPoint)) {
        // If the current position is contained within these bounds, return the box
        return box;
      }
    }

    // The current position isn't contained within a box, so we find the closest fit
    return ALL_COLLISION_BOXES.stream()
        // Find the box where the distance from actual point to clamped point is the smallest
        .min(
            Comparator.comparingDouble(
                box -> box.bounds.nearest(currentPoint).getDistance(currentPoint)))
        .orElseThrow();
  }

  static Translation2d positionToTranslation(SuperstructurePosition position) {
    var armAngle = Rotation2d.fromDegrees(position.armAngle());
    return new Translation2d(0, position.elevatorHeight())
        .plus(new Translation2d(ARM_LENGTH, armAngle.minus(offsetAngle)));
  }

  private static Optional<SuperstructurePosition> getGoalPoint(
      SuperstructurePosition currentSuperstructurePosition,
      SuperstructurePosition goalSuperstructurePosition) {
    var currentZone = getZone(currentSuperstructurePosition);
    var goalZone = getZone(goalSuperstructurePosition);

    DogLog.log("CollisionAvoidance/CurrentZone", currentZone);
    DogLog.log("CollisionAvoidance/GoalZone", goalZone);
    DogLog.log(
        "CollisionAvoidance/CurrentPosition", positionToTranslation(currentSuperstructurePosition));
    DogLog.log(
        "CollisionAvoidance/CurrentElevatorPosition",
        new Translation2d(0, currentSuperstructurePosition.elevatorHeight()));

    // if(currentZone.id==3||currentZone.id==4||currentZone.id==5&&goalZone.id==3||goalZone.id==4||goalZone.id==5){
    if (currentZone.shortCutPossible(
        goalZone, currentSuperstructurePosition, goalSuperstructurePosition)) {
      DogLog.log("CollisionAvoidance/ShortCutActive", true);
      return Optional.empty();
    } else {
      DogLog.log("CollisionAvoidance/ShortCutActive", false);
    }

    if (currentZone.id < goalZone.id) {
      DogLog.log("DEBUG/NextZone", currentZone.id + 1);
      return Optional.of(CollisionBox.getById(currentZone.id + 1).safeZone);
    }

    if (currentZone.id > goalZone.id) {
      DogLog.log("DEBUG/NextZone", currentZone.id - 1);
      return Optional.of(CollisionBox.getById(currentZone.id - 1).safeZone);
    }

    return Optional.empty();
  }
}
