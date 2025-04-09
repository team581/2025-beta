package frc.robot.robot_manager.collision_avoidance;

import com.google.common.collect.ImmutableList;
import com.google.common.graph.ImmutableValueGraph;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Deque;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public class CollisionAvoidance {
  private static final double ELEVATOR_TOLERANCE = 10.0;
  private static final double ARM_TOLERANCE = 10.0;
  private static final double CLIMBER_UNSAFE_ANGLE = 225.0;

  private static final ImmutableValueGraph<Waypoint, WaypointEdge> graph =
      GraphBuilder.createGraph();

  private static final Map<CollisionAvoidanceQuery, Optional<ImmutableList<Waypoint>>> aStarCache =
      new HashMap<>();

  private static CollisionAvoidanceQuery lastQuery =
      new CollisionAvoidanceQuery(Waypoint.L1_UPRIGHT, Waypoint.L1_UPRIGHT, ObstructionKind.NONE);
  private static double lastSolution = 90.0;
  private static boolean lastClimberRisky = true;
  private static ObstructionKind lastObstruction = ObstructionKind.NONE;
  private static ObstructionStrategy lastLeftStrategy = ObstructionStrategy.IGNORE_BLOCKED;
  private static ObstructionStrategy lastRightStrategy = ObstructionStrategy.IGNORE_BLOCKED;
  private static Waypoint lastWaypoint = Waypoint.L1_UPRIGHT;
  private static final Waypoint lastPreviousWaypoint = Waypoint.L1_UPRIGHT;

  private static Deque<Waypoint> lastPath = new ArrayDeque<>();

  private static boolean hasGeneratedPath = false;
  private static Waypoint previousWaypoint;

  /**
   * Returns an {@link Optional} containing the next {@link Waypoint} in the graph to go to. Returns
   * an empty Optional if there is no possible routing (impossible to avoid a collision or you are
   * at final waypoint).
   *
   * @param currentPosition The current position of the superstructure.
   * @param desiredPosition The desired position of the superstructure.
   * @param obstructionKind Additional constraints based on robot position.
   */
  public static Optional<SuperstructurePosition> routePosition(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind,
      double rawArmAngle) {
    double armGoal;
    var maybeWaypoint = route(currentPosition, desiredPosition, obstructionKind);
    if (maybeWaypoint.isEmpty()) {
      return Optional.empty();
    }
    Waypoint waypoint = maybeWaypoint.get();
    var maybeEdge = graph.edgeValue(previousWaypoint, waypoint);
    if (maybeEdge.isEmpty()) {
      return Optional.empty();
    }
    var edge = maybeEdge.orElseThrow();

    if (edge.hitsClimber() != lastClimberRisky
        // || obstructionKind != lastObstruction
        // || edge.get().leftSideStrategy() != lastLeftStrategy
        // || edge.get().rightSideStrategy() != lastRightStrategy
        || waypoint != lastWaypoint) {
      DogLog.timestamp("New Arm Goal Calculation");
      lastSolution =
          getCollisionAvoidanceAngleGoal(
              waypoint.position.armAngle(),
              edge.hitsClimber(),
              obstructionKind,
              edge.leftSideStrategy(),
              edge.rightSideStrategy(),
              rawArmAngle);
      lastClimberRisky = edge.hitsClimber();
      lastObstruction = obstructionKind;
      lastLeftStrategy = edge.leftSideStrategy();
      lastRightStrategy = edge.rightSideStrategy();
      lastWaypoint = waypoint;
    }

    DogLog.log(
        "CollisionAvoidance/CollisionAvoidanceAngleVariables/goalAngle",
        waypoint.position.armAngle());
    DogLog.log(
        "CollisionAvoidance/CollisionAvoidanceAngleVariables/hitsClimber", edge.hitsClimber());

    DogLog.log(
        "CollisionAvoidance/CollisionAvoidanceAngleVariables/obstructionKind", obstructionKind);

    DogLog.log(
        "CollisionAvoidance/CollisionAvoidanceAngleVariables/leftStrat", edge.leftSideStrategy());
    DogLog.log(
        "CollisionAvoidance/CollisionAvoidanceAngleVariables/rightStrat", edge.rightSideStrategy());
    DogLog.log("CollisionAvoidance/CollisionAvoidanceAngleVariables/RawArmAngle", rawArmAngle);
    //  DogLog.log("CollisionAvoidance/CollisionAvoidanceAngleVariables/edge", edge);
    DogLog.log(
        "CollisionAvoidance/CollisionAvoidanceAngleVariables/goalanglefar",
        desiredPosition.armAngle());
    DogLog.log(
        "CollisionAvoidance/CollisionAvoidanceAngleVariables/goalheightfar",
        desiredPosition.elevatorHeight());

    DogLog.log("CollisionAvoidance/CollisionAvoidanceAngleVariables/armsolution", lastSolution);

    return Optional.of(
        new SuperstructurePosition(waypoint.position.elevatorHeight(), lastSolution));
  }

  public static Optional<Waypoint> route(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {
    DogLog.log("CollisionAvoidance/ClawPos", currentPosition.getTranslation());

    var closestToCurrent = Waypoint.getClosest(currentPosition);
    var closestToDesired = Waypoint.getClosest(desiredPosition);

    if (DriverStation.isDisabled()) {
      return Optional.empty();
    }
    if (closestToCurrent == closestToDesired) {
      return Optional.empty();
    }
    DogLog.log("CollisionAvoidance/DesiredWaypoint", closestToDesired);
    // Check if the desired position and obstruction is the same, then use the same path
    if (!lastQuery.goalWaypoint().equals(closestToDesired)
        || !lastQuery.obstructionKind().equals(obstructionKind)) {
      lastQuery = new CollisionAvoidanceQuery(closestToCurrent, closestToDesired, obstructionKind);

      var maybePath = cachedAStar(lastQuery).map(ArrayDeque::new);
      if (maybePath.isPresent()) {
        hasGeneratedPath = true;
        lastPath = maybePath.orElseThrow();
      } else {
        return Optional.empty();
      }
    }

    if (lastPath.isEmpty()) {
      return Optional.empty();
    }

    var currentWaypoint = lastPath.getFirst();
    DogLog.log(
        "CollisionAvoidance/CurrentWaypoint/ElevatorHeight",
        currentWaypoint.position.elevatorHeight());
    DogLog.log("CollisionAvoidance/CurrentWaypoint/ArmAngle", currentWaypoint.position.armAngle());
    DogLog.log("CollisionAvoidance/CurrentWaypoint", currentWaypoint);
    DogLog.log("CollisionAvoidance/ClosestWaypoint", closestToCurrent);
    DogLog.log("CollisionAvoidance/AstarPath", lastPath.toArray(Waypoint[]::new));

    DogLog.log(
        "CollisionAvoidance/CurrentPosition/ElevatorHeight", currentPosition.elevatorHeight());
    DogLog.log("CollisionAvoidance/CurrentPosition/ArmAngle", currentPosition.armAngle());
    DogLog.log("CollisionAvoidance/Obstruction", obstructionKind);

    boolean near =
        currentPosition.isNear(currentWaypoint.position, ELEVATOR_TOLERANCE, ARM_TOLERANCE);
    DogLog.log("CollisionAvoidance/Near", near);

    // Check if our current position is close to the current waypoint in path
    if (near) {
      // If it's close, return the next waypoint
      if (lastPath.isEmpty()) {
        return Optional.empty();
      }
      previousWaypoint = currentWaypoint;

      return Optional.of(lastPath.pop());
    }
    // If it's not close, return the same waypoint
    return Optional.of(currentWaypoint);
  }

  public static double getShortSolution(
      double solution1, double solution2, double currentRawMotorAngle) {
    if (Math.abs(solution2 - currentRawMotorAngle) > Math.abs(solution1 - currentRawMotorAngle)) {
      return solution1;
    } else {
      return solution2;
    }
  }

  public static double[] getCollisionAvoidanceSolutions(
      double currentRawAngle, double normalizedGoalAngle) {
    // var normalizedCurrent = MathHelpers.angleModulus(currentRawAngle);
    // var dif = 0.0;
    // var expected1 = 0.0;
    // var expected2 = 0.0;

    // dif = normalizedGoalAngle - normalizedCurrent;
    // if (normalizedGoalAngle >= 0) {
    //   expected1 = currentRawAngle + dif;
    //   expected2 = currentRawAngle + (dif - 360);
    // } else {
    //   expected1 = currentRawAngle - (dif);
    //   expected2 = (currentRawAngle + 360) - dif;
    // }
    // return new double[] {expected1, expected2};

    // Find the closest lower multiple of 360 so that the unwrapped angle is near current

    int n = (int) currentRawAngle / 360;
    double solution1 = normalizedGoalAngle + 360 * n;
    double solution2 = solution1 + 360;
    double solution3 = solution1 - 360;

    var sorted = new ArrayList<Double>(List.of(solution1, solution2, solution3));
    sorted.sort(Comparator.comparingDouble(solution -> Math.abs(solution - currentRawAngle)));
    var closest = sorted.get(0);
    var secondClosest = sorted.get(1);

    // if (Math.abs(baseUnwrappedGoal - currentRawAngle)
    //     < Math.abs(altUnwrappedGoal - currentRawAngle)) {
    //   smallGoal = baseUnwrappedGoal;
    //   if (Math.abs(altUnwrappedGoal - currentRawAngle)
    //       < Math.abs(secondAltGoal - currentRawAngle)) {
    //     otherSmallGoal = altUnwrappedGoal;
    //   }
    //   {
    //     otherSmallGoal = secondAltGoal;
    //   }
    // } else {
    //   smallGoal = altUnwrappedGoal;
    //   if (Math.abs(baseUnwrappedGoal - currentRawAngle)
    //       < Math.abs(secondAltGoal - currentRawAngle)) {
    //     otherSmallGoal = baseUnwrappedGoal;
    //   } else {
    //     otherSmallGoal = secondAltGoal;
    //   }
    // }

    // System.out.println("1="+baseUnwrappedGoal);
    // System.out.println("2="+altUnwrappedGoal);
    // System.out.println("3="+otherSmallGoal);

    // Return both — determine which is CW/CCW externally if needed
    return new double[] {closest, secondClosest};
  }

  public static double getCollisionAvoidanceAngleGoal(
      double angle,
      boolean climberRisky,
      ObstructionKind currentObstructionKind,
      ObstructionStrategy leftObstructionStrategy,
      ObstructionStrategy rightObstructionStrategy,
      double currentRawMotorAngle) {
    double[] solutions = getCollisionAvoidanceSolutions(currentRawMotorAngle, angle);
    double solution1 = solutions[0];
    double solution2 = solutions[1];
    // System.out.println("1="+solution1);
    // System.out.println("2="+solution2);

    double shortSolution;
    double longSolution;

    int wrap = (int) currentRawMotorAngle / 360;

    double climberUnsafeAngle1 = (wrap * 360) - (360 - CLIMBER_UNSAFE_ANGLE);
    double climberUnsafeAngle2 = (wrap * 360) + CLIMBER_UNSAFE_ANGLE;

    if (climberRisky) {
      if ((Math.max(currentRawMotorAngle, solution1) >= climberUnsafeAngle1
              && Math.min(currentRawMotorAngle, solution1) <= climberUnsafeAngle1)
          || (Math.max(currentRawMotorAngle, solution1) >= climberUnsafeAngle2
              && Math.min(currentRawMotorAngle, solution1) <= climberUnsafeAngle2)) {
        // bad spot is in between the solution 1 path
        return solution2;
      }

      if ((Math.max(currentRawMotorAngle, solution2) > climberUnsafeAngle1
              && Math.min(currentRawMotorAngle, solution2) < climberUnsafeAngle1)
          || (Math.max(currentRawMotorAngle, solution2) > climberUnsafeAngle2
              && Math.min(currentRawMotorAngle, solution2) < climberUnsafeAngle2)) {
        // bad spot is in between the solution 2 path
        return solution1;
      }

    } else {
      // System.out.println("1 = "+solution1);
      // System.out.println("2 = "+solution2);
      // double solution1Difference = Math.abs(Math.max(solution1,
      // currentRawMotorAngle)-Math.min(solution1, currentRawMotorAngle));
      // double solution2Difference = Math.abs(Math.max(solution2,
      // currentRawMotorAngle)-Math.min(solution2, currentRawMotorAngle));
      // System.out.println("sol 1 diff"+solution1Difference);
      // System.out.println("sol 2 diff"+solution2Difference);

      //       if (solution2Difference > solution1Difference) {
      if (Math.abs(solution2 - currentRawMotorAngle) > Math.abs(solution1 - currentRawMotorAngle)) {
        shortSolution = solution1;
        longSolution = solution2;
      } else {
        shortSolution = solution2;
        longSolution = solution1;
      }
      System.out.println("short=" + shortSolution);
      System.out.println("long=" + longSolution);

      return switch (currentObstructionKind) {
        case LEFT_OBSTRUCTED ->
            switch (leftObstructionStrategy) {
              case IGNORE_BLOCKED -> shortSolution;
              case IMPOSSIBLE_IF_BLOCKED -> currentRawMotorAngle;
              case LONG_WAY_IF_BLOCKED -> longSolution;
            };
        case RIGHT_OBSTRUCTED ->
            switch (rightObstructionStrategy) {
              case IGNORE_BLOCKED -> shortSolution;
              case IMPOSSIBLE_IF_BLOCKED -> currentRawMotorAngle;
              case LONG_WAY_IF_BLOCKED -> longSolution;
            };
        default -> shortSolution;
      };
    }

    // System.out.println("1 = "+solution1);
    // System.out.println("2 = "+solution2);
    // double solution1Difference = Math.abs(Math.max(solution1,
    // currentRawMotorAngle)-Math.min(solution1, currentRawMotorAngle));
    // double solution2Difference = Math.abs(Math.max(solution2,
    // currentRawMotorAngle)-Math.min(solution2, currentRawMotorAngle));
    // System.out.println("sol 1 diff"+solution1Difference);
    // System.out.println("sol 2 diff"+solution2Difference);

    //       if (solution2Difference > solution1Difference) {
    if (Math.abs(solution2 - currentRawMotorAngle) > Math.abs(solution1 - currentRawMotorAngle)) {
      shortSolution = solution1;
      longSolution = solution2;
    } else {
      shortSolution = solution2;
      longSolution = solution1;
    }

    return switch (currentObstructionKind) {
      case LEFT_OBSTRUCTED ->
          switch (leftObstructionStrategy) {
            case IGNORE_BLOCKED -> shortSolution;
            case IMPOSSIBLE_IF_BLOCKED -> currentRawMotorAngle;
            case LONG_WAY_IF_BLOCKED -> longSolution;
          };
      case RIGHT_OBSTRUCTED ->
          switch (rightObstructionStrategy) {
            case IGNORE_BLOCKED -> shortSolution;
            case IMPOSSIBLE_IF_BLOCKED -> currentRawMotorAngle;
            case LONG_WAY_IF_BLOCKED -> longSolution;
          };
      case NONE -> shortSolution;
    };
  }

  private static Optional<ImmutableList<Waypoint>> cachedAStar(CollisionAvoidanceQuery query) {
    DogLog.log("CollisionAvoidance/AStarCacheSize", aStarCache.size());

    return aStarCache.computeIfAbsent(
        query,
        (k) ->
            aStar(
                query.currentWaypoint().position,
                query.goalWaypoint().position,
                query.obstructionKind()));
  }

  /**
   * Returns an {@link Optional} containing the next {@link Waypoint} in the graph to go to. Returns
   * an empty Optional if there is no possible routing (impossible to avoid a collision or you are
   * at final waypoint).
   */
  private static ImmutableList<Waypoint> reconstructPath(
      Map<Waypoint, Waypoint> cameFrom, Waypoint endWaypoint) {
    Deque<Waypoint> totalPath = new ArrayDeque<Waypoint>();
    totalPath.add(endWaypoint);
    Waypoint current = endWaypoint;
    while (cameFrom.containsKey(current)) {
      current = cameFrom.get(current);
      totalPath.addFirst(current);
    }

    return ImmutableList.copyOf(totalPath);
  }

  static Optional<ImmutableList<Waypoint>> aStar(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {
    return aStar(
        Waypoint.getClosest(currentPosition),
        Waypoint.getClosest(desiredPosition),
        obstructionKind);
  }

  static Optional<ImmutableList<Waypoint>> aStar(
      Waypoint startWaypoint,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {
    return aStar(startWaypoint, Waypoint.getClosest(desiredPosition), obstructionKind);
  }

  static Optional<ImmutableList<Waypoint>> aStar(
      SuperstructurePosition currentPosition,
      Waypoint endWaypoint,
      ObstructionKind obstructionKind) {
    return aStar(Waypoint.getClosest(currentPosition), endWaypoint, obstructionKind);
  }

  static Optional<ImmutableList<Waypoint>> aStar(
      Waypoint startWaypoint, Waypoint goalWaypoint, ObstructionKind obstructionKind) {
    var openSet = EnumSet.of(startWaypoint);

    Map<Waypoint, Waypoint> cameFrom = new EnumMap<Waypoint, Waypoint>(Waypoint.class);

    Map<Waypoint, Double> gscore = new EnumMap<Waypoint, Double>(Waypoint.class);

    if (startWaypoint.equals(goalWaypoint)) {
      DogLog.timestamp("CollisionAvoidance/StartAndEndSame");
      return Optional.empty();
    }

    gscore.put(startWaypoint, 0.0);
    Waypoint current = Waypoint.L1_UPRIGHT;
    while (!openSet.isEmpty()) {
      // current is equal to the waypoint in openset that has the smallest gscore
      var maybeCurrent =
          openSet.stream()
              .min(
                  Comparator.comparingDouble(
                      waypoint -> gscore.getOrDefault(waypoint, Double.MAX_VALUE)));
      if (maybeCurrent.isPresent()) {
        current = maybeCurrent.orElseThrow();
      }

      if (current == goalWaypoint) {
        var totalPath = reconstructPath(cameFrom, current);
        DogLog.clearFault("Collision avoidance path not possible");
        return Optional.of(totalPath);
      }
      openSet.remove(current);
      Set<Waypoint> options = graph.adjacentNodes(current);

      for (Waypoint neighbor : options) {
        var edge = graph.edgeValue(current, neighbor);
        double tentativeGScore =
            gscore.getOrDefault(current, Double.MAX_VALUE)
                + edge.orElseThrow().getCost(obstructionKind);
        if (tentativeGScore < gscore.getOrDefault(neighbor, Double.MAX_VALUE)) {
          cameFrom.put(neighbor, current);
          gscore.put(neighbor, tentativeGScore);
          openSet.add(neighbor);
        }
      }
    }
    DogLog.logFault("Collision avoidance path not possible", AlertType.kWarning);
    return Optional.of(ImmutableList.of(startWaypoint));
  }

  /**
   * Compute a few common paths to help the JIT warm up A* execution, and add some values to the
   * cache.
   */
  public static void warmup() {
    DogLog.time("CollisionAvoidance/Warmup");
    for (var obstruction : ObstructionKind.values()) {
      cachedAStar(
          new CollisionAvoidanceQuery(Waypoint.HANDOFF, Waypoint.L4_LEFT_LINEUP, obstruction));
      cachedAStar(
          new CollisionAvoidanceQuery(Waypoint.HANDOFF, Waypoint.L4_RIGHT_LINEUP, obstruction));
      cachedAStar(
          new CollisionAvoidanceQuery(Waypoint.HANDOFF, Waypoint.GROUND_ALGAE_INTAKE, obstruction));
      cachedAStar(
          new CollisionAvoidanceQuery(
              Waypoint.L1_UPRIGHT, Waypoint.ALGAE_NET_OUT_RIGHT, obstruction));
    }
    DogLog.timeEnd("CollisionAvoidance/Warmup");

    Waypoint.log();
  }

  /** Don't use this. */
  static ImmutableValueGraph<Waypoint, WaypointEdge> getRawGraph() {
    return graph;
  }

  public CollisionAvoidance() {}
}
