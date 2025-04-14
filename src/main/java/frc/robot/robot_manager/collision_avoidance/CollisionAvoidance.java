package frc.robot.robot_manager.collision_avoidance;

import com.google.common.collect.ImmutableList;
import com.google.common.graph.ElementOrder;
import com.google.common.graph.ImmutableValueGraph;
import com.google.common.graph.MutableValueGraph;
import com.google.common.graph.ValueGraphBuilder;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.arm.ArmState;
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
import java.util.stream.Stream;

public class CollisionAvoidance {
  private static final double ELEVATOR_TOLERANCE = 10.0;
  private static final double ARM_TOLERANCE = 10.0;
  private static final double CLIMBER_UNSAFE_ANGLE = 225.0;

  private static boolean hasBeenEnabled = false;

  private static final ImmutableValueGraph<Waypoint, WaypointEdge> graph = createGraph();

  private static final Map<CollisionAvoidanceQuery, Optional<ImmutableList<Waypoint>>> aStarCache =
      new HashMap<>();

  private static CollisionAvoidanceQuery lastQuery =
      new CollisionAvoidanceQuery(
          Waypoint.L1_UPRIGHT, Waypoint.L1_UPRIGHT, ObstructionKind.NONE, true);
  private static double lastSolution = 90.0;
  private static Waypoint lastWaypoint = Waypoint.L1_UPRIGHT;

  private static Deque<Waypoint> lastPath = new ArrayDeque<>();

  private static Optional<Waypoint> maybePreviousWaypoint = Optional.of(Waypoint.L1_UPRIGHT);
  private static Waypoint previousWaypoint = Waypoint.L1_UPRIGHT;

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
      double rawArmAngle,
      ArmState armState) {
    double armGoal;
    var maybeWaypoint = route(currentPosition, desiredPosition, obstructionKind);
    if (maybeWaypoint.isEmpty()) {
      return Optional.empty();
    }
    Waypoint waypoint = maybeWaypoint.get();
    if (maybePreviousWaypoint.isEmpty()) {
      previousWaypoint = Waypoint.getClosest(currentPosition);
    } else {
      previousWaypoint = maybePreviousWaypoint.get();
    }

    var maybeEdge = graph.edgeValue(previousWaypoint, waypoint);
    if (DriverStation.isEnabled()) {
      hasBeenEnabled = true;
    }
    if (waypoint != lastWaypoint || !hasBeenEnabled || armState == ArmState.PRE_MATCH_HOMING) {
      hasBeenEnabled = true;
      if (maybeEdge.isEmpty()) {
        return Optional.empty();
      }
      var edge = maybeEdge;
      DogLog.timestamp("New Arm Goal Calculation");
      lastSolution =
          getCollisionAvoidanceAngleGoal(
              waypoint.position.armAngle(),
              edge.get().hitsClimber(),
              obstructionKind,
              edge.get().leftSideStrategy(),
              edge.get().rightSideStrategy(),
              rawArmAngle);
      lastWaypoint = waypoint;
    }
    DogLog.log("CollisionAvoidance/ArmSolution", lastSolution);
    return Optional.of(
        new SuperstructurePosition(waypoint.position.elevatorHeight(), lastSolution));
  }

  public static Optional<Waypoint> route(
      SuperstructurePosition currentPosition,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {
    DogLog.log("CollisionAvoidance/ClawPos", currentPosition.translation());

    var closestToCurrent = Waypoint.getClosest(currentPosition);
    var closestToDesired = Waypoint.getClosest(desiredPosition);

    if (closestToCurrent == closestToDesired) {
      return Optional.empty();
    }
    DogLog.log("CollisionAvoidance/DesiredWaypoint", closestToDesired);
    // Check if the desired position is the same, then use the same path
    if (!lastQuery.goalWaypoint().equals(closestToDesired) || DriverStation.isDisabled()) {
      lastQuery =
          new CollisionAvoidanceQuery(
              closestToCurrent, closestToDesired, obstructionKind, DriverStation.isTeleop());

      var maybePath = cachedAStar(lastQuery).map(ArrayDeque::new);
      if (maybePath.isPresent()) {
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
      maybePreviousWaypoint = Optional.of(currentWaypoint);

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

    // Find the closest lower multiple of 360 so that the unwrapped angle is near current

    int n = (int) currentRawAngle / 360;
    double solution1 = normalizedGoalAngle + 360 * n;
    double solution2 = solution1 + 360;
    double solution3 = solution1 - 360;

    var sorted = new ArrayList<Double>(List.of(solution1, solution2, solution3));
    sorted.sort(Comparator.comparingDouble(solution -> Math.abs(solution - currentRawAngle)));
    var closest = sorted.get(0);
    var secondClosest = sorted.get(1);

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
        default -> shortSolution;
      };
    }
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

  private static ImmutableValueGraph<Waypoint, WaypointEdge> createGraph() {
    // Create an undirected value graph to represent safe motion between waypoints. Undirected
    // because if you can go from A to B, you can also go from B to A. Value graph because we want
    // to associate a cost with motion between different waypoints.
    MutableValueGraph<Waypoint, WaypointEdge> graph =
        ValueGraphBuilder.undirected().incidentEdgeOrder(ElementOrder.stable()).build();

    // Try to categorize blocks based on the kinds of collision that may happen
    // Sort L2 before L3, L3 before L4
    // Always have left on the left, and right on the right

    /* If your arm angle doesn't change, you can do whatever with elevator */
    var armStraightUpWaypoints =
        Stream.of(Waypoint.values())
            .filter(waypoint -> waypoint.position.armAngle() == ArmState.HOLDING_UPRIGHT.getAngle())
            .toList();
    var armStraightDownWaypoints =
        Stream.of(Waypoint.values())
            .filter(waypoint -> waypoint.position.armAngle() == ArmState.CORAL_HANDOFF.getAngle())
            .toList();

    for (var a : armStraightUpWaypoints) {
      for (var b : armStraightUpWaypoints) {
        if (a == b) {
          // Skip because it's the same waypoint
          continue;
        }

        a.alwaysSafe(graph, b);
      }
    }

    for (var a : armStraightDownWaypoints) {
      for (var b : armStraightDownWaypoints) {
        if (a == b) {
          // Skip because it's the same waypoint
          continue;
        }

        a.alwaysSafe(graph, b);
      }
    }

    // TODO: If it's possible, allow going HANDOFF_CLEARS_CLIMBER to L1_UPRIGHT
    Waypoint.HANDOFF_CLEARS_CLIMBER.avoidClimberAlwaysSafe(graph, Waypoint.L2_UPRIGHT);
    Waypoint.HANDOFF.avoidClimberAlwaysSafe(
        graph, Waypoint.ELEVATOR_HANDOFF_ARM_GROUND_ALGAE_INTAKE);

    /* Arm up to left/right is always safe */
    Waypoint.L2_UPRIGHT.avoidClimberAlwaysSafe(
        graph,
        Waypoint.L2_LEFT_LINEUP,
        Waypoint.L2_RIGHT_LINEUP,
        Waypoint.GROUND_ALGAE_INTAKE,
        Waypoint.PROCESSOR,
        Waypoint.LOLLIPOP_INTAKE_RIGHT,
        Waypoint.LOLLIPOP_INTAKE_PUSH);
    Waypoint.L3_UPRIGHT.avoidClimberAlwaysSafe(
        graph,
        Waypoint.L3_LEFT_LINEUP,
        Waypoint.L3_RIGHT_LINEUP,
        Waypoint.GROUND_ALGAE_INTAKE,
        Waypoint.PROCESSOR,
        Waypoint.LOLLIPOP_INTAKE_RIGHT,
        Waypoint.LOLLIPOP_INTAKE_PUSH);
    Waypoint.L4_UPRIGHT.avoidClimberAlwaysSafe(
        graph,
        Waypoint.L4_LEFT_LINEUP,
        Waypoint.L4_RIGHT_LINEUP,
        Waypoint.GROUND_ALGAE_INTAKE,
        Waypoint.PROCESSOR,
        Waypoint.LOLLIPOP_INTAKE_RIGHT,
        Waypoint.LOLLIPOP_INTAKE_PUSH);
    Waypoint.ALGAE_NET_UP.alwaysSafe(
        graph, Waypoint.ALGAE_NET_OUT_LEFT, Waypoint.ALGAE_NET_OUT_RIGHT);

    // If you aren't going to hit reef poles, you can skip the in between upright waypoints
    Waypoint.L2_UPRIGHT.avoidClimberLeftSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L3_LEFT_LINEUP,
        Waypoint.L4_LEFT_LINEUP);
    Waypoint.L3_UPRIGHT.avoidClimberLeftSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L2_LEFT_LINEUP,
        Waypoint.L4_LEFT_LINEUP);
    Waypoint.L4_UPRIGHT.avoidClimberLeftSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L2_LEFT_LINEUP,
        Waypoint.L3_LEFT_LINEUP);

    Waypoint.L1_UPRIGHT.rightSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L2_RIGHT_LINEUP,
        Waypoint.L3_RIGHT_LINEUP,
        Waypoint.L4_RIGHT_LINEUP);
    Waypoint.L2_UPRIGHT.rightSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L1_RIGHT_LINEUP,
        Waypoint.L3_RIGHT_LINEUP,
        Waypoint.L4_RIGHT_LINEUP);
    Waypoint.L3_UPRIGHT.rightSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L1_RIGHT_LINEUP,
        Waypoint.L2_RIGHT_LINEUP,
        Waypoint.L4_RIGHT_LINEUP);
    Waypoint.L4_UPRIGHT.rightSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L1_RIGHT_LINEUP,
        Waypoint.L2_RIGHT_LINEUP,
        Waypoint.L3_RIGHT_LINEUP);

    /* Each lineup state is safe to connect to its associated place state */
    Waypoint.L2_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L2_LEFT_PLACE);
    Waypoint.L3_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L3_LEFT_PLACE);
    Waypoint.L4_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L4_LEFT_PLACE);
    Waypoint.L2_RIGHT_LINEUP.alwaysSafe(graph, Waypoint.L2_RIGHT_PLACE);
    Waypoint.L3_RIGHT_LINEUP.alwaysSafe(graph, Waypoint.L3_RIGHT_PLACE);
    Waypoint.L4_RIGHT_LINEUP.alwaysSafe(graph, Waypoint.L4_RIGHT_PLACE);

    Waypoint.L2_LEFT_PLACE.alwaysSafe(graph, Waypoint.L2_LEFT_ARM);
    Waypoint.L3_LEFT_PLACE.alwaysSafe(graph, Waypoint.L3_LEFT_ARM);

    Waypoint.L2_RIGHT_PLACE.alwaysSafe(graph, Waypoint.L2_RIGHT_ARM);
    Waypoint.L3_RIGHT_PLACE.alwaysSafe(graph, Waypoint.L3_RIGHT_ARM);

    // Place

    // You can always go to the push state after grabbing a lollipop coral
    Waypoint.LOLLIPOP_INTAKE_RIGHT.alwaysSafe(graph, Waypoint.LOLLIPOP_INTAKE_PUSH);
    // Used in lollipop auto to go directly to/from scoring L4/L3
    Waypoint.LOLLIPOP_INTAKE_PUSH.avoidClimberAlwaysSafe(
        graph, Waypoint.L2_LEFT_LINEUP, Waypoint.L3_LEFT_LINEUP, Waypoint.L4_LEFT_LINEUP);
    Waypoint.LOLLIPOP_INTAKE_RIGHT.avoidClimberAlwaysSafe(
        graph,
        Waypoint.L3_LEFT_PLACE,
        Waypoint.L4_LEFT_PLACE,
        Waypoint.L2_LEFT_ARM,
        Waypoint.L2_RIGHT_ARM);

    // Additional ways to exit the push state, mostly just to help cover weird edge cases
    // Putting L1_UPRIGHT here is safe, but makes going L4_LEFT_PLACE to L1_UPRIGHT weird
    Waypoint.LOLLIPOP_INTAKE_RIGHT.avoidClimberAlwaysSafe(
        graph, Waypoint.L2_UPRIGHT, Waypoint.L3_UPRIGHT, Waypoint.L4_UPRIGHT);
    Waypoint.LOLLIPOP_INTAKE_PUSH.avoidClimberAlwaysSafe(
        graph, Waypoint.L2_UPRIGHT, Waypoint.L3_UPRIGHT, Waypoint.L4_UPRIGHT);

    /* Switching coral level on the same side is okay if you won't hit the reef */
    var leftCoralScoreWaypoints =
        List.of(Waypoint.L2_LEFT_LINEUP, Waypoint.L3_LEFT_LINEUP, Waypoint.L4_LEFT_LINEUP);
    var rightCoralScoreWaypoints =
        List.of(
            Waypoint.L1_RIGHT_LINEUP,
            Waypoint.L2_RIGHT_LINEUP,
            Waypoint.L3_RIGHT_LINEUP,
            Waypoint.L4_RIGHT_LINEUP);

    for (var a : leftCoralScoreWaypoints) {
      for (var b : leftCoralScoreWaypoints) {
        if (a == b) {
          // Skip because it's the same waypoint
          continue;
        }

        a.leftSideSpecial(graph, ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED, b);
      }
    }

    for (var a : rightCoralScoreWaypoints) {
      for (var b : rightCoralScoreWaypoints) {
        if (a == b) {
          // Skip because it's the same waypoint
          continue;
        }

        a.rightSideSpecial(graph, ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED, b);
      }
    }

    /* Scoring coral directly from handoff, depends a lot on obstructions */
    Waypoint.HANDOFF_CLEARS_CLIMBER.leftSideSpecial(
        graph, ObstructionStrategy.LONG_WAY_IF_BLOCKED, Waypoint.L4_LEFT_LINEUP);

    Waypoint.HANDOFF_CLEARS_CLIMBER.rightSideSpecial(
        graph, ObstructionStrategy.LONG_WAY_IF_BLOCKED, Waypoint.L4_RIGHT_LINEUP);
    Waypoint.HANDOFF.rightSideSpecial(
        graph,
        ObstructionStrategy.LONG_WAY_IF_BLOCKED,
        Waypoint.L2_RIGHT_ARM,
        Waypoint.L3_RIGHT_ARM);
    Waypoint.HANDOFF.leftSideSpecial(
        graph, ObstructionStrategy.LONG_WAY_IF_BLOCKED, Waypoint.L2_LEFT_ARM, Waypoint.L3_LEFT_ARM);

    Waypoint.L2_LEFT_ARM.alwaysSafe(graph, Waypoint.L2_LEFT_LINEUP);
    Waypoint.L3_LEFT_ARM.alwaysSafe(graph, Waypoint.L3_LEFT_LINEUP);
    Waypoint.L2_RIGHT_ARM.alwaysSafe(graph, Waypoint.L2_RIGHT_LINEUP);
    Waypoint.L3_RIGHT_ARM.alwaysSafe(graph, Waypoint.L3_RIGHT_LINEUP);

    Waypoint.HANDOFF_CLEARS_CLIMBER.alwaysSafe(graph, Waypoint.ALGAE_NET_UP);

    // L1 movements
    var l1AreaWaypoints =
        List.of(
            Waypoint.L1_RIGHT_LINEUP,
            Waypoint.GROUND_ALGAE_INTAKE,
            Waypoint.PROCESSOR,
            Waypoint.CLIMB);

    Waypoint.L1_UPRIGHT.alwaysSafe(graph, l1AreaWaypoints.toArray(Waypoint[]::new));

    Waypoint.HANDOFF.alwaysSafe(graph, Waypoint.HANDOFF_CLEARS_CLIMBER);

    for (var a : l1AreaWaypoints) {
      for (var b : l1AreaWaypoints) {
        if (a == b) {
          // Skip because it's the same waypoint
          continue;
        }

        a.alwaysSafe(graph, b);
      }
    }

    Waypoint.ELEVATOR_HANDOFF_ARM_GROUND_ALGAE_INTAKE.alwaysSafe(
        graph, Waypoint.GROUND_ALGAE_INTAKE);

    /* Reef algae */
    Waypoint.HANDOFF_CLEARS_CLIMBER.alwaysSafe(
        graph,
        Waypoint.REEF_ALGAE_L2_LEFT,
        Waypoint.REEF_ALGAE_L3_LEFT,
        Waypoint.REEF_ALGAE_L2_RIGHT,
        Waypoint.REEF_ALGAE_L3_RIGHT);
    // Algae stow position can go directly to any of the intake states
    Waypoint.L1_UPRIGHT.avoidClimberAlwaysSafe(
        graph,
        Waypoint.REEF_ALGAE_L2_LEFT,
        Waypoint.REEF_ALGAE_L2_RIGHT,
        Waypoint.REEF_ALGAE_L3_RIGHT,
        Waypoint.REEF_ALGAE_L3_LEFT);
    // Create an immutable copy of the graph now that we've added all the nodes
    var immutableGraph = ImmutableValueGraph.copyOf(graph);

    // Visualize the generated graph
    Waypoint.log();

    return immutableGraph;
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
      Waypoint startWaypoint,
      SuperstructurePosition desiredPosition,
      ObstructionKind obstructionKind) {
    return aStar(startWaypoint, Waypoint.getClosest(desiredPosition), obstructionKind);
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
      SuperstructurePosition currentPosition,
      Waypoint goalWaypoint,
      ObstructionKind obstructionKind) {
    return aStar(Waypoint.getClosest(currentPosition), goalWaypoint, obstructionKind);
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
        maybePreviousWaypoint = Optional.of(totalPath.get(0));
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
    for (var teleop : new boolean[] {true, false}) {
      for (var obstruction : ObstructionKind.values()) {
        cachedAStar(
            new CollisionAvoidanceQuery(
                Waypoint.HANDOFF, Waypoint.L4_LEFT_LINEUP, obstruction, teleop));
        cachedAStar(
            new CollisionAvoidanceQuery(
                Waypoint.HANDOFF, Waypoint.L4_RIGHT_LINEUP, obstruction, teleop));
        cachedAStar(
            new CollisionAvoidanceQuery(
                Waypoint.HANDOFF, Waypoint.GROUND_ALGAE_INTAKE, obstruction, teleop));
        cachedAStar(
            new CollisionAvoidanceQuery(
                Waypoint.L1_UPRIGHT, Waypoint.ALGAE_NET_OUT_RIGHT, obstruction, teleop));
      }
    }
    DogLog.timeEnd("CollisionAvoidance/Warmup");
  }

  /** Don't use this. */
  static ImmutableValueGraph<Waypoint, WaypointEdge> getRawGraph() {
    return graph;
  }

  public CollisionAvoidance() {}
}
