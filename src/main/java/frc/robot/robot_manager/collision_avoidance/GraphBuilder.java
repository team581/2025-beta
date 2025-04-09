package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.ElementOrder;
import com.google.common.graph.ImmutableValueGraph;
import com.google.common.graph.MutableValueGraph;
import com.google.common.graph.ValueGraphBuilder;
import edu.wpi.first.math.Pair;
import frc.robot.arm.ArmState;
import java.util.List;
import java.util.stream.Stream;

final class GraphBuilder {
  public static ImmutableValueGraph<Waypoint, WaypointEdge> createGraph() {
    // Create an undirected value graph to represent safe motion between waypoints. Undirected
    // because if you can go from A to B, you can also go from B to A. Value graph because we want
    // to associate a cost with motion between different waypoints.
    MutableValueGraph<Waypoint, WaypointEdge> graph =
        ValueGraphBuilder.undirected().incidentEdgeOrder(ElementOrder.stable()).build();

    addStraightUpDownMovements(graph);
    addCoralScoring(graph);
    addCoralScoringSideChanges(graph);
    addCoralScoringLevelChanges(graph);
    addCoralScoringSideAndLevelChanges(graph);
    addHandoffToCoralScoring(graph);
    addReefIntake(graph);
    addHandoffToReefIntake(graph);
    addCoralScoringToReefIntake(graph);
    addReefIntakeSideChanges(graph);
    addNetAlgae(graph);
    addGroundAndProcessorAlgae(graph);
    addOpenSideArea(graph);
    addHandoff(graph);

    // Create an immutable copy of the graph now that we've added all the nodes
    var immutableGraph = ImmutableValueGraph.copyOf(graph);

    return immutableGraph;
  }

  /**
   * Movements where the arm is straight up or down. If the arm doesn't move at all, basically any
   * elevator motion is safe.
   */
  private static void addStraightUpDownMovements(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    var armStraightUpWaypoints =
        Stream.of(Waypoint.values())
            // We exclude net algae since we don't want to swing and hit net on the way down
            .filter(
                waypoint ->
                    waypoint.position.armAngle() == ArmState.HOLDING_UPRIGHT.getAngle()
                        && waypoint != Waypoint.ALGAE_NET_UP)
            .toArray(Waypoint[]::new);

    for (var pair : allPairs(armStraightUpWaypoints)) {
      pair.getFirst().alwaysSafe(graph, pair.getSecond());
    }

    var armStraightDownWaypoints =
        Stream.of(Waypoint.values())
            .filter(waypoint -> waypoint.position.armAngle() == ArmState.CORAL_HANDOFF.getAngle())
            .toArray(Waypoint[]::new);

    for (var pair : allPairs(armStraightDownWaypoints)) {
      pair.getFirst().alwaysSafe(graph, pair.getSecond());
    }
  }

  private static void addHandoff(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    // TODO: Need to determine if this is safe
    Waypoint.HANDOFF_CLEARS_CLIMBER.avoidClimberAlwaysSafe(
        graph, Waypoint.L1_UPRIGHT, Waypoint.L2_UPRIGHT, Waypoint.L3_UPRIGHT, Waypoint.L4_UPRIGHT);
  }

  private static void addHandoffToCoralScoring(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    Waypoint.HANDOFF_CLEARS_CLIMBER.leftSideSpecial(
        graph,
        ObstructionStrategy.LONG_WAY_IF_BLOCKED,
        Waypoint.L2_LEFT_LINEUP,
        Waypoint.L3_LEFT_LINEUP,
        Waypoint.L4_LEFT_LINEUP);
    Waypoint.HANDOFF_CLEARS_CLIMBER.rightSideSpecial(
        graph,
        ObstructionStrategy.LONG_WAY_IF_BLOCKED,
        Waypoint.L2_RIGHT_LINEUP,
        Waypoint.L3_RIGHT_LINEUP,
        Waypoint.L4_RIGHT_LINEUP);

    // TODO: Need to determine if this is safe
    Waypoint.HANDOFF.avoidClimberLeftSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L2_LEFT_LINEUP,
        Waypoint.L3_LEFT_LINEUP,
        Waypoint.L4_LEFT_LINEUP);
    // TODO: Need to determine if this is safe
    Waypoint.HANDOFF.rightSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L2_RIGHT_LINEUP,
        Waypoint.L3_RIGHT_LINEUP,
        Waypoint.L4_RIGHT_LINEUP);
  }

  /** If the arm is upright, you can move left/right as long as elevator doesn't really change. */
  private static void addCoralScoring(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    // Elevator doesn't move, arm just moves left/right
    Waypoint.L2_UPRIGHT.avoidClimberAlwaysSafe(
        graph,
        Waypoint.L2_LEFT_LINEUP,
        Waypoint.L2_LEFT_PLACE,
        Waypoint.L2_RIGHT_LINEUP,
        Waypoint.L2_RIGHT_PLACE);

    // L2 lineup can do L2 place
    Waypoint.L2_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L2_LEFT_PLACE);
    Waypoint.L2_RIGHT_LINEUP.alwaysSafe(graph, Waypoint.L2_RIGHT_PLACE);

    // Elevator doesn't move, arm just moves left/right
    Waypoint.L3_UPRIGHT.alwaysSafe(
        graph,
        Waypoint.L3_LEFT_LINEUP,
        Waypoint.L3_LEFT_PLACE,
        Waypoint.L3_RIGHT_LINEUP,
        Waypoint.L3_RIGHT_PLACE);

    // L3 lineup can do L3 place
    Waypoint.L3_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L3_LEFT_PLACE);
    Waypoint.L3_RIGHT_LINEUP.alwaysSafe(graph, Waypoint.L3_RIGHT_PLACE);

    // Elevator doesn't move, arm just moves left/right
    Waypoint.L4_UPRIGHT.alwaysSafe(
        graph,
        Waypoint.L4_LEFT_LINEUP,
        Waypoint.L4_LEFT_PLACE,
        Waypoint.L4_RIGHT_LINEUP,
        Waypoint.L4_RIGHT_PLACE);

    // L4 lineup can do L4 place
    Waypoint.L4_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L4_LEFT_PLACE);
    Waypoint.L4_RIGHT_LINEUP.alwaysSafe(graph, Waypoint.L4_RIGHT_PLACE);
  }

  /** Changing the side of where you are scoring cora, but not the height. */
  private static void addCoralScoringSideChanges(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    Waypoint.L2_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L2_RIGHT_LINEUP);
    Waypoint.L3_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L3_RIGHT_LINEUP);
    Waypoint.L4_LEFT_LINEUP.alwaysSafe(graph, Waypoint.L4_RIGHT_LINEUP);
  }

  /** Changing the height of where you are scoring coral, but not the left/right side. */
  private static void addCoralScoringLevelChanges(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    for (var pair :
        allPairs(Waypoint.L2_LEFT_LINEUP, Waypoint.L3_LEFT_LINEUP, Waypoint.L4_LEFT_LINEUP)) {
      pair.getFirst()
          .leftSideSpecial(graph, ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED, pair.getSecond());
    }

    for (var pair :
        allPairs(
            Waypoint.L1_RIGHT_LINEUP,
            Waypoint.L2_RIGHT_LINEUP,
            Waypoint.L3_RIGHT_LINEUP,
            Waypoint.L4_RIGHT_LINEUP)) {
      pair.getFirst()
          .rightSideSpecial(graph, ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED, pair.getSecond());
    }
  }

  /** Changing both the side and the level of where you are scoring coral. */
  private static void addCoralScoringSideAndLevelChanges(
      MutableValueGraph<Waypoint, WaypointEdge> graph) {
    // If you aren't going to hit reef poles, you can skip the in between upright waypoints
    Waypoint.L2_UPRIGHT.leftSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L3_LEFT_LINEUP,
        Waypoint.L4_LEFT_LINEUP);
    Waypoint.L3_UPRIGHT.leftSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L2_LEFT_LINEUP,
        Waypoint.L4_LEFT_LINEUP);
    Waypoint.L4_UPRIGHT.leftSideSpecial(
        graph,
        ObstructionStrategy.IMPOSSIBLE_IF_BLOCKED,
        Waypoint.L2_LEFT_LINEUP,
        Waypoint.L3_LEFT_LINEUP);

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
  }

  private static void addReefIntake(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    Waypoint.REEF_ALGAE_L2_UPRIGHT.alwaysSafe(
        graph, Waypoint.REEF_ALGAE_L2_LEFT, Waypoint.REEF_ALGAE_L2_RIGHT);
    Waypoint.REEF_ALGAE_L3_UPRIGHT.alwaysSafe(
        graph, Waypoint.REEF_ALGAE_L3_LEFT, Waypoint.REEF_ALGAE_L3_RIGHT);

    // TODO: Is there a way to go from L1_UPRIGHT to left side reef algae without hitting climber?

    Waypoint.L1_UPRIGHT.alwaysSafe(
        graph, Waypoint.REEF_ALGAE_L2_RIGHT, Waypoint.REEF_ALGAE_L3_RIGHT);
  }

  /**
   * This only happens if we're doing dedicated algae cycles, which isn't super common but still
   * happens fairly often.
   */
  private static void addHandoffToReefIntake(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    Waypoint.HANDOFF.alwaysSafe(graph, Waypoint.REEF_ALGAE_L2_RIGHT, Waypoint.REEF_ALGAE_L3_RIGHT);
    Waypoint.HANDOFF.avoidClimberAlwaysSafe(
        graph, Waypoint.REEF_ALGAE_L2_LEFT, Waypoint.REEF_ALGAE_L3_LEFT);
    Waypoint.HANDOFF_CLEARS_CLIMBER.alwaysSafe(
        graph,
        Waypoint.REEF_ALGAE_L2_LEFT,
        Waypoint.REEF_ALGAE_L3_LEFT,
        Waypoint.REEF_ALGAE_L2_RIGHT,
        Waypoint.REEF_ALGAE_L3_RIGHT);
  }

  private static void addCoralScoringToReefIntake(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    var leftReefAlgae = new Waypoint[] {Waypoint.REEF_ALGAE_L2_LEFT, Waypoint.REEF_ALGAE_L3_LEFT};
    var rightReefAlgae =
        new Waypoint[] {Waypoint.REEF_ALGAE_L2_RIGHT, Waypoint.REEF_ALGAE_L3_RIGHT};

    for (var leftScoringState :
        new Waypoint[] {
          Waypoint.L2_LEFT_LINEUP,
          Waypoint.L3_LEFT_LINEUP,
          Waypoint.L4_LEFT_LINEUP,
          Waypoint.L2_LEFT_PLACE,
          Waypoint.L3_LEFT_PLACE,
          Waypoint.L4_LEFT_PLACE,
        }) {
      leftScoringState.alwaysSafe(graph, leftReefAlgae);
    }

    for (var rightScoringState :
        new Waypoint[] {
          Waypoint.L1_RIGHT_LINEUP,
          Waypoint.L2_RIGHT_LINEUP,
          Waypoint.L3_RIGHT_LINEUP,
          Waypoint.L4_RIGHT_LINEUP,
          Waypoint.L2_RIGHT_PLACE,
          Waypoint.L3_RIGHT_PLACE,
          Waypoint.L4_RIGHT_PLACE
        }) {
      rightScoringState.alwaysSafe(graph, rightReefAlgae);
    }
  }

  private static void addReefIntakeSideChanges(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    Waypoint.REEF_ALGAE_L2_LEFT.alwaysSafe(graph, Waypoint.REEF_ALGAE_L2_RIGHT);
    Waypoint.REEF_ALGAE_L3_LEFT.alwaysSafe(graph, Waypoint.REEF_ALGAE_L3_RIGHT);
  }

  private static void addNetAlgae(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    // Net scoring left/right can only pass through the net upright position
    Waypoint.ALGAE_NET_UP.alwaysSafe(
        graph, Waypoint.ALGAE_NET_OUT_LEFT, Waypoint.ALGAE_NET_OUT_RIGHT);

    // The only way to get to algae net upright is by coming down sufficiently, so you don't hit the
    // net with the arm
    Waypoint.ALGAE_NET_UP.alwaysSafe(graph, Waypoint.L3_UPRIGHT);
  }

  private static void addGroundAndProcessorAlgae(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    // Ensures we have the arm in the correct position before we lower the elevator
    Waypoint.HANDOFF.alwaysSafe(graph, Waypoint.HANDOFF_TO_GROUND_ALGAE_INTAKE);
    Waypoint.HANDOFF_TO_GROUND_ALGAE_INTAKE.alwaysSafe(graph, Waypoint.GROUND_ALGAE_INTAKE);

    // When holding algae, you can always go straight to processor
    Waypoint.L1_UPRIGHT.alwaysSafe(graph, Waypoint.PROCESSOR);

    // You can also go straight to processor from intaking
    Waypoint.GROUND_ALGAE_INTAKE.alwaysSafe(graph, Waypoint.PROCESSOR);
  }

  /** The collection of waypoints on the open (right) side of the robot. */
  private static void addOpenSideArea(MutableValueGraph<Waypoint, WaypointEdge> graph) {
    var openAreaWaypoints =
        new Waypoint[] {
          Waypoint.L1_RIGHT_LINEUP,
          Waypoint.LOLLIPOP_INTAKE_RIGHT_APPROACH,
          Waypoint.LOLLIPOP_INTAKE_RIGHT,
          Waypoint.LOLLIPOP_INTAKE_RIGHT_PUSH,
          // Ground algae and processor also have their own section
          Waypoint.GROUND_ALGAE_INTAKE,
          Waypoint.PROCESSOR
        };

    for (var startPosition :
        new Waypoint[] {
          Waypoint.L1_UPRIGHT,
          Waypoint.L2_UPRIGHT,
          Waypoint.L3_UPRIGHT,
          Waypoint.L4_UPRIGHT,
          Waypoint.HANDOFF_CLEARS_CLIMBER
        }) {
      startPosition.avoidClimberAlwaysSafe(graph, openAreaWaypoints);
    }

    for (var pair : allPairs(openAreaWaypoints)) {
      pair.getFirst().alwaysSafe(graph, pair.getSecond());
    }
  }

  private static List<Pair<Waypoint, Waypoint>> allPairs(Waypoint... waypoints) {
    return Stream.of(waypoints).flatMap(a -> Stream.of(waypoints).map(b -> Pair.of(a, b))).toList();
  }

  private GraphBuilder() {}
}
