package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.commands.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import java.util.ArrayList;
import java.util.function.Supplier;

public class MechanismActions {
  private static final String ROOT_TABLE = "MechanismActions";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Bool log_runningArm = logGroup.buildBoolean("runningArm");
  private static final LoggerEntry.Bool log_runningElevator =
      logGroup.buildBoolean("runningElevator");
  private static final LoggerEntry.Bool log_runningPivot = logGroup.buildBoolean("runningPivot");
  private static final LoggerEntry.Bool log_ElevatorInPosition =
      logGroup.buildBoolean("ElevatorInPosition");
  private static final LoggerEntry.Bool log_PivotInPosition =
      logGroup.buildBoolean("PivotInPosition");
  private static final LoggerEntry.Bool log_ArmInPosition = logGroup.buildBoolean("ArmInPosition");
  private static final LoggerEntry.Bool log_AtPathEnd = logGroup.buildBoolean("AtPathEnd");
  private static final LoggerEntry.Integer log_PathIndex = logGroup.buildInteger("PathIndex");
  private static final LoggerEntry.Integer log_SafePathIndex =
      logGroup.buildInteger("SafePathIndex");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  public static final LoggedTunableNumber climbDownElevatorVelocity =
      group.build("MechanismActions/climbDownElevatorVelocity", 500);
  public static final LoggedTunableNumber climbDownElevatorAcceleration =
      group.build("climbDownElevatorAcceleration", 500);

  public static final LoggedTunableNumber tunableArmVoltage =
      group.build("MechanismActions/tunableArmVoltage", 5.0);

  public static Command reefPosition(
      Elevator elevator, Arm arm, Intake intake, ScoringLevel scoringLevel) {
    return goToPositionParallel(
        elevator, arm, intake, () -> MechanismPositions.reefPosition(scoringLevel));
  }

  public static Command coralStationPosition(Elevator elevator, Arm arm, Intake intake) {
    return goToPositionParallel(elevator, arm, intake, MechanismPositions::coralStationPosition);
  }

  public static Command stowPosition(Elevator elevator, Arm arm, Intake intake) {
    return goToPositionParallel(elevator, arm, intake, MechanismPositions::stowPosition);
  }

  public static Command clearAlgaeLow1Position(Elevator elevator, Arm arm, Intake intake) {
    return goToPositionParallel(elevator, arm, intake, MechanismPositions::clearAlgaeLow1Position);
  }

  public static Command clearAlgaeLow2Position(Elevator elevator, Arm arm, Intake intake) {
    return goToPositionParallel(elevator, arm, intake, MechanismPositions::clearAlgaeLow2Position);
  }

  public static Command clearAlgaeHigh1Position(Elevator elevator, Arm arm, Intake intake) {
    return goToPositionParallel(elevator, arm, intake, MechanismPositions::clearAlgaeHigh1Position);
  }

  public static Command clearAlgaeHigh2Position(Elevator elevator, Arm arm, Intake intake) {
    return goToPositionParallel(elevator, arm, intake, MechanismPositions::clearAlgaeHigh2Position);
  }

  // TODO: Change Logic for 2025 Robot Geometry
  private static Command goToPositionParallel(
      Elevator elevator, Arm arm, Intake intake, Supplier<MechanismPosition> position) {
    return goToPositionParallel(elevator, arm, intake, position, false);
  }

  // TODO: get actual positions and connections
  // for later: when adding more positions make sure to add it to the safePositions list
  // and add its corresponding connections to the connections list
  private static MechanismPosition[] safePositions = {
    MechanismPositions.stowPosition(),
    MechanismPositions.reefPosition(ScoringLevel.L1),
    MechanismPositions.reefPosition(ScoringLevel.L2),
    MechanismPositions.reefPosition(ScoringLevel.L3),
    MechanismPositions.reefPosition(ScoringLevel.L4),
    MechanismPositions.coralStationPosition(),
    MechanismPositions.clearAlgaeLow1Position(),
    MechanismPositions.clearAlgaeLow2Position(),
    MechanismPositions.clearAlgaeHigh1Position(),
    MechanismPositions.clearAlgaeHigh2Position(),
    // intermediate between stow and coral station
    new MechanismPosition(
        Units.Inches.of(32), Rotation2d.fromDegrees(-70), Rotation2d.fromDegrees(0)),
    // flip from one side to the other
    new MechanismPosition(
        Units.Inches.of(55), Rotation2d.fromDegrees(80), Rotation2d.fromDegrees(0)),
    // go under the elevator to get to L1/2
    new MechanismPosition(
        Units.Inches.of(0), Rotation2d.fromDegrees(80), Rotation2d.fromDegrees(0)),
    // intermediate
    new MechanismPosition(
        Units.Inches.of(32), Rotation2d.fromDegrees(80), Rotation2d.fromDegrees(0)),
  };
  // the indices that the corresponding safePosition can safely get to without contacting anything
  private static int[][] connections = {
    {10},
    {2, 12},
    {1},
    {4, 11},
    {3, 11},
    {10, 11},
    {7, 1, 12},
    {6, 2},
    {9, 11},
    {8, 11},
    {0, 5, 11, 13},
    {3, 4, 6, 7, 8, 9, 10, 12, 13},
    {1, 13, 11},
    {10, 12, 11},
  };

  // generates a path with waypoints from one mechanismPos to another
  private static class MechanismPath {
    Node[] intermediatePositions;
    MechanismPosition endPosition;
    int currentIndex = 0;

    private static class Node {
      Node parentNode;
      MechanismPosition position;
      double distance;
      int positionIndex;

      Node(Node parentNode, int positionIndex, double distance) {
        this.parentNode = parentNode;
        this.positionIndex = positionIndex;
        this.position = safePositions[positionIndex];
        this.distance = distance;
      }

      @Override
      public boolean equals(Object o) {
        if (o == null) {
          return false;
        }
        Node n = (Node) o;
        return n.positionIndex == this.positionIndex;
      }
    }

    // returns all nodes the nodes safePos connects to
    Node[] getConnectedNodes(Node n) {
      int[] connectedIndecies = connections[n.positionIndex];
      Node[] connectedNodes = new Node[connectedIndecies.length];
      for (int i = 0; i < connectedIndecies.length; i++) {
        connectedNodes[i] = new Node(n, connectedIndecies[i], 0);
      }
      return connectedNodes;
    }

    // gets the "Distance" from one MechanismPosition to another
    double getDistance(MechanismPosition m1, MechanismPosition m2) {
      return Math.abs(m1.elevatorHeight().in(Units.Inches) - m2.elevatorHeight().in(Units.Inches))
          + Math.abs(m1.armAngle().getDegrees() - m2.armAngle().getDegrees()) * (0.5);
      // the pivot should not have much effect on the "distance" between mech poses
      // + Math.abs(m1.intakeAngle().getDegrees() - m2.intakeAngle().getDegrees());
    }

    // the entire path creation is done when it is initialised, no need to call other methods
    MechanismPath(MechanismPosition startPosition, MechanismPosition endPosition) {
      this.endPosition = endPosition;
      int closestStartIndex = -1;
      double closestDistance = 10000000;
      // find the closest end and start positions
      for (int i = 0; i < safePositions.length; i++) {
        double distance = getDistance(startPosition, safePositions[i]);
        if (distance < closestDistance) {
          closestStartIndex = i;
          closestDistance = distance;
        }
      }
      int closestEndIndex = -1;
      closestDistance = 10000000;
      for (int i = 0; i < safePositions.length; i++) {
        double distance = getDistance(endPosition, safePositions[i]);
        if (distance < closestDistance) {
          closestEndIndex = i;
          closestDistance = distance;
        }
      }
      // pathfinding using a Dijkstra like algorithem
      ArrayList<Node> newPoses = new ArrayList<>();
      newPoses.add(new Node(null, closestStartIndex, 0));
      ArrayList<Node> allPoses = new ArrayList<>();
      do {
        // find the newPose closest to the start
        int lowestDistIndex = -1;
        double lowestDist = 10000000;
        for (int i = 0; i < newPoses.size(); i++) {
          double distance = newPoses.get(i).distance;
          if (distance < lowestDist) {
            lowestDistIndex = i;
            lowestDist = distance;
          }
        }
        Node currentNode = newPoses.get(lowestDistIndex);
        // we found the end
        if (currentNode.equals(new Node(null, closestEndIndex, 0))) {
          // go through the parents untill we find the start node
          ArrayList<Node> pathNodes = new ArrayList<>();
          Node lastNode = currentNode;
          do {
            pathNodes.add(lastNode);
            lastNode = lastNode.parentNode;
          } while (!(lastNode == null));
          Node[] intposes = new Node[pathNodes.size()];
          // reverse the arrayList and send it to an array
          for (int i = 0; i < pathNodes.size(); i++) {
            intposes[pathNodes.size() - 1 - i] = pathNodes.get(i);
          }
          intermediatePositions = intposes;
          break;
        }
        allPoses.add(currentNode);
        newPoses.remove(currentNode);
        // loop over the connected poses
        Node[] connectedNodes = getConnectedNodes(currentNode);
        for (int a = 0; a < connectedNodes.length; a++) {
          // dont check our parent node
          if (connectedNodes[a].equals(currentNode.parentNode)
              || allPoses.contains(connectedNodes[a])) {
            continue;
          }
          // allready checked
          if (allPoses.contains(connectedNodes[a])) {
            double newDistance =
                currentNode.distance
                    + getDistance(currentNode.position, connectedNodes[a].position)
                    + 10;
            // faster route
            if (newDistance < connectedNodes[a].distance) {
              connectedNodes[a].distance = newDistance;
              connectedNodes[a].parentNode = currentNode;
              newPoses.add(connectedNodes[a]);
              allPoses.remove(connectedNodes[a]);
            }
            // new
          } else {
            connectedNodes[a].distance =
                currentNode.distance
                    + getDistance(currentNode.position, connectedNodes[a].position);
            connectedNodes[a].parentNode = currentNode;
            newPoses.add(connectedNodes[a]);
          }
        }
      } while (newPoses.size() != 0);
      // it should never exit with this condition
      // it should break after finding the end
      if (intermediatePositions == null) {
        // if this triggers it means that a node has no inputs or outputs
        System.err.println("--------------NO PATH FOUND--------------");
      }
    }

    MechanismPosition getNextPosition() {
      MechanismPosition nextPosition = null;
      if (isAtEnd()) {
        nextPosition = endPosition;
      } else {
        nextPosition = intermediatePositions[currentIndex].position;
        currentIndex++;
      }

      return nextPosition;
    }

    boolean isAtEnd() {
      return currentIndex == intermediatePositions.length;
    }
  }

  private static Command goToPositionParallel(
      Elevator elevator,
      Arm arm,
      Intake intake,
      Supplier<MechanismPosition> position,
      boolean ignoreSafety) {
    var cmd =
        new Command() {

          boolean elevatorInPosition = false;
          boolean armInPosition = false;
          boolean pivotInPosition = false;
          MechanismPosition targetPosition = position.get();
          MechanismPosition currentTargetPosition;

          MechanismPath path;

          Trigger shouldEnd =
              new Trigger(
                      () ->
                          elevatorInPosition && armInPosition && pivotInPosition && path.isAtEnd())
                  .debounce(0.2);

          @Override
          public void initialize() {
            path =
                new MechanismPath(
                    new MechanismPosition(
                        elevator.getHeight(), arm.getAngle(), intake.getPivotAngle()),
                    targetPosition);
            if (path.intermediatePositions == null) {
              shouldEnd = new Trigger(() -> true);
              return;
            }
            currentTargetPosition = path.getNextPosition();
          }

          boolean runningElevator;
          boolean runningArm;
          boolean runningPivot;

          @Override
          public void execute() {
            if (path.intermediatePositions == null) {
              return;
            }
            if (elevator.isAtTarget() && arm.isAtTargetAngle() && intake.isPivotAtTargetAngle()) {
              currentTargetPosition = path.getNextPosition();
            }

            elevator.setHeight(currentTargetPosition.elevatorHeight());
            arm.setAngle(currentTargetPosition.armAngle());
            intake.setPivotAngle(currentTargetPosition.intakeAngle());

            elevatorInPosition = elevator.isAtTarget();
            armInPosition = arm.isAtTargetAngle();
            pivotInPosition = intake.isPivotAtTargetAngle();

            log_AtPathEnd.info(path.isAtEnd());

            log_PathIndex.info(path.currentIndex);
            log_SafePathIndex.info(
                path.intermediatePositions[Math.max(path.currentIndex - 1, 0)].positionIndex);

            log_runningArm.info(runningArm);
            log_runningElevator.info(runningElevator);
            log_runningPivot.info(runningPivot);

            log_ElevatorInPosition.info(elevatorInPosition);
            log_ArmInPosition.info(armInPosition);
            log_PivotInPosition.info(pivotInPosition);
          }

          @Override
          public boolean isFinished() {
            return shouldEnd.getAsBoolean();
          }
        };

    cmd.addRequirements(elevator, arm, intake);
    cmd.setName("MechanismAction");
    return cmd;
  }
}
