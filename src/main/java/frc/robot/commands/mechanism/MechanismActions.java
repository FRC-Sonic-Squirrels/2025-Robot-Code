package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.commands.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import java.nio.ByteBuffer;
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

  private static final LoggerEntry.Struct<Pose2d> log_EstimatedElevatorPosision =
      logGroup.buildStruct(Pose2d.class, "EstimatedElevatorPosision");
  private static final LoggerEntry.Struct<Pose2d> log_EstimatedArmPosision =
      logGroup.buildStruct(Pose2d.class, "EstimatedArmPosision");
  private static final LoggerEntry.Struct<Pose2d> log_EstimatedPivotPosision =
      logGroup.buildStruct(Pose2d.class, "EstimatedPivotPosision");
  private static final LoggerEntry.StructArray<Collision> log_Collisions =
      logGroup.buildStructArray(Collision.class, "Collisions");

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

  private static class collisionSquare {
    Pose2d location;
    double xScale;
    double yScale;

    collisionSquare(Pose2d location, double xScale, double yScale) {
      this.location = location;
      this.xScale = xScale / 2;
      this.yScale = yScale / 2;
    }

    boolean isInside(Translation2d point) {
      return Math.abs(point.getX()) < this.xScale && Math.abs(point.getY()) < this.yScale;
    }

    Translation2d tranformPoint(double x, double y, Pose2d otherlocation) {
      return new Translation2d(x, y)
          .rotateBy(otherlocation.getRotation())
          .plus(otherlocation.getTranslation())
          .minus(this.location.getTranslation())
          .rotateBy(this.location.getRotation().unaryMinus());
    }

    // for easier typey typey
    boolean checkCollision(collisionSquare other) {
      return checkCollision(other, true);
    }

    boolean checkCollision(collisionSquare other, boolean recurse) {
      // subtract the location of this from other
      Transform2d newPos = new Transform2d(Pose2d.kZero, other.location.relativeTo(this.location));
      // check all four corners of the other against this
      Translation2d cornerpp = tranformPoint(other.xScale, other.yScale, other.location);
      if (isInside(cornerpp)) {
        return true;
      }
      Translation2d cornerpn = tranformPoint(other.xScale, -other.yScale, other.location);
      if (isInside(cornerpn)) {
        return true;
      }
      Translation2d cornernp = tranformPoint(-other.xScale, other.yScale, other.location);
      if (isInside(cornernp)) {
        return true;
      }
      Translation2d cornernn = tranformPoint(-other.xScale, -other.yScale, other.location);
      if (isInside(cornernn)) {
        return true;
      }

      if (recurse) {
        // have the other check against us if it did not tell us to
        return other.checkCollision(this, false);
      } else {
        // return false if none intersect
        return false;
      }
    }
  }

  public static record Collision(int collider, int colliding, int bad)
      implements StructSerializable {

    /** Pose3d struct for serialization. */
    public static final CollisionStruct struct = new CollisionStruct();
  }

  public static class CollisionStruct implements Struct<Collision> {
    @Override
    public Class<Collision> getTypeClass() {
      return Collision.class;
    }

    @Override
    public String getTypeName() {
      return "Collision";
    }

    @Override
    public int getSize() {
      return 9;
    }

    @Override
    public String getSchema() {
      return "double collider;double colliding;int bad";
    }

    @Override
    public Collision unpack(ByteBuffer bb) {
      var collider = (int) bb.getDouble();
      var colliding = (int) bb.getDouble();
      var bad = (int) bb.getInt();
      return new Collision(collider, colliding, bad);
    }

    @Override
    public void pack(ByteBuffer bb, Collision value) {
      bb.putDouble(value.collider);
      bb.putDouble(value.colliding);
      bb.putInt(value.bad);
    }

    @Override
    public boolean isImmutable() {
      return true;
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
          // the first 3 are the elevator, arm, and pivot
          // 0,0 is the bottom of the elevator
          // 1 unit is 1 inch
          // positive x is from the elevator to the intake
          // positive y is up
          // TODO: get actual values and poses for colldiers
          collisionSquare[] colliders = {
            new collisionSquare(new Pose2d(new Translation2d(0, -1), Rotation2d.kZero), 2.0, 1.0),
            new collisionSquare(new Pose2d(new Translation2d(0, -1), Rotation2d.kZero), 7.0, 9.75),
            new collisionSquare(new Pose2d(new Translation2d(0, -1), Rotation2d.kZero), 13.0, 5.0),
            new collisionSquare(new Pose2d(new Translation2d(8, -1), Rotation2d.kZero), 18, 1),
            new collisionSquare(new Pose2d(new Translation2d(17, 18), Rotation2d.kZero), 5, 5)
          };
          double SAFETY_BARRIER = 1.0;
          Pose2d intakePos = new Pose2d(new Translation2d(19, 6.2), Rotation2d.kZero);
          collisionSquare[] safeColliders = new collisionSquare[colliders.length];

          @Override
          public void initialize() {
            for (int i = 0; i < colliders.length; i++) {
              safeColliders[i] =
                  new collisionSquare(
                      colliders[i].location,
                      colliders[i].xScale + SAFETY_BARRIER,
                      colliders[i].yScale + SAFETY_BARRIER);
            }
          }

          @Override
          public void execute() {
            // update colliders for the moving parts
            // elevator
            colliders[0].location =
                new Pose2d(
                    new Translation2d(0.0, elevator.getHeight().in(Units.Inches)),
                    colliders[0].location.getRotation());
            safeColliders[0].location = colliders[0].location;
            // arm, elevator plus arm stuff
            colliders[1].location =
                colliders[0].location.plus(
                    new Transform2d(
                        new Translation2d(ArmConstants.ARM_LENGTH.in(Units.Inches), 0)
                            .rotateBy(arm.getAngle()),
                        arm.getAngle()));
            safeColliders[1].location = colliders[1].location;
            // pivot
            colliders[2].location =
                intakePos.plus(
                    new Transform2d(
                        new Translation2d(PivotConstants.PIVOT_LENGTH.in(Units.Inches), 0)
                            .rotateBy(intake.getPivotAngle()),
                        intake.getPivotAngle()));
            safeColliders[2].location = colliders[2].location;

            ArrayList<Collision> collisions = new ArrayList<>(1);
            // check the three moving parts against all other parts
            for (int i = 0; i < 3; i++) {
              for (int c = 0; c < colliders.length; c++) {
                if (c == 0) { // dont check if the other is myself
                  continue;
                }
                // collision between the two colliders
                if (colliders[i].checkCollision(colliders[i])) {
                  // they hit eachother
                  collisions.add(new Collision(i, c, 1));
                } else if (safeColliders[i].checkCollision(safeColliders[i])) {
                  // oh no they are getting very close
                  collisions.add(new Collision(i, c, 0));
                }
              }
            }
            // check elevator
            // 0 means it is fine
            // 1 means it is close to another collider
            // 2 means it is hitting another collider
            int elevatorMovePriority = 0;

            boolean runningElevator = true;
            for (Collision c : collisions) { // check against all collisions
              if (c.collider != 0) {
                continue;
              }
              if (c.bad == 1) {
                // if the thing we are hitting is lower than us
                if (colliders[0].location.getY() < colliders[c.colliding].location.getY()) {
                  // move down
                  elevator.setHeight(Units.Inches.of(0));
                } else {
                  // otherwise move up
                  elevator.setHeight(ElevatorConstants.MAX_HEIGHT);
                }
                elevatorMovePriority = 2;
                runningElevator = false;

                break;
                // close to hitting
              } else if (c.bad == 0) {
                // same logic as above
                if (colliders[0].location.getY() < colliders[c.colliding].location.getY()) {
                  elevator.setHeight(Units.Inches.of(0));
                } else {
                  elevator.setHeight(ElevatorConstants.MAX_HEIGHT);
                }
                elevatorMovePriority = 1;
                runningElevator = false;

                break;
              }
            }
            // same collision logic for arm and pivot
            // check arm
            boolean runningArm = true;
            for (Collision c : collisions) { // check against all collisions
              if (c.collider != 1) {
                continue;
              }
              if (colliders[1].checkCollision(colliders[c.colliding])) {
                // only move the elevator if it is ok
                if (elevatorMovePriority < 2) {
                  if (colliders[1].location.getY() < colliders[c.colliding].location.getY()) {
                    elevator.setHeight(Units.Inches.of(0));
                  } else {
                    elevator.setHeight(ElevatorConstants.MAX_HEIGHT);
                  }
                  runningElevator = false;
                }
                // if the thing we are colliding with is to our left
                // TODO: make sure this math is correct
                if (colliders[c.colliding]
                        .location
                        .relativeTo(colliders[1].location.relativeTo(colliders[0].location))
                        .getX()
                    < 0.0) {
                  // move right
                  arm.setAngle(ArmConstants.MAX_ARM_ANGLE);
                } else {
                  // otherwise move left
                  arm.setAngle(ArmConstants.MIN_ARM_ANGLE);
                }
                runningArm = false;

                break;
              } else if (safeColliders[1].checkCollision(safeColliders[c.colliding])) {
                // if we are close dont check the safe colliders
                if (Math.abs(arm.getAngle().getDegrees() - targetPosition.armAngle().getDegrees())
                    < 5) {
                  break;
                }
                // only move the elevator if it is ok
                if (elevatorMovePriority < 1) {
                  if (colliders[1].location.getY() < colliders[c.colliding].location.getY()) {
                    elevator.setHeight(Units.Inches.of(0));
                  } else {
                    elevator.setHeight(ElevatorConstants.MAX_HEIGHT);
                  }
                  runningElevator = false;
                }
                // same logic as above
                if (colliders[c.colliding]
                        .location
                        .relativeTo(colliders[1].location.relativeTo(colliders[0].location))
                        .getX()
                    < 0.0) {
                  arm.setAngle(ArmConstants.MAX_ARM_ANGLE);
                } else {
                  arm.setAngle(ArmConstants.MIN_ARM_ANGLE);
                }
                runningArm = false;

                break;
              }
            }
            // check pivot
            boolean runningPivot = true;
            for (Collision c : collisions) { // check against all collisions
              if (c.collider != 2) {
                continue;
              }
              if (colliders[2].checkCollision(colliders[c.colliding])) {

                // same logic as above
                if (colliders[c.colliding]
                        .location
                        .relativeTo(colliders[2].location.relativeTo(intakePos))
                        .getX()
                    < 0.0) {
                  intake.setPivotAngle(PivotConstants.MAX_PIVOT_ANGLE);
                } else {
                  intake.setPivotAngle(PivotConstants.MIN_PIVOT_ANGLE);
                }
                runningPivot = false;

                break;
              } else if (safeColliders[2].checkCollision(safeColliders[c.colliding])) {
                // same logic as above
                if (colliders[c.colliding]
                        .location
                        .relativeTo(colliders[2].location.relativeTo(intakePos))
                        .getX()
                    < 0.0) {
                  intake.setPivotAngle(PivotConstants.MAX_PIVOT_ANGLE);
                } else {
                  intake.setPivotAngle(PivotConstants.MIN_PIVOT_ANGLE);
                }
                runningPivot = false;

                break;
              }
            }
            if (runningElevator) {
              elevator.setHeight(targetPosition.elevatorHeight());
            }
            if (runningArm) {
              arm.setAngle(targetPosition.armAngle());
            }
            if (runningPivot) {
              intake.setPivotAngle(targetPosition.intakeAngle());
            }

            elevatorInPosition = elevator.isAtTarget();
            armInPosition = arm.isAtTargetAngle();
            pivotInPosition = intake.isPivotAtTargetAngle();

            log_runningArm.info(runningArm);
            log_runningElevator.info(runningElevator);
            log_runningPivot.info(runningPivot);

            log_ElevatorInPosition.info(elevatorInPosition);
            log_ArmInPosition.info(armInPosition);
            log_PivotInPosition.info(pivotInPosition);

            log_EstimatedElevatorPosision.info(colliders[0].location);
            log_EstimatedArmPosision.info(colliders[1].location);
            log_EstimatedPivotPosision.info(colliders[2].location);

            log_Collisions.info(collisions.toArray(new Collision[0]));
          }

          @Override
          public boolean isFinished() {
            return elevatorInPosition && armInPosition && pivotInPosition;
          }
        };

    cmd.addRequirements(elevator, arm, intake);
    cmd.setName("MechanismAction");
    return cmd;
  }
}
