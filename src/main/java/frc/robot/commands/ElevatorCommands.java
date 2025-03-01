package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {

  public static Command controlElevatorWithController(
      DoubleSupplier inputx,
      DoubleSupplier inputy,
      Elevator
          elevator) { // Control the elevator, this will only alter the RHeight and RAngle, moving
    // of motors is handled in Elevator.
    return Commands.run(
        () -> {
          elevator.adjustRHeight(
              MathUtil.applyDeadband(inputy.getAsDouble(), 0.3)
                  / 3.5); // Divide by 3.5 to adjust stepSize.
          elevator.adjustRAngle(MathUtil.applyDeadband(inputx.getAsDouble(), 0.2) / 5);
        });
  }

  public static Command microAdjustShoulderWithTrigger(
      DoubleSupplier trigger, BooleanSupplier inverse, Elevator elevator) {
    return Commands.run(
        () -> {
          // Trigger's base pos is 0.5, max is 1. So we'll subtract 0.5 and multiply by 2 to get a
          // range of 0 to 1.
          elevator.adjustRAngle(
              (inverse.getAsBoolean() ? 1 : -1)
                  * Math.pow((trigger.getAsDouble() - 0.5) * 2, 3)); // Cube the value.
        },
        elevator);
  }

  public static Command goToTier(int tier, Elevator elevator) {
    return Commands.runOnce(
        () -> {
          elevator.setRHeight(ElevatorConstants.kElevatorHeights[tier]);
        } // Set the height to the height based on a list of tier heights.
        ,
        elevator);
  }

  public static Command goToTier(int tier, Elevator elevator, Intake intake) {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              elevator.setRHeight(ElevatorConstants.kElevatorHeights[tier]);
            } // Set the height to the height based on a list of tier heights.
            ,
            elevator),
        Commands.runOnce(intake::rotateOutIntake, intake).onlyWhile(() -> !intake.getIntakeOpen()));
  }

  public static Command place(Elevator elevator) {
    // Assuming alignment to reef.
    return Commands.runOnce(() -> elevator.setRAngle(ElevatorConstants.scoringAngle), elevator);
  }

  public static Command placeOnTier(int tier, Elevator elevator, Intake intake) {
    // Assuming alignment to reef.
    return Commands.sequence(goToTier(tier, elevator, intake), place(elevator));
  }

  public static Command pickUpCoralFromIntake(Elevator elevator, Intake intake) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              elevator.setRAngle(0.75); // Move the arm down.
              elevator.setRHeight(
                  ElevatorConstants.kElevatorHeights[0]
                      + 7); // Move the elevator to above the handoff height.
            },
            elevator),
        Commands.waitUntil(
            () ->
                MathUtil.isNear(
                    0.75, elevator.getShoulderAngle(), 0.005)), // Wait until the arm is down,
        goToTier(0, elevator), // Then move down to the coral to intake it.
        Commands.waitUntil(
            () ->
                MathUtil.isNear(ElevatorConstants.kElevatorHeights[0], elevator.getHeight(), 0.05)),
        Commands.runOnce(
            () -> elevator.setRHeight(ElevatorConstants.kElevatorHeights[0] + 7), elevator),
        Commands.waitUntil(() -> elevator.getHeight() > ElevatorConstants.kElevatorHeights[0] + 2),
        Commands.runOnce(() -> elevator.setRAngle(0.25)));
  }
}
