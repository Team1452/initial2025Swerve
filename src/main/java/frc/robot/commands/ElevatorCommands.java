package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
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
        Commands.runOnce(intake::rotateOutIntake, intake)
            .onlyIf(
                () -> ElevatorConstants.kElevatorHeights[tier] <= ElevatorConstants.intakeHeight)
            .onlyWhile(() -> !intake.getIntakeOpen()));
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
                      + 10); // Move the elevator to above the handoff height.
            },
            elevator),
        Commands.waitUntil(
            () ->
                MathUtil.isNear(
                    0.75, elevator.getShoulderAngle(), 0.005)), // Wait until the arm is down,
        Commands.runOnce(() -> intake.spinSucker(-0.1), intake), // Spit out.
        Commands.waitSeconds(0.02),
        Commands.runOnce(intake::stopSucker, intake), // Stop the sucker.
        goToTier(0, elevator), // Then move down to the coral to intake it.
        Commands.waitSeconds(0.5),
        Commands.runOnce(
            () -> elevator.setRHeight(ElevatorConstants.kElevatorHeights[0] + 7), elevator),
        Commands.waitUntil(() -> elevator.getHeight() > ElevatorConstants.kElevatorHeights[0] + 4),
        Commands.runOnce(() -> elevator.setRAngle(0.25)));
  }
}
