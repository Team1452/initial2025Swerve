package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public final class IntakeConstants {
  public static final int IntakeID = 51; // CAN ID for the intake rotator NEEDS TO BE ADJUSTED
  public static final int RollerID = 52; // CAN ID for the intake roller NEEDS TO BE ADJUSTED
  public static final double INTAKE_ROTATION_SPEED = 0.5; // Adjust speed as needed
  public static final double INTAKE_SUCK_SPEED = 0.5; // Adjust speed as needed
  public static final Angle INTAKE_OPEN_ANGLE = Rotations.of(1.5); // NEEDS TO BE ADJUSTED
  public static final double openClosedVarianceThrehhold =
      0.05; // Tolerance for the intake open and closed state. Adjust if needed.
}
