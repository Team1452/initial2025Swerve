package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public final class IntakeConstants {
  public static final int IntakeID = 51; // CAN ID for the intake rotator NEEDS TO BE ADJUSTED
  public static final int RollerID = 52; // CAN ID for the intake roller NEEDS TO BE ADJUSTED
  public static final double INTAKE_ROTATION_SPEED = 0.5; // Adjust speed as needed
  public static final double INTAKE_SUCK_SPEED = 0.5; // Adjust speed as needed
  public static final Angle INTAKE_CLOSED_ANGLE = Rotations.of(0.2);
  public static final Angle INTAKE_OPEN_ANGLE = Rotations.of(1.5); // NEEDS TO BE ADJUSTED
  public static final double suckerSpikeThreshhold = 3; //MUST BE TESTED AND REPLACED
  public static final boolean reversedRotator = false;
  public static final boolean reversedSucker = false;
  public static final double[] kIntakeGains = {
    0.5, //P
    0, //I
    0, //D
    0 //F
  };

}
