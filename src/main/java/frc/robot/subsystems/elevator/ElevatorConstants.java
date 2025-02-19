package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.Measure;

public class ElevatorConstants {

  public static final int kmotorOnePort = 8;
  public static final int kmotorTwoPort = 9;
  public static final int kshoulderPort = 57;
  public static final boolean motorsInverted = true;
  public static final double[] kElevatorHeights = {1, 10, 20, 30}; // Handoff height, tier 1, 2, 3.
  public static final double coralPosition = 3; // UPDATE
  public static final double[] kElevatorGains = {
    0.3, // P
    0, // I
    0, // D
    0 // F
  };
  public static final double kShoulderOffset = 0;
  public static final double shoulderLength = 0.5; //Meters, needs to be updated
}
