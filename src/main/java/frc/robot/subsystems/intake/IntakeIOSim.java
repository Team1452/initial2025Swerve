package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

public class IntakeIOSim implements IntakeIO {

  // Simulated state variables
  private double rotatorPosition = 0.0; // Position in rotations
  private double commandedRotatorVelocity = 0.0; // Rotator velocity command (rotations per second)

  private double commandedSuckerVelocity =
      0.0; // Sucker velocity command (unitless, arbitrary scale)
  private double simulatedSuckerSpeed = 0.0; // Simulated sucker speed (RPM)

  // Fixed simulation timestep (seconds)
  private final double dt = 0.02;

  @Override
  public void setRotatorVelocity(double speed) {
    // In simulation simply store the commanded speed
    commandedRotatorVelocity = speed;
  }

  @Override
  public void setSuckerVelocity(double speed) {
    // In simulation simply store the commanded speed
    commandedSuckerVelocity = speed;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Update simulation state using a fixed timestep dt
    // Integrate rotator position
    rotatorPosition += commandedRotatorVelocity * dt;

    // Simulate sucker speed
    // Arbitrary conversion: scale the commanded sucker velocity to represent RPM
    simulatedSuckerSpeed = commandedSuckerVelocity * 100.0;

    // Update sensor inputs
    inputs.intakeAngle = Rotations.of(rotatorPosition);

    // Determine intake state based on angle; use tolerance from constants
    if (inputs.intakeAngle.isNear(
        IntakeConstants.INTAKE_OPEN_ANGLE, IntakeConstants.openClosedVarianceThrehhold)) {
      inputs.intakeState = true;
    } else if (inputs.intakeAngle.isNear(
        Rotations.of(0), IntakeConstants.openClosedVarianceThrehhold)) {
      inputs.intakeState = false;
    }
    // Otherwise, leave intakeState unchanged

    // Update sucker sensor values
    inputs.suckerSpeed = simulatedSuckerSpeed;
    inputs.suckerRunning = Math.abs(commandedSuckerVelocity) > 0.05;
  }
}
