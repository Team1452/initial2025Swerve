package frc.robot.subsystems.shoulder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shoulder extends SubsystemBase {
  // Hardware interface for the elevator.
  private final ShoulderIO io;
  private double shoulderRAngle = 0.275;
  // Inputs from the elevator hardware.
  private final ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
  /**
   * @param io The interfce)
   */
  public Shoulder(ShoulderIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs); // Refresh the inputs.
    io.setShoulderAngle(shoulderRAngle);
    Logger.processInputs("Shoulder", inputs);
    Logger.recordOutput("Shoulder/ShoulderRAngle", shoulderRAngle);
  }

  public void setRAngle(double angle) {
    shoulderRAngle = angle;
  }

  public double getAngle() {
    return inputs.shoulderAngle;
  }

  public double getRAngle() {
    return shoulderRAngle;
  }

  public boolean nearRPosition() {
    return MathUtil.isNear(
        shoulderRAngle, inputs.shoulderAngle, 0.008); // about 3 degrees tolerance.
  }

  public void adjustRAngle(double angle) {
    shoulderRAngle += angle;
  }
}
