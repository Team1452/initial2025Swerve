package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Angle;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.SparkUtil;

public class IntakeIOSpark implements IntakeIO {
  // Intake has two motors, one that rotates the intake and one that sucks in the coral
  private final SparkMax m_rotator;
  private final SparkMax m_sucker;
  SparkMaxConfig rotatorConfig;
  SparkMaxConfig suckerConfig;
  SparkClosedLoopController rotatorController;


  public IntakeIOSpark() {
    // Configure and declare the motors
    m_rotator = new SparkMax(IntakeConstants.IntakeID, MotorType.kBrushless);
    m_sucker =
        new SparkMax(
            IntakeConstants.RollerID,
            MotorType.kBrushless); // The motor that runs the roller and sucks in the coral
    rotatorConfig = new SparkMaxConfig();
    suckerConfig = new SparkMaxConfig();
    
    rotatorConfig
      .inverted(IntakeConstants.reversedRotator)
      .idleMode(IdleMode.kBrake)
      .closedLoop
      .maxOutput(0.3) // Limit speed
      .minOutput(-0.3) // Limit speed
      .positionWrappingEnabled(false)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pidf(
          IntakeConstants.kIntakeGains[0],
          IntakeConstants.kIntakeGains[1],
          IntakeConstants.kIntakeGains[2],
          IntakeConstants.kIntakeGains[3]);
    suckerConfig
      .inverted(IntakeConstants.reversedSucker)
      .idleMode(IdleMode.kBrake);
    SparkUtil.tryUntilOk(
        m_rotator,
        5,
        () ->
            m_rotator.configure(
                rotatorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters)); // Try until the motor is configured
    SparkUtil.tryUntilOk(
        m_sucker,
        5,
        () ->
            m_sucker.configure(
                suckerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    rotatorController = m_rotator.getClosedLoopController();
    
  }

  @Override
  public void setRotatorVelocity(double speed) {
    m_rotator.set(speed);
  }
  @Override
  public void setIntakeAngle(Angle angle) {
    rotatorController.setReference(angle.baseUnitMagnitude(), ControlType.kPosition);
  }

  @Override
  public void setSuckerVelocity(double speed) {
    m_sucker.set(speed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.suckerCurrent = m_sucker.getAppliedOutput();
    inputs.intakeAngle =
        Rotations.of(m_rotator.getEncoder().getPosition()); // Position in rotations
    inputs.suckerSpeed = m_sucker.getEncoder().getVelocity(); // Rotations per minute
  }
}
