package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

public class IntakeIOSpark implements IntakeIO {
  // Intake has two motors, one that rotates the intake and one that sucks in the coral
  private final SparkMax m_rotator;
  private final SparkMax m_sucker;
  SparkMaxConfig rotatorConfig;
  SparkMaxConfig suckerConfig;
  SparkClosedLoopController rotatorPID;

  public IntakeIOSpark() {
    // Configure and declare the motors
    m_rotator = new SparkMax(IntakeConstants.IntakeID, MotorType.kBrushless);
    m_sucker = new SparkMax(IntakeConstants.RollerID, MotorType.kBrushless);
    rotatorConfig = new SparkMaxConfig();
    suckerConfig = new SparkMaxConfig();
    rotatorConfig
        .inverted(IntakeConstants.reversedRotator)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .pidf(
            IntakeConstants.kIntakeGains[0],
            IntakeConstants.kIntakeGains[1],
            IntakeConstants.kIntakeGains[2],
            IntakeConstants.kIntakeGains[3])
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-0.5, 0.5)
        .positionWrappingEnabled(false);
    suckerConfig.inverted(IntakeConstants.reversedSucker).idleMode(IdleMode.kCoast);
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
    SparkUtil.tryUntilOk(
        m_rotator,
        5,
        () ->
            m_rotator
                .getEncoder()
                .setPosition(
                    IntakeConstants.intakeStartUpAngle)); // Set the encoder to 0 on startup.
    rotatorPID = m_rotator.getClosedLoopController();
  }

  @Override
  public void setRotatorVelocity(double speed) {
    m_rotator.set(speed);
  }

  @Override
  public void setRotatorAngle(double angle) {
    rotatorPID.setReference(angle, ControlType.kPosition);
  }

  @Override
  public void setSuckerVelocity(double speed) {
    m_sucker.set(speed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.suckerCurrent = m_sucker.getOutputCurrent();
    inputs.rotatorCurrent = m_rotator.getOutputCurrent();
    inputs.intakeAngle = m_rotator.getEncoder().getPosition(); // Position in rotations
    inputs.rotatorSpeed = m_rotator.getEncoder().getVelocity();
    inputs.suckerSpeed = m_sucker.getEncoder().getVelocity();
    inputs.intakeOpen =
        inputs.intakeAngle > IntakeConstants.intakeHandOffAngle; // If the intake is open
  }
}
