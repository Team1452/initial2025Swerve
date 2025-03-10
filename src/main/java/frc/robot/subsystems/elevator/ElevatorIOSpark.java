package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.SparkUtil;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax m_one;
  private final SparkMax m_two;
  private final SparkMaxConfig m_oneConfig;
  private final SparkMaxConfig m_twoConfig;
  private final SparkClosedLoopController m_oneController;
  private final DigitalInput elevatorlimitSwtich;

  public ElevatorIOSpark() {
    elevatorlimitSwtich = new DigitalInput(1);

    m_one = new SparkMax(ElevatorConstants.kmotorOnePort, MotorType.kBrushless);
    m_two = new SparkMax(ElevatorConstants.kmotorTwoPort, MotorType.kBrushless);
    m_oneConfig = new SparkMaxConfig();
    m_twoConfig = new SparkMaxConfig();
    m_oneConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .maxOutput(0.45) // Limit speed
        .minOutput(-0.45) // Limit speed
        .positionWrappingEnabled(false)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pidf(
            ElevatorConstants.kElevatorGains[0],
            ElevatorConstants.kElevatorGains[1],
            ElevatorConstants.kElevatorGains[2],
            ElevatorConstants.kElevatorGains[3]);
    m_twoConfig.apply(m_oneConfig).follow(m_one, ElevatorConstants.motorsInverted);
    SparkUtil.tryUntilOk(
        m_one,
        5,
        () ->
            m_one.configure(
                m_oneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        m_two,
        5,
        () ->
            m_two.configure(
                m_twoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(m_one, 20, () -> m_one.getEncoder().setPosition(0));
    m_oneController = m_one.getClosedLoopController();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorlimitSwtich = elevatorlimitSwtich.get();
    inputs.height = m_one.getEncoder().getPosition();
  }

  @Override
  public void setHeight(double setpoint) {
    m_oneController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void resetEncoder() {
    SparkUtil.tryUntilOk(m_one, 20, () -> m_one.getEncoder().setPosition(0));
  }
}
