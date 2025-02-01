package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.SparkUtil;

public class IntakeIOSpark implements IntakeIO {
    //Intake has two motors, one that rotates the intake and one that sucks in the coral 
    private final SparkMax m_rotator;
    private final SparkMax m_sucker;
    SparkMaxConfig rotatorConfig;
    SparkMaxConfig suckerConfig;
    boolean reversedRotator = false;
    boolean reversedSucker = false;


    public IntakeIOSpark() {
        //Configure and declare the motors
        m_rotator = new SparkMax(IntakeConstants.IntakeID, MotorType.kBrushless);
        m_sucker = new SparkMax(IntakeConstants.RollerID, MotorType.kBrushless); //The motor that runs the roller and sucks in the coral
        rotatorConfig = new SparkMaxConfig();
        suckerConfig = new SparkMaxConfig();

        rotatorConfig
            .inverted(reversedRotator);
        suckerConfig
            .inverted(reversedSucker);

        SparkUtil.tryUntilOk(m_rotator, 5, () -> m_rotator.configure(rotatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)); //Try until the motor is configured
        SparkUtil.tryUntilOk(m_sucker, 5, () -> m_sucker.configure(suckerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void setRotatorVelocity(double speed) {
        m_rotator.set(speed);
    }
    @Override
    public void setSuckerVelocity(double speed) {
        m_sucker.set(speed);
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeAngle = Rotations.of(m_rotator.getEncoder().getPosition()); //Position in rotations
        inputs.intakeState = inputs.intakeAngle
            .isNear(IntakeConstants.INTAKE_OPEN_ANGLE, 0.05) ? true //If the intake at the opened state, set the intake state to true
            : (inputs.intakeAngle.isNear(Rotations.of(0), 0.05) ? false //If the intake at the closed state, set the intake state to false
            : inputs.intakeState); //If the intake is not at the opened or closed state, then just ignore until the next update. ADJUST THE TOLERANCE IF NEEDED
        inputs.suckerSpeed = m_sucker.getEncoder().getVelocity(); //Rotations per minute
        inputs.suckerRunning = m_sucker.getOutputCurrent() > 0.1; //Check if the sucker is running, ADJUST THIS VALUE IF NEEDED
    }
}