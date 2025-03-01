package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputConstants;
import frc.robot.Constants.MechanismConstants;

public class InternalElevator extends SubsystemBase{
    private final SparkMax internalElevatorMotor = new SparkMax(MechanismConstants.kInternalElevatorSparkMaxPort, MotorType.kBrushless);
    private final SparkMaxConfig internalElevatorMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController internalElevatorMotorPIDController = internalElevatorMotor.getClosedLoopController();
    private final double maxSpeed = MechanismConstants.kMaxInternalElevatorSpeed;
    private final double minSpeed = MechanismConstants.kMinInternalElevatorSpeed;

    DigitalInput topLimitSwitch = new DigitalInput(DigitalInputConstants.kTopInternalElevatorLimitSwitchPort);
    DigitalInput bottomLimitSwitch = new DigitalInput(DigitalInputConstants.kBottomInternalElevatorLimitSwitchPort);
    
    public InternalElevator() {
        internalElevatorMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        internalElevatorMotorConfig.encoder
                .positionConversionFactor(MechanismConstants.kInternalElevatorConversionFactor)
                .velocityConversionFactor(2.0);
        internalElevatorMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(MechanismConstants.kPInternalElevator, MechanismConstants.kIInternalElevator, MechanismConstants.kDInternalElevator);

        internalElevatorMotor.configure(internalElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        internalElevatorMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("INTERNAL ELEVATOR ENCODER", getPosition());
    }

    public boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    public boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    public void runUp(double triggerValue) {
        if (isTopLimitSwitchPressed()) {
            stop();
        } else {
            double speed = minSpeed + (maxSpeed - minSpeed) * ((triggerValue - 0.5) / 0.5);
            internalElevatorMotor.set(speed);
        }

    }

    public void runDown(double triggerValue) {
        if (isBottomLimitSwitchPressed()) {
            stop();
        } else {
            double speed = minSpeed + (maxSpeed - minSpeed) * ((triggerValue - 0.5) / 0.5);
            internalElevatorMotor.set(-speed);
        }
    }

    public void stop() {
        internalElevatorMotor.set(MechanismConstants.kMinInternalElevatorSpeed);
    }

    public double getPosition() {
        return internalElevatorMotor.getEncoder().getPosition();
    }

    public void setSetpoint(double position) {
        if (position > getPosition() && isTopLimitSwitchPressed() || isTopLimitSwitchPressed()) {
            return;
        }
        if (position < getPosition() && isBottomLimitSwitchPressed() || isTopLimitSwitchPressed()) {
            return;
        }
        internalElevatorMotorPIDController.setReference(position, ControlType.kPosition);

    }

}

