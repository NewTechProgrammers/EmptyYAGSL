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

public class Elevator extends SubsystemBase {
    private final SparkMax elevatorMotor = new SparkMax(MechanismConstants.kElevatorSparkMaxPort, MotorType.kBrushless);
    private final SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController elevatorMotorPIDController = elevatorMotor.getClosedLoopController();

    DigitalInput topLimitSwitch = new DigitalInput(DigitalInputConstants.kTopElevatorLimitSwitchPort);
    DigitalInput bottomLimitSwitch = new DigitalInput(DigitalInputConstants.kBottomElevatorLimitSwitchPort);

    public Elevator() {
        elevatorMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        elevatorMotorConfig.encoder
                .positionConversionFactor(MechanismConstants.kElevatorConversionFactor)
                .velocityConversionFactor(2.0);
        elevatorMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(MechanismConstants.kPElevator, MechanismConstants.kIElevator, MechanismConstants.kDElevator);

        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorMotor.getEncoder().setPosition(0);

        SmartDashboard.putBoolean("Elevator", false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("topLimitSwitch", isTopLimitSwitchPressed());
        SmartDashboard.putBoolean("bottomLimitSwitch", isBottomLimitSwitchPressed());
        SmartDashboard.putNumber("ELEVATOR ENCODER", getPosition());
    }

    public boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    public boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    public void runUp() {
        if (isTopLimitSwitchPressed()) {
            elevatorMotor.set(0);
        } else {
            elevatorMotor.set(getExponentialLimitedSpeed(MechanismConstants.kMaxElevatorSpeed, getPosition()));
        }
    }

    public void runDown() {
        if (isBottomLimitSwitchPressed()) {
            elevatorMotor.set(0);
        } else {
            elevatorMotor.set(getExponentialLimitedSpeed(-MechanismConstants.kMaxElevatorSpeed, getPosition()));
        }
    }

    public void stop() {
        elevatorMotor.set(0);
    }

    public double getExponentialLimitedSpeed(double desiredSpeed, double currentPosition) {
        double distanceToLimit = Math.min(currentPosition - MechanismConstants.kMinElevatorPosition, MechanismConstants.kMaxElevatorPosition - currentPosition);

        double scale = Math.exp(-MechanismConstants.lambda * (MechanismConstants.kElevatorfullSpeedStartPosition - distanceToLimit));

        scale = Math.max(0, Math.min(1, scale));

        double limitedSpeed = MechanismConstants.kMaxElevatorSpeed * scale * Math.signum(desiredSpeed);
        if (Math.abs(limitedSpeed) < 0.15 && limitedSpeed != 0) {
            limitedSpeed = Math.signum(limitedSpeed) * 0.15;
        }

        return limitedSpeed;
    }

    public double getPosition() {
        return elevatorMotor.getEncoder().getPosition();
    }

    public void setSetpoint(double position) {
        if (position > getPosition() && isTopLimitSwitchPressed() || isTopLimitSwitchPressed()) {
            return;
        }
        if (position < getPosition() && isBottomLimitSwitchPressed() || isTopLimitSwitchPressed()) {
            return;
        }
        elevatorMotorPIDController.setReference(position, ControlType.kPosition);

    }
}
