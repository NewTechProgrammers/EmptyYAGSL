package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Lift extends SubsystemBase {
    private final SparkMax liftMotor = new SparkMax(MechanismConstants.kLiftSparkMaxPort, MotorType.kBrushless);
    private final SparkMaxConfig liftMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController liftMotorPIDController = liftMotor.getClosedLoopController();

    DigitalInput liftLimitSwitch = new DigitalInput(7);

    public Lift() {
        liftMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        liftMotorConfig.encoder
                .positionConversionFactor(MechanismConstants.kLiftConversionFactor)
                .velocityConversionFactor(1.0);

        liftMotor.configure(liftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        liftMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("liftLimitSwitch", isLiftLimitSwitchPressed());
        SmartDashboard.putNumber("LIFT ENCODER", getPosition());
    }

    public boolean isLiftLimitSwitchPressed() {
        return !liftLimitSwitch.get();
    }

    public void runUp() {
        if (isLiftLimitSwitchPressed()) {
            liftMotor.set(0);
        } else {
            liftMotor.set(MechanismConstants.kMaxLiftSpeed);
        }
    }

    public void runDown() {
        if (isLiftLimitSwitchPressed()) {
            liftMotor.set(0);
        } else {
            liftMotor.set(-MechanismConstants.kMaxElevatorSpeed);
        }
    }

    public void stop() {
        liftMotor.set(0);
    }

    public double getPosition() {
        return liftMotor.getEncoder().getPosition();
    }
}
