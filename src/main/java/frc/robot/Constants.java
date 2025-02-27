// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
    public static final double MAX_SPEED = Units.feetToMeters(4.5);
  
  public static final class DrivebaseConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class MechanismConstants {
    public static final int    kElevatorSparkMaxPort = 18;
    public static final double kMaxElevatorSpeed = 0.4;
    public static final double kMinElevatorSpeed = 0.1;
    public static final double kElevatorConversionFactor = ( 8 / 3 ) * 0.9975641;

    public static final double kPElevator = 0.008;
    public static final double kIElevator = 0.0;
    public static final double kDElevator = 0.001;
    

    public static final int    kInternalElevatorSparkMaxPort = 19;
    public static final double kMaxInternalElevatorSpeed = 0.3;
    public static final double kMinInternalElevatorSpeed = 0.02;
    public static final double kInternalElevatorConversionFactor = ( 47.75 * Math.PI ) / 5;

    public static final double kPInternalElevator = 0.0095;
    public static final double kIInternalElevator = 0.0;
    public static final double kDInternalElevator = 0.001;


    public static final int    kLiftSparkMaxPort = 16;
    public static final double kMaxLiftSpeed = 0.2;
    public static final double kMinLiftSpeed = 0.1;
    public static final double kLiftConversionFactor = 1.0;

    public static final int    kIntakeSparkMaxPort = 20;
    public static final double kMaxIntakeSpeed = 0.10;
    public static final double kMaxIntakeShootSpeed = 0.30;
    public static final double kIntakeConversionFactor = 1.0;
  }

  public static final class DigitalInputConstants {
    public static final int    kTopElevatorLimitSwitchPort = 9;
    public static final int    kBottomElevatorLimitSwitchPort = 8;
    public static final int    kLiftLimitSwitchPort = 7;
    public static final int    kIntakeLimitSwitchPort = 6;
    public static final int    kTopInternalElevatorLimitSwitchPort = 5;
    public static final int    kBottomInternalElevatorLimitSwitchPort = 4;
  }

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
