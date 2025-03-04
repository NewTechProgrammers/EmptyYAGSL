// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.general.levels.L0Command;
import frc.robot.commands.general.levels.L2Command;
import frc.robot.commands.general.levels.L3Command;
import frc.robot.commands.general.levels.L4Command;
import frc.robot.commands.general.modules.elevator.LowerElevatorCommand;
import frc.robot.commands.general.modules.elevator.RaiseElevatorCommand;
import frc.robot.commands.general.modules.elevatorInternal.LowerInternalElevatorCommand;
import frc.robot.commands.general.modules.elevatorInternal.RaiseInternalElevatorCommand;
import frc.robot.commands.general.modules.intake.IntakeTakeCommand;
import frc.robot.commands.general.modules.intake.IntakeTimerCommand;
import frc.robot.commands.general.modules.ballintake.ballIntakeShootCommand;
import frc.robot.commands.general.modules.ballintake.ballIntakeTakeCommand;
import frc.robot.commands.general.modules.lift.LowerLiftCommand;
import frc.robot.commands.general.modules.lift.RaiseLiftCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.InternalElevator;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Intake;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {   

        final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
        final CommandXboxController supportXbox = new CommandXboxController(1);
        final XboxController supportXboxAdditionalController = new XboxController(1);

        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));

        private final Elevator elevator = new Elevator();
        private final InternalElevator internalElevator = new InternalElevator();

        private final Lift lift = new Lift();

        private final Intake intake = new Intake();
        private final BallIntake ballIntake = new BallIntake();

        public static final LEDSubsystem leds = new LEDSubsystem();
        
        DoubleSupplier driverXboxRightXInverted = () -> -new XboxController(OperatorConstants.kDriverControllerPort)
                        .getRightX();
        /*
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1)
                        .withControllerRotationAxis(driverXboxRightXInverted)
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY).headingWhile(true);

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative
         * input stream.
         */
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);
                NamedCommands.registerCommand("test", Commands.print("I EXIST"));
                NamedCommands.registerCommand("Tower L3", new L2Command(elevator, internalElevator));
                NamedCommands.registerCommand("Shoot", new IntakeTimerCommand(intake, 1));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureBindings() {
                Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                driverXbox.rightBumper().whileTrue(driveRobotOrientedAngularVelocity).whileFalse(driveFieldOrientedAnglularVelocity);
                driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
                driverXbox.start().whileTrue(Commands.none());
                driverXbox.back().whileTrue(Commands.none());
                driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                driverXbox.rightBumper().onTrue(Commands.none());

                driverXbox.povLeft().whileTrue(new ballIntakeTakeCommand(ballIntake));
                driverXbox.povRight().whileTrue(new ballIntakeShootCommand(ballIntake));
                //driverXbox.povUp().whileTrue(new MoveLinearServoToMaxPosition(linearServo));
                //driverXbox.povDown().whileTrue(new MoveLinearServoToMinPosition(linearServo));
                
                supportXbox.leftBumper().whileTrue(new LowerElevatorCommand(elevator));
                supportXbox.rightBumper().whileTrue(new RaiseElevatorCommand(elevator));

                supportXbox.leftTrigger().whileTrue(new LowerInternalElevatorCommand(internalElevator, () -> supportXboxAdditionalController.getLeftTriggerAxis()));
                supportXbox.rightTrigger().whileTrue(new RaiseInternalElevatorCommand(internalElevator, () -> supportXboxAdditionalController.getRightTriggerAxis()));


                supportXbox.y().onTrue(new L0Command(elevator, internalElevator));
                supportXbox.b().onTrue(new L2Command(elevator, internalElevator));
                supportXbox.a().onTrue(new L3Command(elevator, internalElevator));
                supportXbox.x().onTrue(new L4Command(elevator, internalElevator));

                supportXbox.povUp().whileTrue(new ballIntakeTakeCommand(ballIntake));
                supportXbox.povLeft().whileTrue(new LowerLiftCommand(lift));
                supportXbox.povDown().whileTrue(new ballIntakeShootCommand(ballIntake));
                supportXbox.povRight().whileTrue(new IntakeTakeCommand(intake));

                leds.setColor(true, false, true);
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return drivebase.getAutonomousCommand("New Auto");
        }
}
