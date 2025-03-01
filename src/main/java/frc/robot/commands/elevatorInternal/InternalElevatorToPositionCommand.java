package frc.robot.commands.elevatorInternal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

import frc.robot.subsystems.InternalElevator;

public class InternalElevatorToPositionCommand extends Command  {
    private final InternalElevator internalElevator;
    private final double setpoint;
    private static final double TOLERANCE = 0.05;

    public InternalElevatorToPositionCommand(InternalElevator internalElevator, double setpoint) {
        this.internalElevator = internalElevator;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        internalElevator.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        if (setpoint > internalElevator.getPosition() && internalElevator.isTopLimitSwitchPressed()) {
            internalElevator.stop();
        }
        if (setpoint < internalElevator.getPosition() && internalElevator.isBottomLimitSwitchPressed()) {
            internalElevator.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(internalElevator.getPosition() - setpoint) < TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        internalElevator.stop();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(internalElevator);
    }
    
}
