package frc.robot.commands.elevatorInternal;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

import frc.robot.subsystems.InternalElevator;

public class LowerInternalElevatorCommand extends Command {
    private final InternalElevator internalElevator;

    public LowerInternalElevatorCommand(InternalElevator internalElevator) {
        this.internalElevator = internalElevator;
    }

    @Override
    public void initialize() {       
    }

    @Override
    public void execute() {
        internalElevator.runDown();
    }

    @Override
    public void end(boolean interrupted) {
        internalElevator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(internalElevator);
    }
}
