package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

import frc.robot.subsystems.Elevator;

public class RaiseElevatorCommand extends Command {
    private final Elevator elevator;

    public RaiseElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {       
    }

    @Override
    public void execute() {
        elevator.runUp();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(elevator);
    }
    
}
