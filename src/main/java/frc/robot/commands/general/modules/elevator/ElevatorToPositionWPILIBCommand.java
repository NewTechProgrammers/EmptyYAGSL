package frc.robot.commands.general.modules.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

import frc.robot.subsystems.Elevator;

public class ElevatorToPositionWPILIBCommand extends Command  {
    private final Elevator elevator;
    private final double setpoint;

    public ElevatorToPositionWPILIBCommand(Elevator elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // elevator.setSetpoint(setpoint);
    }

    @Override
    public void execute() {

        elevator.moveToPositionWPILIB(setpoint, setpoint > this.elevator.getPosition() ? true : false);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(elevator);
    }
    
}
