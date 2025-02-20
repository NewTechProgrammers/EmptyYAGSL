package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

import frc.robot.subsystems.Elevator;

public class ElevatorToPositionCommand extends Command  {
    private final Elevator elevator;
    private final double setpoint;
    private static final double TOLERANCE = 0.05;

    public ElevatorToPositionCommand(Elevator elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        elevator.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        if (setpoint > elevator.getPosition() && elevator.isTopLimitSwitchPressed()) {
            elevator.stop();
        }
        if (setpoint < elevator.getPosition() && elevator.isBottomLimitSwitchPressed()) {
            elevator.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getPosition() - setpoint) < TOLERANCE;
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
