package frc.robot.commands.general.levels;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.general.modules.elevatorInternal.InternalElevatorToPositionCommand;
import frc.robot.subsystems.InternalElevator;

public class L2Command extends Command {
    private final InternalElevator internalElevator;
    private final Command l2Command;

    public L2Command(InternalElevator internalElevator) {
        this.internalElevator = internalElevator;
        this.l2Command = new InternalElevatorToPositionCommand(this.internalElevator, 120);
    }

    @Override
    public void initialize() {
        l2Command.initialize();
    }

    @Override
    public void execute() {
        l2Command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        l2Command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return l2Command.isFinished();
    }
}
