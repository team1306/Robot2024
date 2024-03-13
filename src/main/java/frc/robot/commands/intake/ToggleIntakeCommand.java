package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ToggleIntakeCommand extends Command{

    private final Intake intake;
    private final BooleanSupplier on, reverse;

    public ToggleIntakeCommand(Intake intake, BooleanSupplier on, BooleanSupplier reverse){
        this.intake = intake;
        this.on = on;
        this.reverse = reverse;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (on.getAsBoolean()) {
            intake.setTargetSpeed(IntakeDriverCommand.INTAKE_SPEED);
        } else if (reverse.getAsBoolean()) {
            intake.setTargetSpeed(-1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTargetSpeed(0);
    }
}
