package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeDriverCommand extends Command{
    
    private final Intake intake;
    private final BooleanSupplier buttonPressed;

    public IntakeDriverCommand (Intake intake, BooleanSupplier buttonPressed) {
        this.intake = intake;
        this.buttonPressed = buttonPressed;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (buttonPressed.getAsBoolean()) {
            intake.setTargetRPM(Intake.MAX_RPM);
        } else {
            intake.setTargetRPM(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTargetRPM(0);
    }
}