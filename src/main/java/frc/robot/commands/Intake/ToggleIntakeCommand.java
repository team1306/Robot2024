package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ToggleIntakeCommand extends Command{

    private final Intake intake;
    private final DoubleSupplier speed;
    private boolean finished = false;
    
    public ToggleIntakeCommand (Intake intake, DoubleSupplier speed) {
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void execute () {
       intake.setTargetSpeed(speed.getAsDouble());
       finished = intake.notePresent(); //when note detected, stop intake
    }

    @Override
    public void end (boolean interrupted) {
        intake.setTargetSpeed(0);
    }

    @Override
    public boolean isFinished () {
        return finished;
    }
}