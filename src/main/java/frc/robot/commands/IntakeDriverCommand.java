package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeDriverCommand extends Command{
    
    private final Intake intake;

    public IntakeDriverCommand (Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setTargetRPM(Intake.MAX_RPM);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTargetRPM(0);
    }
}