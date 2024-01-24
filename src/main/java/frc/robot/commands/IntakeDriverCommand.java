package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeDriverCommand extends Command{
    private Intake intake;
    private XboxController driver2Controller;
    public IntakeDriverCommand (Intake intake) {
        this.intake = intake;
        this.driver2Controller = new XboxController(1);
        addRequirements(intake);
    }

    
    @Override
    public void execute() {
        if (driver2Controller.getBButton()) {
            intake.setTargetSpeed(Intake.MAX_SPEED);
        } else {
            intake.setTargetSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTargetSpeed(0);
    }
}