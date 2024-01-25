package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeDriverCommand extends Command{
    private Intake intake;
    public IntakeDriverCommand (Intake intake) {
        this.intake = intake;
    }

    public void Intake (double RPM) {
        intake.setTargetRPM(RPM);
    }
}
