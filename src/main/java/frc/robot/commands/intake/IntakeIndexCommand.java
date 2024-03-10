package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeIndexCommand extends Command {
    private final Intake intake;
    private Timer timer = new Timer();
    public IntakeIndexCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        timer.restart();
        System.out.println("Intake spinny spinny");
    }

    @Override
    public void execute() {
        intake.setTargetSpeed(1);
    }



    @Override
    public void end(boolean interrupted) {
        intake.setTargetSpeed(0);
        System.out.println("Intake no spinny spinny");
    }

    @Override
    public boolean isFinished() {
        System.out.println(timer.get());
        return timer.hasElapsed(0.5);
    }
}
