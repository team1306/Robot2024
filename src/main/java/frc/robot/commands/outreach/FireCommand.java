package frc.robot.commands.outreach;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FireCommand extends Command{
    private static final double INTAKE_SPEED = 0.6;
    private final Intake intake;
    private final Shooter shooter;
    private Timer timer = new Timer();

    public FireCommand(Intake intake, Shooter shooter){
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake);
    }

    public void execute(){
        if(shooter.getBottomRPM() > 1000 && intake.notePresent())
            intake.setTargetSpeed(INTAKE_SPEED);
        else{
            cancel();
        }
    }

    public boolean isFinished(){
        return timer.hasElapsed(0.25);
    }

    public void end(boolean interrupted){
        intake.setTargetSpeed(0);
    }
}
