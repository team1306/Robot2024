package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command{

    private static final double TIME_BEFORE_SHOT_MS = 1000;
    private static final double TIME_AFTER_SHOT_MS = 1000;

    private final Shooter shooter;

    private boolean finished = false;
    private final Intake intake;
    private final Timer timer = new Timer();
    private boolean done = false;

    public ShootCommand(Shooter shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;

        this.addRequirements(shooter, intake);
    }
    @Override
    public void execute(){
        shooter.setTargetRPM(Shooter.PEAK_RPM);
        if (timer.hasElapsed(TIME_BEFORE_SHOT_MS/1000) && !done){
            intake.setTargetRPM(Intake.MAX_RPM);
            done = true;
            timer.reset();
        }
        else if(timer.hasElapsed(TIME_AFTER_SHOT_MS/1000) && done){
            intake.setTargetRPM(0);
            finished = true;
        }
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}
