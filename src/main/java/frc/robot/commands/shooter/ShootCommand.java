package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Wait;

public class ShootCommand extends Command{

    private static final double TIME_BEFORE_SHOT_MS = 1000;
    private static final double TIME_AFTER_SHOT_MS = 1000;

    private final Shooter shooter;
    private final Wait wait;
    private Wait afterWait;

    private boolean finished = false;
    private final Intake intake;

    public ShootCommand(Shooter shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
        this.wait = new Wait(TIME_BEFORE_SHOT_MS);
        
        this.addRequirements(shooter, intake);
    }
    @Override
    public void execute(){
        shooter.setTargetRPM(Shooter.PEAK_RPM);
        if (wait.update()){
            intake.setTargetRPM(Intake.MAX_RPM);
            afterWait = new Wait(TIME_AFTER_SHOT_MS);
        }
        if(afterWait != null && afterWait.update()){
            intake.setTargetRPM(0);
            finished = true;
        }
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}
