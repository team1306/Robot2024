package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command{

    private static double TIME_BEFORE_SHOT_MS = 1000;
    private static double TIME_AFTER_SHOT_MS = 1000;

    private final Shooter shooter;

    private boolean finished = false;
    private final Intake intake;
    private final Timer timer = new Timer();
    private boolean done = false;

    public ShootCommand(Shooter shooter, Intake intake){
        this.shooter = shooter;
        this.intake = intake;
        this.addRequirements(shooter, intake);

        SmartDashboard.putNumber("Time Before Shot (MS)", TIME_BEFORE_SHOT_MS);
        SmartDashboard.putNumber("Time After Shot (MS)", TIME_AFTER_SHOT_MS);
    }
    @Override
    public void execute(){
        TIME_BEFORE_SHOT_MS = SmartDashboard.getNumber("Time Before Shot (MS)", 1000);
        TIME_AFTER_SHOT_MS = SmartDashboard.getNumber("Time After Shot (MS)", 1000);

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
        shooter.setTargetRPM(0);
        return finished;
    }
}
