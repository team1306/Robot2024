package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class NoteIndexingCommand extends Command{

    public static double TIME_BEFORE_SHOT_MS = 1000;
    public static double TIME_AFTER_SHOT_MS = 1000;

    private boolean finished = false;
    private final Intake intake;
    private final Timer timer = new Timer();
    private boolean done = false;

    public NoteIndexingCommand(Intake intake){
        this.intake = intake;
        this.addRequirements(intake);

        SmartDashboard.putNumber("Time Before Shot (MS)", TIME_BEFORE_SHOT_MS);
        SmartDashboard.putNumber("Time After Shot (MS)", TIME_AFTER_SHOT_MS);
    }

    @Override
    public void initialize(){
        timer.restart();
    }
    @Override
    public void execute(){
        TIME_BEFORE_SHOT_MS = SmartDashboard.getNumber("Time Before Shot (MS)", 1000);
        TIME_AFTER_SHOT_MS = SmartDashboard.getNumber("Time After Shot (MS)", 1000);

        if (timer.hasElapsed(TIME_BEFORE_SHOT_MS/1000) && !done){
            intake.setTargetSpeed(1);
            done = true;
            timer.restart();
        }
        else if(timer.hasElapsed(TIME_AFTER_SHOT_MS/1000) && done){
            intake.setTargetSpeed(0);
            finished = true;
        }
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}
