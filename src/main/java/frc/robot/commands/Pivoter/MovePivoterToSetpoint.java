package frc.robot.commands.pivoter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivoter;
import frc.robot.subsystems.utils.PivoterSetpoint;

public class MovePivoterToSetpoint extends Command{

    private final Pivoter pivoter;
    private final PivoterSetpoint setpoint;
    public MovePivoterToSetpoint (Pivoter pivoter, PivoterSetpoint setpoint){
        this.pivoter = pivoter;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize(){
        pivoter.setTargetAngle(setpoint.getAngle());
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
