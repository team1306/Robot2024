package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Setpoint;
import frc.robot.util.Utilities;

public class ShooterPitchControlCommand extends InstantCommand{
    private static double a = 1, b = 1, c = 1;

    public ShooterPitchControlCommand(Arm arm){
        super(setArmTargetAngle(arm), arm);
    }

    private static Runnable setArmTargetAngle(Arm arm){
        return () -> {
            double speakerDistance = Utilities.getSpeakerDistance(Utilities.getRobotPos());
            
            //Theta must be in terms of degrees
            double theta = a * Math.pow(speakerDistance, 2) + b * speakerDistance + c;
            // Set the target angle of the arm
            new MoveArmToSetpointCommand(arm, new Setpoint.Custom(theta)).schedule();
        };   
    }
}
