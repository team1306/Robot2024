package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Setpoint;
import frc.robot.util.Utilities;
@Deprecated
public class ShooterPitchControlCommand extends InstantCommand{
    private static double a = -1.82e-3, b = 0.369, c = 16.6;

    public ShooterPitchControlCommand(Arm arm){
        super(setArmTargetAngle(arm), arm);
    }

    private static Runnable setArmTargetAngle(Arm arm){
        return () -> {
            double speakerDistance = Units.metersToInches(Utilities.getSpeakerDistance(Utilities.getRobotPos()));
            
            //Theta must be in terms of degrees
            double theta = a * Math.pow(speakerDistance, 2) + b * speakerDistance + c;
            theta = MathUtil.clamp(theta, 0, 90);
            // Set the target angle of the arm
            new MoveArmToSetpointCommand(arm, new Setpoint.Custom(theta)).schedule();
        };   
    }
}
