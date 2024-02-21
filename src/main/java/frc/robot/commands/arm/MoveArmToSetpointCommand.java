package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ControlMode;

public class MoveArmToSetpointCommand extends Command {
    private final Arm.Setpoint setpoint;
    private final Arm arm;
    public MoveArmToSetpointCommand(Arm arm, Arm.Setpoint setpoint) {
        this.setpoint = setpoint;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setControlMode(ControlMode.AUTOMATIC);
        arm.setTargetAngle(Rotation2d.fromDegrees(setpoint.pos));
    }

    @Override
    public void end(boolean interrupted) {
        // STUB
    }

    public static void moveArmToSetpoint(Arm arm, Arm.Setpoint setpoint) {

    }
    
}
