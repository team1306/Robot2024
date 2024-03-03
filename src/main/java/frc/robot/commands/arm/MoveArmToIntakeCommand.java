package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ControlMode;
import frc.robot.subsystems.Arm.Setpoint;

public class MoveArmToIntakeCommand extends MoveArmToSetpointCommand {
    private BooleanSupplier notePresentFlag;

    public MoveArmToIntakeCommand(Arm arm, BooleanSupplier notePresentFlag, BooleanSupplier interruptionFlag) {
        super(arm, Setpoint.INTAKE, interruptionFlag);
        this.notePresentFlag = notePresentFlag;
    }

    @Override
    public void initialize() {
        arm.setControlMode(ControlMode.AUTOMATIC);
    }

    @Override
    public void execute() {
        arm.setTargetAngle(Rotation2d.fromDegrees(notePresentFlag.getAsBoolean() ?
            Setpoint.DOWN.pos :
            Setpoint.INTAKE.pos
        ));
    }
    
}
