package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.util.NeoGroupSubsystem;
import frc.robot.util.MotorUtil;

import static frc.robot.Constants.*;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class TestShooterIntake extends NeoGroupSubsystem {
    private final XboxController controller = new XboxController(0);

    public TestShooterIntake() {
        super(
            new Pair<>(MotorUtil.initSparkMax(INTAKE_MOTOR_ID, kBrushless, IdleMode.kCoast), false),
            new Pair<>(MotorUtil.initSparkMax(SHOOTER_BOTTOM_MOTOR_ID, kBrushless, IdleMode.kCoast), false),
            new Pair<>(MotorUtil.initSparkMax(SHOOTER_TOP_MOTOR_ID, kBrushless, IdleMode.kCoast), true)
        );
    }

    @Override
    public double getPowerOutput() {
        return Math.pow(controller.getLeftY(), 2);
    }
    
}
