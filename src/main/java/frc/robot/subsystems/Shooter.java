package frc.robot.subsystems;

import frc.robot.subsystems.utils.NeoGroupSubsystem;
import frc.robot.util.MotorUtil;
import static frc.robot.Constants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends NeoGroupSubsystem{
    
    private final CANSparkMax shooterTopMotor = MotorUtil.initSparkMax(SHOOTER_BOTTOM_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast),
                              shooterBottomMotor = MotorUtil.initSparkMax(SHOOTER_TOP_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast);
    private final RelativeEncoder topEncoder, bottomEncoder;
    
    private double targetSpeed = 0;

    public Shooter(){
        super();
        super.addNeo(new NeoData(shooterTopMotor, false));
        super.addNeo(new NeoData(shooterBottomMotor, false));
        
        super.relativeSpeed = 0;

        topEncoder = shooterTopMotor.getEncoder();
        bottomEncoder = shooterBottomMotor.getEncoder();
    }

    @Override
    public void periodic(){
        super.relativeSpeed = targetSpeed;

        SmartDashboard.putNumber("Shooter Target Speed", targetSpeed); 
        super.periodic();
    }

    public double getTargetSpeed(){
        return targetSpeed;
    }

    public void setTargetSpeed(double targetSpeed){
        this.targetSpeed = MotorUtil.clampPercent(targetSpeed);
    }

    /**
     * @return speed of top wheel in RPM
     */
    public double getTopRPM() {
        return topEncoder.getVelocity();
    }

    /**
     * @return speed of bottom wheel in RPM
     */
    public double getBottomRPM() {
        return bottomEncoder.getVelocity();
    }
    
}
