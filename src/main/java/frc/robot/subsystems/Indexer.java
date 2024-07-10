package frc.robot.subsystems;

import frc.robot.subsystems.utils.NeoGroupSubsystem;
import frc.robot.util.MotorUtil;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends NeoGroupSubsystem{
    private final DigitalInput indexerSensor = new DigitalInput(INDEXER_SENSOR_DIO_PORT);

    private double targetSpeed = 0;

    private boolean notePresent;
    

    public Indexer(){
        super(new NeoData(MotorUtil.initSparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless, IdleMode.kCoast), false));
        super.relativeSpeed = 0;
    }

    @Override
    public void periodic(){
        super.relativeSpeed = targetSpeed;
        notePresent = indexerSensor.get();

        SmartDashboard.putNumber("Indexer Target Speed", targetSpeed);
        SmartDashboard.putBoolean("Indexer Note Present", notePresent);

        super.periodic();
    }

    public boolean notePresent(){
        return notePresent;
    }

    public double getTargetSpeed(){
        return targetSpeed;
    }

    public void setTargetSpeed(double targetSpeed){
        this.targetSpeed = MotorUtil.clampPercent(targetSpeed);
    }
}
