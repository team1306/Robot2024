// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.DashboardGetter;
import frc.robot.util.MotorUtil;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.PutValue;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private CANSparkMax motor1 = MotorUtil.initSparkMax(5, MotorType.kBrushless, IdleMode.kCoast);
    private CANSparkMax motor2 = MotorUtil.initSparkMax(6, MotorType.kBrushless, IdleMode.kCoast);
      private CANSparkMax motor3 = MotorUtil.initSparkMax(7, MotorType.kBrushless, IdleMode.kCoast);
        private CANSparkMax motor4 = MotorUtil.initSparkMax(8, MotorType.kBrushless, IdleMode.kCoast);



  
  private SparkAnalogSensor encoder1 = motor1.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
    private SparkAnalogSensor encoder2 = motor2.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
      private SparkAnalogSensor encoder3 = motor3.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
        private SparkAnalogSensor encoder4 = motor4.getAnalog(SparkAnalogSensor.Mode.kAbsolute);




  @PutValue(key = "Encoder Output 1")
  private double value1 = 0;
    @PutValue(key = "Encoder Output 2")
  private double value2 = 0;
    @PutValue(key = "Encoder Output 3")
  private double value3 = 0;
    @PutValue(key = "Encoder Output 4")
  private double value4 = 0;

  @Override
  public void robotInit() {
    // robotContainer = new RobotContainer();
    DashboardHelpers.addClassToRefresh(this);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    DashboardGetter.update();
    DashboardHelpers.updateValues();
    value1 = encoder1.getPosition();
        value2 = encoder2.getPosition();
          value3 = encoder3.getPosition();
             value4 = encoder4.getPosition();



  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    DashboardHelpers.updateValues();
  
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
  }
}
