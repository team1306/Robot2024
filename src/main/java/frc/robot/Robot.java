// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.DashboardGetter;
import frc.robot.util.MotorUtil;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;
import frc.robot.util.Dashboard.PutValue;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  @PutValue(key = "push")
  private double value = 0;

  private CANSparkMax motor = MotorUtil.initSparkMax(5, MotorType.kBrushless, IdleMode.kBrake);
  private SparkAnalogSensor encoder = motor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);
  private AnalogInput input = new AnalogInput(0);

  @Override
  public void robotInit() {
    // robotContainer = new RobotContainer();
    // DashboardHelpers.addClassToRefresh(this);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    DashboardGetter.update();
    DashboardHelpers.updateValues();
    SmartDashboard.putNumber("Encoder Pos", input.getValue());
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
