// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.DashboardGetter;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    PortForwarder.add(5800, "photonvision.local", 5800);
  }

  @Override
  public void robotPeriodic() {
    DashboardGetter.update();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  
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
