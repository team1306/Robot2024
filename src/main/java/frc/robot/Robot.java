// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.arm.DebugArmCommand;
import frc.robot.util.Utilities;

import static frc.robot.util.Utilities.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand, m_armDebugCommand;
  private RobotContainer m_robotContainer;
  
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    
    PortForwarder.add(5800, "photonvision.local", 5800);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Speaker Distance", Utilities.getSpeakerDistance(Utilities.getRobotPos()));
  }
  

  @Override
  public void disabledInit() {
    SmartDashboard.putBoolean("Load Auto", false);
  }

  @Override
  public void disabledPeriodic() {
    if (SmartDashboard.getBoolean("Load Auto", false)) {
      m_robotContainer.loadAuto(); 
      SmartDashboard.putBoolean("Load Auto", false);
    }
    SmartDashboard.putString("Auto Name", m_robotContainer.getAutonomousCommand().getName());

    final Rotation2d armAngle = m_robotContainer.arm.getCurrentAngle();
    SmartDashboard.putNumber("Arm Current Angle", armAngle.getDegrees());
    m_robotContainer.arm.setTargetAngle(armAngle);

    m_robotContainer.autoWaitGetterPeriodic();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    removeAndCancelDefaultCommand(m_robotContainer.driveTrain);
    removeAndCancelDefaultCommand(m_robotContainer.intake);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if (m_armDebugCommand != null) {
      m_armDebugCommand.cancel();
    }
    m_robotContainer.intake.setDefaultCommand(m_robotContainer.intakeDriverCommand);
    m_robotContainer.driveTrain.setDefaultCommand(m_robotContainer.teleopDriveCommand);
    // m_robotContainer.climberDriverCommand.resetState();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_armDebugCommand = new DebugArmCommand(m_robotContainer.arm);
    m_armDebugCommand.schedule();
    m_robotContainer.configureSysIDBindings();
    m_robotContainer.intake.setDefaultCommand(m_robotContainer.intakeDriverCommand);
    m_robotContainer.driveTrain.setDefaultCommand(m_robotContainer.teleopDriveCommand);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}


}
