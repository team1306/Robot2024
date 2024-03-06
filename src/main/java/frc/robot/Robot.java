// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.arm.DebugArmCommand;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand, m_armDebugCommand;
  private UsbCamera front, back;
  private RobotContainer m_robotContainer;


  @Override
  public void robotInit() {
    front = new UsbCamera("front", 0);
    front.setResolution(100, 100);
    front.setFPS(8);
    front.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    CameraServer.startAutomaticCapture(front);
    back = new UsbCamera("back", 1);
    back.setResolution(100, 100);
    back.setFPS(8);
    back.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    CameraServer.startAutomaticCapture(back);
    m_robotContainer = new RobotContainer();

    PortForwarder.add(5800, "photonvision.local", 5800);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("Arm Current Angle", m_robotContainer.arm.getCurrentAngle().getDegrees());
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //m_autonomousCommand =

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
    m_robotContainer.moveArmCommand.reset();
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
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}


}
