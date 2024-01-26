// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.MotorUtil;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;


  private final XboxController controller = new XboxController(0);


  CANSparkMax top, bottom, intake;
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    intake = MotorUtil.initSparkMax(INTAKE_MOTOR_ID, kBrushless, IdleMode.kCoast);
    bottom = MotorUtil.initSparkMax(SHOOTER_BOTTOM_MOTOR_ID, kBrushless, IdleMode.kCoast);
    top = MotorUtil.initSparkMax(SHOOTER_TOP_MOTOR_ID, kBrushless, IdleMode.kCoast);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
  }

  @Override
  public void teleopPeriodic() {
    //TEST REMOVE LATER
    double speed = Math.pow(controller.getLeftY(), 2);
    top.set(speed);
    bottom.set(speed);
    intake.set(Math.pow(controller.getRightY(), 2));
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
