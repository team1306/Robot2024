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
import frc.robot.util.DashboardGetter;
import frc.robot.util.Utilities;

import static frc.robot.util.Utilities.removeAndCancelDefaultCommand;

public class Robot extends TimedRobot {
  private Command autonomousCommand, armDebugCommand;
  private RobotContainer robotContainer;
  
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    
    PortForwarder.add(5800, "photonvision.local", 5800);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    DashboardGetter.update();
    SmartDashboard.putNumber("Speaker Distance", Utilities.getSpeakerDistance(robotContainer.driveTrain.getPose()));
  }
  

  @Override
  public void disabledInit() {
    SmartDashboard.putBoolean("Load Auto", false);
  }

  @Override
  public void disabledPeriodic() {
    if (SmartDashboard.getBoolean("Load Auto", false)) {
      robotContainer.loadAuto();
      SmartDashboard.putBoolean("Load Auto", false);
    }
    SmartDashboard.putString("Auto Name", robotContainer.getAutonomousCommand().getName());

    final Rotation2d armAngle = robotContainer.arm.getCurrentAngle();
    SmartDashboard.putNumber("Arm Current Angle", armAngle.getDegrees());
    robotContainer.arm.setTargetAngle(armAngle);

    robotContainer.autoWaitGetterPeriodic();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    removeAndCancelDefaultCommand(robotContainer.driveTrain);
    removeAndCancelDefaultCommand(robotContainer.intake);

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
    if (armDebugCommand != null) {
      armDebugCommand.cancel();
    }
    robotContainer.intake.setDefaultCommand(robotContainer.intakeDriverCommand);
    robotContainer.bindDrivetrainTeleop();
    // robotContainer.climberDriverCommand.resetState();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    armDebugCommand = new DebugArmCommand(robotContainer.arm);
    armDebugCommand.schedule();
    robotContainer.intake.setDefaultCommand(robotContainer.intakeDriverCommand);
    robotContainer.bindDrivetrainTestMode();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}


}
