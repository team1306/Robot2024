// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutonomousFactory;
import frc.robot.auto.CloseRingsFromStartMid;
import frc.robot.auto.FarRingsFromStartBottom;
import frc.robot.auto.FarRingsFromStartMid;
import frc.robot.auto.FarRingsFromStartTop;
import frc.robot.auto.MoveOutLeft;
import frc.robot.auto.MoveOutMid;
import frc.robot.auto.MoveOutRight;
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.commands.climber.ClimberDriverCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.intake.IntakeDriverCommand;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.commands.intake.ToggleIntakeCommand;
import frc.robot.commands.shooter.ShooterDriveCommand;
import frc.robot.commands.shooter.ShooterPitchControlCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;
import frc.robot.subsystems.vision.SwitchableDriverCam;
import frc.robot.util.Utilities;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;


public class RobotContainer {
  private CommandXboxController controller1 = new CommandXboxController(0); // Creates an XboxController on port 1.
  private CommandXboxController controller2 = new CommandXboxController(1); // Creates an XboxController on port 1.

  private SendableChooser<AutonomousFactory> autoChooser = new SendableChooser<>();

  public DriveTrain driveTrain;
  public Intake intake;
  public Arm arm;

  private Shooter shooter; 
  private Climber climber;
  
  public TeleopDriveCommand teleopDriveCommand;
  public IntakeDriverCommand intakeDriverCommand;
 
  private ShooterDriveCommand shooterDriveCommand;
  private ShooterPitchControlCommand shooterPitchControlCommand;
  private MoveArmCommand moveArmCommand;
  private ClimberDriverCommand climberDriverCommand;
  private ToggleShooterCommand toggleShooterCommand;
  private ToggleIntakeCommand toggleIntakeCommand;

  private SwitchableDriverCam switchableDriverCam;
  private UsbCamera front, back;
  private Command autonomousCommand = new InstantCommand();
  private final BooleanSupplier cancelSetpoint = () -> controller2.getRightY() > 0 || controller2.getRightY() < 0 || controller2.b().getAsBoolean(); // b acts as cancel button
  
  private final DigitalOutput ledRedBlueOutput, notePresentOutput;
  private double beginningAutoWait = 0;

  public RobotContainer() {
    SmartDashboard.putNumber("Beginning Auto Wait", beginningAutoWait);

    front = CameraServer.startAutomaticCapture(0);
    back = CameraServer.startAutomaticCapture(1);

    front.setResolution(320, 240);
    back.setResolution(320, 240);
    front.setFPS(12);
    back.setFPS(12);

    //SWITCH DOUBLE FRONT TO BACK
    switchableDriverCam = new SwitchableDriverCam(CameraServer.getServer(), front, back);
    
    // TODO: Ideally include these below, if we can't it's whatever
    // front.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    // back.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    driveTrain = new DriveTrain(switchableDriverCam);
    intake = new Intake();
    shooter = new Shooter();
    arm = new Arm();
    climber = new Climber();
    shooterDriveCommand = new ShooterDriveCommand(driveTrain);
    shooterPitchControlCommand = new ShooterPitchControlCommand(arm);
    moveArmCommand = new MoveArmCommand(arm, () -> controller2.getRightY());
    toggleIntakeCommand = new ToggleIntakeCommand(intake, controller2.a(), controller2.b());
    intakeDriverCommand = new IntakeDriverCommand(intake, controller2.b());
    climberDriverCommand = new ClimberDriverCommand(climber);
    teleopDriveCommand = new TeleopDriveCommand(driveTrain, controller1::getLeftTriggerAxis, controller1::getRightTriggerAxis, () -> -controller1.getLeftX());
    toggleShooterCommand = new ToggleShooterCommand(() -> Shooter.PEAK_OUTPUT, arm.getCurrentAngle()::getDegrees, shooter);

    climber.setDefaultCommand(climberDriverCommand);
    arm.setDefaultCommand(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.INTAKE));
    configureBindings();
    
    autoChooser.setDefaultOption("move out mid", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutMid(driveTrain, shooter, arm, intake));
    autoChooser.addOption("move out left", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutLeft(driveTrain, shooter, arm, intake));
    autoChooser.addOption("move out right", (NoteDetector unused,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new MoveOutRight(driveTrain, shooter, arm, intake));
    autoChooser.addOption("Close Rings from Start Mid", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new CloseRingsFromStartMid(detector, intake, shooter, arm));
    autoChooser.addOption("Far Rings from Start Bottom (right around stage)", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new FarRingsFromStartBottom(detector, intake, shooter, arm));
    autoChooser.addOption("Far Rings from Start Top (left around stage)", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new FarRingsFromStartMid(detector, intake, shooter, arm));
    autoChooser.addOption("Far Rings from Start Mid (left around stage)", (NoteDetector detector,  DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) -> new FarRingsFromStartTop(detector, intake, shooter, arm));
    autoChooser.addOption("testPath", (NoteDetector a,  DriveTrain b, Shooter c, Arm d, Intake e) -> new PathPlannerAuto("testPath"));

    SmartDashboard.putData(autoChooser);

    notePresentOutput = new DigitalOutput(8);
    ledRedBlueOutput = new DigitalOutput(9);
    Commands.run(() -> notePresentOutput.set(intake.notePresent()));
    Commands.run(() -> ledRedBlueOutput.set(Utilities.isRedAlliance())).ignoringDisable(true);
  }

  public void autoWaitGetterPeriodic() {
    beginningAutoWait = SmartDashboard.getNumber("Beginning Auto Wait", beginningAutoWait);
  }

  /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
  private void configureBindings() {
    controller1.a()
    .onTrue(new ParallelCommandGroup(shooterDriveCommand, toggleShooterCommand)
    .andThen(new WaitCommand(0.5))
    .andThen(new IntakeIndexCommand(intake))
    .andThen(toggleShooterCommand::stop));
    controller1.b().whileTrue(driveTrain.getSetSpeedMultiplierCommand(0.5));
    controller1.back().onTrue(new InstantCommand(climberDriverCommand::buttonPress));

    controller2.a().onTrue(new InstantCommand(intakeDriverCommand::buttonPress));
    controller2.x().toggleOnTrue(toggleShooterCommand);
    controller2.y().onTrue(shooterPitchControlCommand);
    controller2.rightBumper().onTrue(new InstantCommand(intakeDriverCommand::clearNote));

    controller2.povUp().onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.AMP, cancelSetpoint));
    controller2.povLeft().onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.STAGE_SHOT, cancelSetpoint));
    controller2.povRight().onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.SHOOT_CLOSE, cancelSetpoint));
    controller2.povDown().onTrue(new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.INTAKE, cancelSetpoint));

    controller2.rightStick().onTrue(moveArmCommand);
    controller2.back().toggleOnTrue(toggleIntakeCommand.andThen(new InstantCommand(intakeDriverCommand::reset)));
  }

  public Command getAutonomousCommand() {
    return autonomousCommand;
  }

  public void loadAuto() {
    autonomousCommand = Commands.waitSeconds(beginningAutoWait).andThen(
      autoChooser.getSelected().createAutonomousCommand(new NoteDetector.NoteDetectorPlaceHolder(), driveTrain, shooter, arm, intake)
    );
  }

  public void configureSysIDBindings() {
    final Command temp = driveTrain.getDefaultCommand();
    driveTrain.removeDefaultCommand();
    
    if (temp != null) temp.cancel();
    controller1
        .a()
        .and(controller1.rightBumper())
        .whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    controller1
        .b()
        .and(controller1.rightBumper())
        .whileTrue(driveTrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    controller1
        .x()
        .and(controller1.rightBumper())
        .whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    controller1
        .y()
        .and(controller1.rightBumper())
        .whileTrue(driveTrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  
}
