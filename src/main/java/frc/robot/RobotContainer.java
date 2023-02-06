// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ApproachCombined;
import frc.robot.commands.DriveToTarget;
import frc.robot.subsystems.AirCompressor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ConeRunner;
import frc.robot.subsystems.TankDrivetrain;
import frc.robot.subsystems.Imu;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.commands.SwerveAutos;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TankAutos;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.DriveToTarget.PipelineType;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */// 10.6.87.98:5800
public class RobotContainer {

  public static Arm arm = new Arm();
  public static Claw claw = new Claw();
  public static Imu imu = new Imu();
  public static Limelight objDetectCamera = new Limelight();
  public static ConeRunner coneRunner = new ConeRunner();
  public static final boolean IsSwerveDrive = true;
  public static TankDrivetrain tankDrive;
  public static SwerveDrivetrain swerveDrive;
  public AirCompressor airCompressor = new AirCompressor();

  private PipelineType obj = PipelineType.ATAG;

  private final CommandPS4Controller driverController = new CommandPS4Controller(
      ControllerConstants.kDriverControllerPort);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller operatorController = new CommandPS4Controller(
      ControllerConstants.kOperatorControllerPort);

  private final PS4Controller driverControllerButtons = new PS4Controller(ControllerConstants.kDriverControllerPort);

  SendableChooser<CommandBase> autoChooser = new SendableChooser<CommandBase>();

  public double swerveTargetAngle = 180;

  // Two different drivetrain modes
  private RunCommand arcadeRunCommand;
  private RunCommand visionRunCommand;
  
  // Two different drivetrain modes
  // private RunCommand arcadeRunCommand = new RunCommand(() -> drive.tankDrive(driverController.getLeftY(), driverController.getRightY()), drive);
  // private RunCommand visionRunCommand = new RunCommand(() -> drive.arcadeDrive(drive.getApriltagLinear(), drive.getApriltagRotation()), drive);
  // private RunCommand visionRunCommandArea = new RunCommand(() -> drive.arcadeDrive(drive.getAprilTagAreaLinear(), drive.getApriltagRotation()), drive);

  public Command swerveCommand;

  public SwerveJoystickCommand swerveJoystickCommand;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    if (IsSwerveDrive) {
      swerveDrive = new SwerveDrivetrain(imu);

      swerveCommand = new RepeatCommand(
          new SequentialCommandGroup(
              new WaitCommand(5),
              new InstantCommand(swerveDrive::resetEncoders)));

      autoChooser.setDefaultOption("Hard Carry", SwerveAutos.hardCarryAuto(swerveDrive));
      autoChooser.addOption("Hard Carry", SwerveAutos.hardCarryAuto(swerveDrive));
      autoChooser.addOption("Vending Machine", SwerveAutos.vendingMachine(swerveDrive));
      autoChooser.addOption("Test auto", SwerveAutos.twoPieceChargeAuto(swerveDrive, arm, claw));
      SmartDashboard.putData(autoChooser);
      SmartDashboard.putData("Encoder reset", Commands.runOnce(swerveDrive::resetEncoders, swerveDrive));

    } else {
      tankDrive = new TankDrivetrain();

      // visionRunCommand = new RunCommand(
      //     () -> tankDrive.arcadeDrive(tankDrive.getApriltagLinear(), tankDrive.getApriltagRotation()), tankDrive);
    }


    // Configure the trigger bindings
    configureBindings();
  }

  public void initDefaultCommands() {
    arm.setDefaultCommand(
      new RunCommand(
        () -> arm.moveArmJoystick(operatorController.getLeftY()), 
        arm
      ));
    // arm.setDefaultCommand(arm.moveArmJoystickCommand(operatorController::getLeftY));

    if (IsSwerveDrive) {
      swerveDrive.setDefaultCommand(
        new SwerveJoystickCommand(
          swerveDrive,
          () -> -driverController.getLeftY(),
          driverController::getLeftX,
          // () -> 0.0,
          driverController::getRightY,
          // () -> true,
          driverControllerButtons::getSquareButton,
          () -> false,
          // driverControllerButtons::getTriangleButton,
          driverControllerButtons::getCrossButton
        ));
    } else {
      tankDrive.setDefaultCommand(
        new RunCommand(
          () -> tankDrive.drive(
            -driverController.getLeftY(), 
            -driverController.getRightY()
          ), tankDrive));
    }


  }

  private void configureBindings() {
    // Note: whileTrue() does not restart the command if it ends while the button is
    // still being held
    // These button bindings are chosen for testing, and may be changed based on
    // driver preference
    if (!IsSwerveDrive) {
      driverController.L1().whileTrue(tankDrive.shiftHigh()); // TODO: use it for swerve too? inch-drive
      driverController.R1().whileTrue(tankDrive.shiftLow());
    }
    operatorController.triangle().whileTrue(arm.armExtend());
    operatorController.square().whileTrue(arm.armStow());
    operatorController.circle().onTrue(claw.clawOpen());
    operatorController.cross().onTrue(claw.clawClose());

    if (IsSwerveDrive) {
      driverController.circle().onTrue(new InstantCommand(imu::zeroHeading));
      driverController.cross().onTrue(new InstantCommand(swerveDrive::resetEncoders));
      
      driverController.R1().whileTrue(new TurnToAngle(180, swerveDrive));
      driverController.L1().whileTrue(new TurnToAngle(0, swerveDrive));
      // driverController.triangle().whileTrue(new DriveToTarget(swerveDrive, objDetectCamera, 4, obj))
      //                  .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));

      driverController.triangle().whileTrue(new ApproachCombined(swerveDrive, objDetectCamera, 4, obj))
      .onFalse(Commands.runOnce(swerveDrive::stopModules, swerveDrive));
    }
  }
  
  public void initShuffleboard() {
    if (!IsSwerveDrive) {
      tankDrive.initShuffleboard();
    }
    // autoChooser = new SendableChooser<CommandBase>();
    // autoChooser.setDefaultOption("Hard Carry Auto",
    // TankAutos.HardCarryAuto(drive, claw, arm));
    
    // autoChooser.addOption("Diet Coke Auto",
    // TankAutos.DietCokeAuto(drive, claw, arm));
  }

  public void reportAllToSmartDashboard() {
    imu.reportToSmartDashboard();
    claw.reportToSmartDashboard();
    arm.reportToSmartDashboard();
    objDetectCamera.reportToSmartDashboard();
    coneRunner.reportToSmartDashboard();
    if (IsSwerveDrive) {
      swerveDrive.reportToSmartDashboard();
      swerveDrive.reportModulesToSmartDashboard();
    } else {
      tankDrive.reportToSmartDashboard();
    }
    airCompressor.reportToSmartDashboard();
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    if (IsSwerveDrive)
      return SwerveAutos.twoPieceChargeAuto(swerveDrive, arm, claw);
    else
      return TankAutos.HardCarryAuto(tankDrive, claw, arm);
  }

  public void autonomousInit() {
    if (!IsSwerveDrive) { // TODO: Move resets to robot init? 
      tankDrive.resetEncoders();
      // drive.setEncoder(drive.meterToTicks(0.381));
      imu.zeroHeading();
    }

    // if (IsSwerveDrive) {
    //   swerveDrive.resetEncoders();
    // }
  }
}
