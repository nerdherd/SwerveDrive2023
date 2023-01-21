// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.PreloadTaxi;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Imu;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.BadPS4.Button;
import frc.robot.commands.SwerveAutos;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TheGreatBalancingAct;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */// 10.6.87.98:5800
public class RobotContainer {

  public static Arm arm = new Arm();
  public static Claw claw = new Claw();
  public static Vision vision = new Vision();
  public static Drivetrain drive = new Drivetrain(vision);
  public static Limelight objDetectCamera = new Limelight();
  // public static Drivetrain drive = new Drivetrain();
  public static Imu imu = new Imu();// AHRS(SPI.Port.kMXP);

  // private SwerveDrivetrain swerveDrive = new SwerveDrivetrain();

  private final CommandPS4Controller driverController = 
      new CommandPS4Controller(ControllerConstants.kDriverControllerPort);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller operatorController =
      new CommandPS4Controller(ControllerConstants.kOperatorControllerPort);
  
  private final PS4Controller driverControllerButtons = 
    new PS4Controller(ControllerConstants.kDriverControllerPort);
    
  // Two different drivetrain modes
  private RunCommand arcadeRunCommand = new RunCommand(() -> drive.tankDrive(driverController.getLeftY(), driverController.getRightY()), drive);
  private RunCommand visionRunCommand = new RunCommand(() -> drive.arcadeDrive(drive.getApriltagLinear(), drive.getApriltagRotation()), drive);

  // public Command swerveCommand = new RepeatCommand(
  //   new SequentialCommandGroup(
  //       new WaitCommand(5),
  //       new InstantCommand(swerveDrive::resetEncoders)
  //   ));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // swerveDrive.setDefaultCommand(
    //       new SwerveJoystickCommand(swerveDrive, 
    //         () -> -driverController.getLeftY(),  
    //         driverController::getLeftX, 
    //         driverController::getRightY, 
    //         driverControllerButtons::getSquareButton));
      
	// Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Note: whileTrue() does not restart the command if it ends while the button is still being held
    // These button bindings are chosen for testing, and may be changed based on driver preference
    operatorController.circle().whileTrue(arm.armToMiddleNodePosition());
    operatorController.triangle().whileTrue(arm.armToTopNodePosition());
    driverController.square().whileTrue(claw.clawOpen());
    driverController.cross().whileTrue(claw.clawClose());
    driverController.L1().whileTrue(drive.shiftHigh());
    driverController.R1().whileTrue(drive.shiftLow());

    // driverController.circle().onFalse(arcadeRunCommand);
    // driverController.circle().whileTrue(visionRunCommand);
    // driverController.circle().onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Vision Mode", false)));
    // driverController.circle().whileTrue(new InstantCommand(() -> SmartDashboard.putBoolean("Vision Mode", true)));
    
    // driverController.circle().onTrue(new InstantCommand(swerveDrive::zeroHeading));
    // driverController.square().onTrue(new InstantCommand(swerveDrive::resetEncoders));
    // SmartDashboard.putData("Turn to 180 degrees", new TurnToAngle(180, swerveDrive));
    // driverController.cross().whileTrue(new TurnToAngle(180, swerveDrive));
    // driverController.triangle().whileTrue(new TheGreatBalancingAct(swerveDrive));
  }

  public void configurePeriodic() {
    arm.movePercentOutput(operatorController.getRightY());
    drive.tankDrive(-driverController.getRightY(), driverController.getLeftY());
    claw.periodic();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PreloadTaxi(drive, claw, arm);
    // return SwerveAutos.testAuto(swerveDrive);
  }
}
