// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.PathPlannerAutos;
import frc.robot.commands.SquareTest;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.SwerveJoystickCommand.DodgeDirection;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.NavX;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.swerve.SwerveDrivetrain.SwerveModuleType;
import frc.robot.subsystems.vision.primalWallnut.PrimalPotatoMine;
import frc.robot.subsystems.vision.primalWallnut.PrimalSunflower;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public Gyro imu = new NavX();
  // public Gyro imu = new Pigeon(60);
  public SwerveDrivetrain swerveDrive;

  //private PrimalSunflower ps;

  private final CommandPS4Controller driverController = new CommandPS4Controller(
      ControllerConstants.kDriverControllerPort);
  private final PS4Controller badPS5 = driverController.getHID();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller operatorController = new CommandPS4Controller(
      ControllerConstants.kOperatorControllerPort);
  private final PS4Controller badPS4 = operatorController.getHID();
  // private final Joystick joystick = new Joystick(2);

  private final LOG_LEVEL loggingLevel = LOG_LEVEL.ALL;

  private SendableChooser<Supplier<CommandBase>> autoChooser = new SendableChooser<Supplier<CommandBase>>();

  // private PrimalSunflower backSunflower = new PrimalSunflower(VisionConstants.kLimelightBackName);
  private PrimalSunflower frontSunflower = new PrimalSunflower(VisionConstants.kLimelightFrontName, 0.6); //0.6 is threshold for consistent ATag detection

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      // Pass in "sunflowers" in reverse order of priority (most important last)
      swerveDrive = new SwerveDrivetrain(imu, SwerveModuleType.CANCODER, frontSunflower);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }

    initAutoChoosers();

    // Configure the trigger bindings
    configureBindings();

    DriverStation.reportWarning("Initalization complete", false);
  }

  public void initDefaultCommands() {
    swerveDrive.setDefaultCommand(
      new SwerveJoystickCommand(
        swerveDrive,
        () -> -driverController.getLeftY(), // Horizontal translation
        driverController::getLeftX, // Vertical Translation
        // () -> 0.0, // debug
        driverController::getRightX, // Rotation
        badPS5::getSquareButton, // Field oriented
        badPS5::getL2Button, // Towing
        // Dodge
        // () -> {return badPS5.getL1Button() || badPS5.getR1Button();},
        () -> false,
        // Dodging
        () -> {
          // if (badPS5.getL1Button()) {
          //   return DodgeDirection.LEFT;
          // } 
          // if (badPS5.getR1Button()) {
          //   return DodgeDirection.RIGHT;
          // }
          return DodgeDirection.NONE;
        },
        badPS5::getR2Button, // Precision/"Sniper Button"
        () -> badPS5.getR1Button() || badPS5.getL1Button(), // Turn to angle
        () -> { // Turn To angle Direction
          if (badPS5.getR1Button()) {
            return 180.0;
          } else {
            return 0.0;
          }
        }
      ));
  }

  private void configureBindings() {
    // Note: whileTrue() does not restart the command if it ends while the button is
    // still being held
    // These button bindings are chosen for testing, and may be changed based on
    driverController.share().onTrue(Commands.runOnce(imu::zeroHeading));
    driverController.options().onTrue(Commands.runOnce(swerveDrive::resetEncoders));

    //driverController.L2().onTrue(Commands.runOnce(() -> ps.usePlantFood()));
    //SmartDashboard.putData("Get Closest Grid" , Commands.runOnce(() -> ps.getClosestZombieTile()));
  }

  private void initAutoChoosers() {
    // Remember to load the pathplanner paths here
    final String[] paths = {
      "TestPath", "ChargeAroundLEFT", "TaxiRIGHT", "TaxiLEFT", "TestSquare", "Test Line", "TestSquare3"
    };
    
    PathPlannerAutos.init(swerveDrive);

    for (String path : paths) {
      PathPlannerAutos.initPath(path);
      PathPlannerAutos.initPathGroup(path);
    }

    autoChooser.addOption("Do Nothing", Commands::none);
    autoChooser.addOption("Path Planner Test Auto", () -> PathPlannerAutos.pathplannerAuto("TestPath", swerveDrive));
    autoChooser.addOption("Path Planner Charge Around LEFT", () -> PathPlannerAutos.pathplannerAuto("ChargeAroundLEFT", swerveDrive));
    autoChooser.addOption("Path Planner TaxiRIGHT", () -> PathPlannerAutos.pathplannerAuto("TaxiRIGHT", swerveDrive));
    autoChooser.addOption("Path Planner TaxiLEFT", () -> PathPlannerAutos.pathplannerAuto("TaxiLEFT", swerveDrive));
    autoChooser.addOption("Path Planner TestSquare", () -> PathPlannerAutos.pathplannerAuto("TestSquare", swerveDrive));
    autoChooser.addOption("Path Planner Test3", () -> PathPlannerAutos.pathplannerAuto("Test Line", swerveDrive));
    autoChooser.addOption("Path Planner TestSquare3", () -> PathPlannerAutos.pathplannerAuto("TestSquare3", swerveDrive));
    autoChooser.addOption("Path Planner TestSquare4", () -> new SquareTest(PathPlannerAutos.autoBuilder));

    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
  }
  
  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(loggingLevel);
    //ps.initShuffleboard(loggingLevel);
  }

  public void reportAllToSmartDashboard() {
    imu.reportToSmartDashboard(loggingLevel);
    swerveDrive.reportToSmartDashboard(loggingLevel);
    swerveDrive.reportModulesToSmartDashboard(loggingLevel);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command currentAuto = autoChooser.getSelected().get();

    swerveDrive.setDriveMode(DRIVE_MODE.AUTONOMOUS);
    return currentAuto;
  }
}
