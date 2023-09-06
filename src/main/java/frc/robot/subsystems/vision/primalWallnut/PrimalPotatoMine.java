package frc.robot.subsystems.vision.primalWallnut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.util.NerdyMath;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PrimalPotatoMine {

    private Limelight limelight;
    private SwerveDrivetrain drivetrain;

    private double goalArea = 4.1;
    private double goalTX = 0;
    private double goalYaw = 0;

    final PIDController PIDArea = new PIDController(0.75, 0, 0.02);
    final PIDController PIDTX = new PIDController(0.05, 0, 0.008);
    final PIDController PIDYaw = new PIDController(0, 0, 0);

    public PrimalPotatoMine(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        try {
            limelight = new Limelight("limelight-high");
            limelight.setLightState(Limelight.LightMode.OFF);
            SmartDashboard.putString("LL Status", "Instantiated");

        } catch (Exception ex) {
            limelight = null;
            SmartDashboard.putString("LL Status", "Error Instantiating");
        }

        limelight.setPipeline(2);
        SmartDashboard.putNumber("Checkpoint 0", 0);


    }
    

    public CommandBase PickupGroundNoArm() {
        SmartDashboard.putNumber("Checkpoint 1", 1);
        return Commands.race(
            new RunCommand(() -> SmartDashboard.putNumber("Checkpoint 3", 1)),
            new RunCommand(() -> driveToTarget()),
            Commands.waitSeconds(2)
        );
    }


    public void driveToTarget() {
        SmartDashboard.putNumber("Checkpoint 2", 2);
        double xSpeed = 0;
        double ySpeed = 0;
        double rotationSpeed = 0;
        
        if(limelight == null) {
            SmartDashboard.putString("return status", "return");
            return;
        }

        ChassisSpeeds chassisSpeeds;
        
        if(!limelight.hasValidTarget()) {
            chassisSpeeds = new ChassisSpeeds(0, 0, 0);
            SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
            SmartDashboard.putString("driveToTarget status", "no valid target");
        }
        else {
            SmartDashboard.putString("driveToTarget status", "has valid target");

            double averageX = limelight.getArea_avg();
            double averageY = limelight.getArea_avg();

            if((NerdyMath.inRange(averageY, -2, 2)) && (averageX > 7)) {
                chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                SmartDashboard.putString("driveToTarget status", "averageY and averageX conditions met");
                return;
            }

            xSpeed = PIDArea.calculate(averageX, goalArea);
            ySpeed = PIDTX.calculate(averageY, goalTX);

            SmartDashboard.putNumber("X-Speed", xSpeed);
            SmartDashboard.putNumber("Y-Speed", ySpeed);

            rotationSpeed = PIDYaw.calculate(drivetrain.getImu().getHeading(), goalYaw);

            if(NerdyMath.inRange(xSpeed, -0.1, 0.1) &&
            NerdyMath.inRange(ySpeed, -0.1, 0.1)) {
                chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                SmartDashboard.putString("driveToTarget status", "xSpeed ySpeed in range conditions met");
                return;
            }
            else {
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                SmartDashboard.putString("driveToTarget status", "chassisSpeeds in motion");
                return;
            }
        }
    }
}
