package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.util.NerdyMath;

public class PrimalPotatoMine {

    private Limelight limelight;
    private SwerveDrivetrain drivetrain;

    private double goalArea;
    private double goalTX;
    private double goalYaw;

    final PIDController PIDArea = new PIDController(0, 0, 0);
    final PIDController PIDTX = new PIDController(0, 0, 0);
    final PIDController PIDYaw = new PIDController(0, 0, 0);

    public PrimalPotatoMine(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        try {
            limelight = new Limelight("limelight-low");
            limelight.setLightState(Limelight.LightMode.OFF);
        } catch (Exception ex) {
            limelight = null;
        }

        limelight.setPipeline(2);
    }
    

    public CommandBase PickupGroundNoArm() {
        return Commands.race(
            new RunCommand(() -> driveToTarget()),
            Commands.waitSeconds(2)
        );
    }


    public void driveToTarget() {
        double xSpeed = 0;
        double ySpeed = 0;
        double rotationSpeed = 0;
        
        if(limelight == null) {
            return;
        }

        ChassisSpeeds chassisSpeeds;
        
        if(!limelight.hasValidTarget()) {
            chassisSpeeds = new ChassisSpeeds(0, 0, 0);
            SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            drivetrain.setModuleStates(moduleStates);
        }
        else {
            double averageX = limelight.getArea_avg();
            double averageY = limelight.getArea_avg();

            if((NerdyMath.inRange(averageY, -2, 2)) && (averageX > 7)) {
                chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                return;
            }

            xSpeed = PIDArea.calculate(averageX, goalArea);
            ySpeed = PIDTX.calculate(averageY, goalTX);
            rotationSpeed = PIDYaw.calculate(drivetrain.getImu().getHeading(), goalYaw);

            if(NerdyMath.inRange(xSpeed, -0.1, 0.1) &&
            NerdyMath.inRange(ySpeed, -0.1, 0.1)) {
                chassisSpeeds = new ChassisSpeeds(0, 0, 0);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                return;
            }
            else {
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);
                SwerveModuleState[] moduleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
                drivetrain.setModuleStates(moduleStates);
                return;
            }
        }
    }
}
