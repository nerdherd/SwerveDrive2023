package frc.robot.subsystems.vision.primalWallnut;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerAutos;
import frc.robot.subsystems.Reportable;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.Limelight.LightMode;
import frc.robot.subsystems.vision.jurrasicMarsh.LimelightHelpers;

public class PrimalSunflower implements Reportable{
    //filler positions, need to actually get right positions later
    private Double[][] gridPositions = {
        {6.5, 1.1, 0.0},
        {6.5, 0.4, 0.0},
        {6.5, -0.15, 0.0},
        {6.5, -0.7, 0.0},
        {6.5, -1.25, 0.0},
        {6.5, -1.8, 0.0},
        {6.5, -2.35, 0.0},
        {6.5, -2.95, 0.0},
        {6.5, -3.55, 0.0}
    };

    //robot position
    private Double[] robotPos = {0.0, 0.0, 0.0};

    //points in the path to get to the closest grid
    PathPoint firstPoint;
    PathPoint secondPoint;
    PathPoint thirdPoint;

    private Limelight limelight;

    private SwerveDrivetrain drivetrain;

    private PIDController PIDArea = new PIDController(0, 0, 0);
    private PIDController PIDTX = new PIDController(0, 0, 0);
    private PIDController PIDYaw = new PIDController(0, 0, 0);

    private String llname;
    
    /*
     * Params:
     * limelightName = name of the limelight
     * drivetrain = swerve drive 
    */
    public PrimalSunflower(String limelightName, SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.llname = limelightName;

        try {
            limelight = new Limelight(limelightName);
            limelight.setLightState(LightMode.OFF);
            limelight.setPipeline(4); // april tag
        } catch (Exception e) {
            limelight = null;
            DriverStation.reportWarning("Error instantiating limelight with name " + limelightName + ": " + e.getMessage(), true);
        }

        SmartDashboard.putNumber("Tx P", 0);       
        SmartDashboard.putNumber("Tx I", 0);
        SmartDashboard.putNumber("Tx D", 0);

        SmartDashboard.putNumber("Ta P", 0);       
        SmartDashboard.putNumber("Ta I", 0);
        SmartDashboard.putNumber("Ta D", 0);

        SmartDashboard.putNumber("Yaw P", 0);       
        SmartDashboard.putNumber("Yaw I", 0);
        SmartDashboard.putNumber("Yaw D", 0);
    }
    

    //get robot position if limelight has target else, return 0, 0, 0
    private Double[] generateSun() {
        Double[] yee = {0.0, 0.0, 0.0};
        if(limelight == null) return yee;
        Pose3d pos = new Pose3d();
        if(limelight.hasValidTarget()) {
            pos = LimelightHelpers.getBotPose3d(llname); // Replace w different met.
            return new Double[]{pos.getX(), pos.getY(), pos.getZ()};

        }
        return yee;
    }

    //get closest grid node by comparing robot pos to each known grid node pos
    public Double[] getClosestZombie() {
        robotPos = generateSun();
        int gridNumber = 0;
        Double distance = Math.sqrt(Math.pow(gridPositions[0][1] - robotPos[1], 2) + Math.pow(gridPositions[0][0] - robotPos[0], 2)); // distance formula
        for (int i = 0; i < gridPositions.length; i++) {
            Double newDistance = Math.sqrt(Math.pow(gridPositions[i][1] - robotPos[1], 2) + Math.pow(gridPositions[i][0] - robotPos[0], 2)); // distance formula
            if(newDistance < distance) {
                distance = newDistance;
                gridNumber = i;
            }
        }
        SmartDashboard.putNumber("Closest Grid:", gridNumber);
        return gridPositions[gridNumber];
    }

    /**
     * @return PathPlannerTrajectory to get to the closest grid
     */
    public PathPlannerTrajectory attackZombie() {
        robotPos = generateSun();
        Double[] gridPos = getClosestZombie();

        Double yDist = gridPos[1] - robotPos[1];
        Double xDist = gridPos[0] - robotPos[0];
        Double offset = 0.1;

        firstPoint = new PathPoint(new Translation2d(xDist - offset, robotPos[1]), Rotation2d.fromDegrees(0));
        secondPoint = new PathPoint(new Translation2d(robotPos[0], yDist), Rotation2d.fromDegrees(0));
        thirdPoint = new PathPoint(new Translation2d(offset, robotPos[1]), Rotation2d.fromDegrees(0));
        
        SmartDashboard.putString("ATag First Point Coords", "X: " + firstPoint.position.getX() + " Y: " + firstPoint.position.getY());
        SmartDashboard.putString("ATag Second Point Coords", "X: " + secondPoint.position.getX() + " Y: " + secondPoint.position.getY());
        SmartDashboard.putString("ATag Third Point Coords", "X: " + thirdPoint.position.getX() + " Y: " + thirdPoint.position.getY());

        return PathPlanner.generatePath(
            new PathConstraints(3, 3),
            firstPoint,
            secondPoint,
            thirdPoint
            );
    }


    @Override
    public void reportToSmartDashboard(LOG_LEVEL priority) {
        // TODO Auto-generated method stub
        
    }


    @Override
    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab;
        switch (level) {
            case OFF:
                break;
            case ALL:
            tab = Shuffleboard.getTab("Vision");
                tab.addString("Robot Position", () -> ("X: " + robotPos[0] + "Y: " + robotPos[1]));
                default:
                break;
        }
    }
}
