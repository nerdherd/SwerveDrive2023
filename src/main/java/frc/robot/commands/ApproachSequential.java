package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToTarget.PipelineType;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;

public class ApproachSequential extends SequentialCommandGroup {
    public ApproachSequential(Limelight limelight, SwerveDrivetrain drivetrain, PipelineType pipeline){
        addCommands(
            new TurnToAngle(180, drivetrain),
            new DriveToTarget(drivetrain, limelight, 0.2, pipeline)
        );
    }
}
