package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final int driveMotorID;
    private final int turnMotorID;
    private final int absoluteEncoderId;

    private final PIDController turningController;
    private final TalonSRX absoluteTurningEncoder;
    private final boolean invertTurningEncoder;
    private final double absoluteEncoderOffset;

    /**
     * Constuct a new Swerve Module
     * 
     * @param driveMotorId              CAN ID of the drive motor
     * @param turningMotorId            CAN ID of the turning motor
     * @param invertDriveMotor          Whether or not the drive motor is inverted         
     * @param invertTurningMotor        Whether or not the turning motor is inverted
     * @param absoluteEncoderId         CAN ID of the absolute encoder
     * @param absoluteEncoderOffset     Zero position for the absolute encoder (in ticks)
     * @param absoluteEncoderReversed   Whether or not the absolute encoder is inverted
     */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean invertDriveMotor, boolean invertTurningMotor, 
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.driveMotor = new TalonFX(driveMotorId);
        this.turnMotor = new TalonFX(turningMotorId);

        this.driveMotor.setNeutralMode(NeutralMode.Coast);
        this.turnMotor.setNeutralMode(NeutralMode.Coast);

        this.driveMotorID = driveMotorId;
        this.turnMotorID = turningMotorId;
        this.absoluteEncoderId = absoluteEncoderId;

        this.turningController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        turningController.enableContinuousInput(-Math.PI, Math.PI);
        turningController.setTolerance(.025);

        this.driveMotor.setInverted(invertDriveMotor);
        this.turnMotor.setInverted(invertTurningMotor);
        this.absoluteTurningEncoder = new TalonSRX(absoluteEncoderId);
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.invertTurningEncoder = absoluteEncoderReversed;

        initEncoders();
    }

    private void initEncoders() {
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 1000);
        absoluteTurningEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 1000);
        absoluteTurningEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.PulseWidthEncodedPosition, 1, 1000);
    }

    /**
     * Reset the SRX's quadrature encoder (slot 0) using the SRX's builtin encoder (slot 1)
     */
    public void resetEncoder() {
        double startPos = (absoluteTurningEncoder.getSelectedSensorPosition(1) - absoluteEncoderOffset) % 4096;
        double startAngle = startPos / 4096;
        SmartDashboard.putNumber("Reset Angle Encoder #" + absoluteEncoderId, startAngle);
        absoluteTurningEncoder.setSelectedSensorPosition(startPos, 0, 100);
    }

    /**
     * Set the percent output of both motors to zero.
     */
    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(ControlMode.PercentOutput, 0);
    }

    //****************************** GETTERS ******************************/

    /**
     * Get the distance travelled by the motor in meters
     * @return Distance travelled by motor (in meters)
     */
    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition(0) 
            * ModuleConstants.kDriveTicksToMeters
            * ModuleConstants.kDriveMotorGearRatio;
    }

    /**
     * Get the angle of the turning motor from the absolute encoder's quad encoder
     * @return Angle in radians
     */
    public double getTurningPosition() {
        double turningPosition = -(Math.IEEEremainder(absoluteTurningEncoder.getSelectedSensorPosition(0), 4096) * ModuleConstants.kTurningTicksToRad);
        // SmartDashboard.putNumber("turning position motor #" + turnMotorID, turningPosition);
        // SmartDashboard.putNumber("Turning angle #" + turnMotorID, 180 * turningPosition / Math.PI);
        return turningPosition;
    }

    /**
     * Get the velocity of the drive motor
     * @return Velocity of the drive motor (in meters / sec)
     */
    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity(0) * ModuleConstants.kDriveTicksPer100MsToMetersPerSec;
    }

    /**
     * Get the velocity of the turning motor
     * @return Velocity of the turning motor (in radians / sec)
     */
    public double getTurningVelocity() {
        double turnVelocity = turnMotor.getSelectedSensorVelocity(0) * ModuleConstants.kTurningTicksPer100MsToRadPerSec;
        // SmartDashboard.putNumber("Turn velocity Motor #" + turnMotorID, turnVelocity);
        return turnVelocity;
    }

    /**
     * Get the position of the absolute encoder
     * @return Position of the absolute encoder (in radians)
     */
    public double getAbsoluteEncoderRadians() {
        double angle = (absoluteTurningEncoder.getSelectedSensorPosition(1) % 4096) * ModuleConstants.kTurningTicksToRad + absoluteEncoderOffset;
        // SmartDashboard.putNumber("AbsoluteEncoder in rad", angle * (invertTurningEncoder ? -1 : 1));
        return angle * (invertTurningEncoder ? -1 : 1);
    }

    /**
     * Return the current state (velocity and rotation) of the Swerve Module
     * @return This Swerve Module's State
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    //****************************** SETTERS ******************************/

    /**
     * Set the desired state of the Swerve Module and move towards it
     * @param state The desired state for this Swerve Module
     */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // state.angle = state.angle.rotateBy(Rotation2d.fromDegrees(-90));
        state = SwerveModuleState.optimize(state, getState().angle);
        // SmartDashboard.putNumber("Desired Angle Motor #" + turnMotorID, state.angle.getDegrees());
        SmartDashboard.putNumber("Angle Difference Motor #" + turnMotorID, state.angle.getDegrees() - (getTurningPosition() * 180 / Math.PI));
        
        // TODO: switch to velocity control
        // driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond);
        double percentOutput = state.speedMetersPerSecond / SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        SmartDashboard.putNumber("Motor percent #" + turnMotorID, percentOutput);
        driveMotor.set(ControlMode.PercentOutput, percentOutput);
        
        double turnPower = turningController.calculate(getTurningPosition(), state.angle.getRadians());
        // SmartDashboard.putNumber("Turn Power Motor #" + turnMotorID, turnPower);

        turnMotor.set(ControlMode.PercentOutput, turnPower);
        SmartDashboard.putNumber("Turn angle #" + turnMotorID, Math.toDegrees(getTurningPosition()));

        SmartDashboard.putNumber("Current Motor #" + turnMotorID, driveMotor.getStatorCurrent());
    }

    public void setBreak(boolean breaking) {
        NeutralMode mode = (breaking ? NeutralMode.Brake : NeutralMode.Coast);
        this.driveMotor.setNeutralMode(mode);
        this.turnMotor.setNeutralMode(mode);
    }
}
