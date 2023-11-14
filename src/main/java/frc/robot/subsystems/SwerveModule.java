package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleState;
import frc.lib.util.SwerveModuleConstants;

import frc.robot.Constants;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Swerve.angleKF;
import static frc.robot.Constants.Swerve.angleKP;
import static java.lang.Math.PI;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private SparkMaxPIDController velocityController;

    private PIDController anglePIDcontroller = new PIDController(0.5 * (6.29 / 360), 0, 0);

    private DutyCycleEncoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new DutyCycleEncoder(moduleConstants.ABS_ENCIDER_PORT);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.ANGLE_MOTOR_ID, kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.DRIVE_MOTOR_ID, kBrushless);
        configDriveMotor();

        lastAngle = getState().angle;

        velocityController = driveMotor.getPIDController();

        anglePIDcontroller.enableContinuousInput(0, 360);
    }

    public void setDesiredState(edu.wpi.first.math.kinematics.SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
//        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
//        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(edu.wpi.first.math.kinematics.SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            driveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPSToMotor(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            velocityController.setReference(velocity, CANSparkMax.ControlType.kVelocity, 0, feedforward.calculate(velocity, 0), SparkMaxPIDController.ArbFFUnits.kVoltage);
        }
    }

    private void setAngle(edu.wpi.first.math.kinematics.SwerveModuleState desiredState){
        if (Math.abs(desiredState.speedMetersPerSecond) > (Constants.Swerve.maxSpeed * 0.01)) {

//            Rotation2d angle = desiredState.angle;
            Rotation2d angle = Rotation2d.fromDegrees(180);

//            angleController.setReference(Conversions.degreesToMotor(angle.getDegrees(), Constants.Swerve.angleGearRatio), CANSparkMax.ControlType.kPosition, 0);
//            mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
            angleMotor.set(anglePIDcontroller.calculate(angleMotor.getEncoder().getPosition(), angle.getDegrees()));
//            System.out.println(Conversions.degreesToMotor(angle.getDegrees(), Constants.Swerve.angleGearRatio));
//            System.out.println(angleMotor.getEncoder().getPosition());

            lastAngle = angle;
        }
    }

    public Rotation2d getRelativeAngle(){
        return Rotation2d.fromDegrees(Conversions.motorToDegrees(angleMotor.getEncoder().getPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getABSangle(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition() * 360);
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToMotor(getABSangle().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        angleMotor.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.reset();
    }

    private void configAngleMotor(){
        angleMotor.restoreFactoryDefaults();
        angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        angleMotor.setIdleMode(Constants.Swerve.angleIdleMode);
        angleMotor.getEncoder().setPositionConversionFactor(Constants.Swerve.angleGearRatio);
//        resetToAbsolute();
        angleMotor.getEncoder().setPosition(0);
    }

    private void configDriveMotor(){        
        driveMotor.restoreFactoryDefaults();
        driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        driveMotor.setIdleMode(Constants.Swerve.driveIdleMode);
        driveMotor.getEncoder().setPositionConversionFactor(Constants.Swerve.driveGearRatio);
        driveMotor.getEncoder().setPosition(0);
    }

    public edu.wpi.first.math.kinematics.SwerveModuleState getState(){
        return new edu.wpi.first.math.kinematics.SwerveModuleState(
            Conversions.motorToMPS(driveMotor.getEncoder().getVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
            getRelativeAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.motorToMeters(driveMotor.getEncoder().getPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
            getRelativeAngle()
        );
    }
}