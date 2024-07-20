package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class MAXSwereveModule {

    // create variables for modules
    private final CANSparkMax kTurningMAX;
    private final CANSparkMax kDrivingMAX;

    // create variables for encoders
    private final RelativeEncoder kDriveEncoder;
    private final AbsoluteEncoder kTurningEncoder;

    // create variables for PIDcontrollers
    private final SparkPIDController kTurningPID;
    private final SparkPIDController kDrivingPID;

    // initial value for chassis offset
    private double chassisoffset = 0.0;
    // create variable for the state of the module
    private SwerveModuleState targetstate = new SwerveModuleState();

    public MAXSwereveModule(int turningCANid, int drivingCANid, double angleOffset) {
        // initiate the module motors
        kTurningMAX = new CANSparkMax(turningCANid, MotorType.kBrushless);
        kDrivingMAX = new CANSparkMax(drivingCANid, MotorType.kBrushless);

        // reset to factory defaults
        kDrivingMAX.restoreFactoryDefaults();
        kTurningMAX.restoreFactoryDefaults();
        
        // initiate the module encoders
        kDriveEncoder = kDrivingMAX.getEncoder();
        kTurningEncoder = kTurningMAX.getAbsoluteEncoder(Type.kDutyCycle);
        // initiate the PIDcontrollers
        kTurningPID = kTurningMAX.getPIDController();
        kDrivingPID = kDrivingMAX.getPIDController();
        // set up the encoders into the PIDcontrollers
        kTurningPID.setFeedbackDevice(kTurningEncoder);
        kTurningPID.setFeedbackDevice(kDriveEncoder);

        // use the conversion factors from class "Constants" and affiliate them with the PIDcontrollers
        kDriveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        kDriveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        kTurningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        kTurningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        // invert the turning encoder since it is positioned inversely into the motor
        kTurningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

        // enable the PIDcontrollers to efficiently(quickly) reset to the offset of the turning motor
        kTurningPID.setPositionPIDWrappingEnabled(true);
        kTurningPID.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        kTurningPID.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        // assign PID contstants from "Constants" for driving
        kDrivingPID.setP(ModuleConstants.kDrivingP);
        kDrivingPID.setI(ModuleConstants.kDrivingI);
        kDrivingPID.setD(ModuleConstants.kDrivingD);
        kDrivingPID.setFF(ModuleConstants.kDrivingFF);
        // set an output range for the PID constants
        kDrivingPID.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

        // assign PID contstants from "Constants" for turning
        kTurningPID.setP(ModuleConstants.kTurningP);
        kTurningPID.setI(ModuleConstants.kTurningI);
        kTurningPID.setD(ModuleConstants.kTurningD);
        kTurningPID.setFF(ModuleConstants.kTurningFF);
        // set an output range for the PID constants
        kTurningPID.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

        // setup idlemode tying to the class "Constant"
        kDrivingMAX.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        kTurningMAX.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        // setup smartcurrent system that prevents unnecessary usage of power 
        kDrivingMAX.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        kTurningMAX.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        // save up and apply the configuration
        kDrivingMAX.burnFlash();
        kTurningMAX.burnFlash();

        chassisoffset = angleOffset;
        targetstate.angle = new Rotation2d(kTurningEncoder.getPosition());
        kDriveEncoder.setPosition(0);
        
    }

    public SwerveModuleState getState() {
        // method that returns the current state value of swerve
        // uses drive encoder to get velocity
        // uses the turning encoder to get the position after being adjusted by the chassis offset
        // returns velocity and angle
        return new SwerveModuleState(kDriveEncoder.getVelocity(), new Rotation2d(kTurningEncoder.getPosition() - chassisoffset));
    }

    public SwerveModulePosition getPosition() {
        // method that returns the current position of swerve
        // uses the drive encoder to get the position
        // uses the turning encoder to the position after being adjusted by the chassis offset
        // returns poosition and angle
        return new SwerveModulePosition(kDriveEncoder.getPosition(), new Rotation2d(kTurningEncoder.getPosition() - chassisoffset));
    }

    // sets the desired state of the module
    public void setState(SwerveModuleState tState){

        // creates a new swerve module state that holds the adjusted state
        SwerveModuleState correctState = new SwerveModuleState();

        // set to target speed from "tState"
        correctState.speedMetersPerSecond = tState.speedMetersPerSecond;
        // added chassis offset
        correctState.angle = tState.angle.plus(Rotation2d.fromRadians(chassisoffset));

        // optimize with minimal movement
        SwerveModuleState dState = SwerveModuleState.optimize(correctState, new Rotation2d(kTurningEncoder.getPosition()));

        // sets desired speed of driving motor and angle for turning motor with the PID controllers
        kDrivingPID.setReference(dState.speedMetersPerSecond, ControlType.kVelocity);
        kTurningPID.setReference(dState.angle.getRadians(), ControlType.kPosition); // there is no such thing as angle controltype

        // updated the target state
        targetstate = dState;
    }

    // resets the drive encoder position to zero
    public void resetEncoder() {
        kDriveEncoder.setPosition(0);
        // the turning encoder is not resetted because it is important to keep the turning state offset acknowledged
    }
}
