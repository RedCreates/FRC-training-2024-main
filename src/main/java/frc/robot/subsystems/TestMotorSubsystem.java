package frc.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TestMotorConstants;

public class TestMotorSubsystem extends SubsystemBase {
    // Motors
    private final CANSparkMax rotMotor = new CANSparkMax(TestMotorConstants.kTestPosMotorCanId, MotorType.kBrushed);
    private final CANSparkMax posMotor = new CANSparkMax(TestMotorConstants.kTestRotMotorCanId, MotorType.kBrushless);

    private final XboxController subController = new XboxController(OIConstants.kCoPilotControllerPort);

    // PID Controllers
    private final SparkPIDController posMotorPID = posMotor.getPIDController();
    // private final SparkPIDController rotMotorPID = rotMotor.getPIDController();
    // Encoders
    private final RelativeEncoder posMotorEncoder = posMotor.getEncoder();
    

    // Target Variables
    private double targetVelocitySetpoint = 0.0;
    private Double targetPositionSetpoint = null;


    public TestMotorSubsystem() {
        // Register the subsystem
        CommandScheduler.getInstance().registerSubsystem(this);

        // Configure motors
        rotMotor.setIdleMode(IdleMode.kBrake);
        posMotor.setIdleMode(IdleMode.kBrake);

        rotMotor.setSmartCurrentLimit(20);
        posMotor.setSmartCurrentLimit(20);

        // Configure PID controllers
        // rotMotorPID.setP(TestMotorConstants.kTestMotorP);
        // rotMotorPID.setI(TestMotorConstants.kTestMotorI);
        // rotMotorPID.setD(TestMotorConstants.kTestMotorD);
        // rotMotorPID.setFF(TestMotorConstants.kTestMotorFF);

        posMotorPID.setP(TestMotorConstants.kTestPosMotorP);
        posMotorPID.setI(TestMotorConstants.kTestPosMotorI);
        posMotorPID.setD(TestMotorConstants.kTestPosMotorD);
        posMotorPID.setFF(TestMotorConstants.kTestPosMotorFF);
        posMotorPID.setFeedbackDevice(posMotorEncoder);

        // Configure soft limits for position motor
        posMotor.setSoftLimit(SoftLimitDirection.kForward, (float) TestMotorConstants.kTestMotorTopPosition);
        posMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) TestMotorConstants.kTestMotorIdlePosition);
        posMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        posMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        // Set output ranges
        // rotMotorPID.setOutputRange(TestMotorConstants.kTestMotorMinOutput, TestMotorConstants.kTestMotorMaxOutput);
        posMotorPID.setOutputRange(TestMotorConstants.kTestPosMotorMinOutput, TestMotorConstants.kTestPosMotorMaxOutput);

        // Initialize encoder position
        posMotorEncoder.setPosition(0);
        posMotorPID.setFeedbackDevice(posMotorEncoder);
    }

    // @Override
    // public void periodic() {
    //     // Update positional motor PID only if targetPositionSetpoint is not null
    //     if (targetPositionSetpoint != null) {
    //         posMotorPID.setReference(targetPositionSetpoint, ControlType.kPosition);
    //     }

    //     // Send telemetry to SmartDashboard
    //     SmartDashboard.putNumber("Positional Motor Position", posMotorEncoder.getPosition());
    //     SmartDashboard.putNumber("Rotational Motor Output", rotMotor.getAppliedOutput());
    //     SmartDashboard.putBoolean("Rotational Motor At Speed", isAtTargetSpeed());
    //     SmartDashboard.putBoolean("Positional Motor At Position", isAtTargetPosition());

    //     if (targetPositionSetpoint != null) {
    //         SmartDashboard.putNumber("Target Position Setpoint", targetPositionSetpoint);
    //     }
    // }

    public void setPositionalMotorPosition(double position){
        posMotorPID.setReference(position, ControlType.kPosition);
    }

    // Rotational Motor Methods
    public void setRotationalMotorDutyCycle(double dutyCycle) {
        rotMotor.set(dutyCycle);
    }

    // public void setRotationalMotorVelocity(double velocity) {
    //     rotMotorPID.setReference(velocity, ControlType.kVelocity);
    //     targetVelocitySetpoint = velocity;
    // }

    public void stopRotationalMotor() {
        setRotationalMotorDutyCycle(0);
        targetVelocitySetpoint = 0;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("posMotorPosition", getPositionalMotorPosition());
    }

    // public boolean isAtTargetSpeed() {
    //     return Math.abs(rotMotor.getEncoder().getVelocity() - targetVelocitySetpoint) < TestMotorConstants.kTestMotorSpeedDeadband;
    // }

    // Positional Motor Methods
    public void setPositionalMotorDutyCycle(double dutyCycle) {
        // targetPositionSetpoint = null; // Disable position control
        posMotor.set(dutyCycle);
    }

    public void stopPositionalMotor() {
        posMotor.stopMotor();
        targetPositionSetpoint = posMotorEncoder.getPosition();
    }

    public boolean isAtTargetPosition() {
        if (targetPositionSetpoint == null) {
            return false; // No position target set
        }
        return Math.abs(posMotorEncoder.getPosition() - targetPositionSetpoint) < TestMotorConstants.kTestPosMotorPositionDeadband;
    }

    public double getPositionalMotorPosition() {
        return posMotorEncoder.getPosition();
    }
}
