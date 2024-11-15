package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TestMotorConstants;

public class TestMotorSubsystem extends SubsystemBase {
    private final CANSparkMax kTestMotor = new CANSparkMax(TestMotorConstants.kTestMotorCanId, MotorType.kBrushless);

    private final XboxController subController = new XboxController(OIConstants.kCoPilotControllerPort);

    private final SparkPIDController testMotorPID = kTestMotor.getPIDController();

    private double targetSetPoint = 0;

    public TestMotorSubsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);
        kTestMotor.setIdleMode(IdleMode.kBrake);

        testMotorPID.setP(TestMotorConstants.kTestMotorP);
        testMotorPID.setI(TestMotorConstants.kTestMotorI);
        testMotorPID.setD(TestMotorConstants.kTestMotorD);
        testMotorPID.setFF(TestMotorConstants.kTestMotorFF);

        testMotorPID.setOutputRange(TestMotorConstants.kTestMotorMinOutput,TestMotorConstants.kTestMotorMaxOutput);

        kTestMotor.set(subController.getLeftY()*TestMotorConstants.kTestMotorSpeed);

    }

    public void setTestMotor(int setPoint) {
        kTestMotor.set(setPoint);
    }

    public void setTestMotorVelocity(double setPoint){
        testMotorPID.setReference(setPoint, ControlType. kVelocity);
        targetSetPoint = setPoint;
    }

    public boolean atSpeed() {
    return Math.abs(kTestMotor.getEncoder().getVelocity() - targetSetPoint) < TestMotorConstants.kTestMotorSpeedDeadband;
    } 

    public void stopRollers(boolean stopTestMotor) {
    setTestMotor(0);
    if(stopTestMotor) {
      stopShooterRollers();
    }
    }

    public void stopShooterRollers() {
    testMotorPID.setReference(0, ControlType.kDutyCycle);
    targetSetPoint = 0;
    }

}
