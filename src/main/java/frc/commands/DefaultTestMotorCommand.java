package frc.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TestMotorConstants;
import frc.robot.subsystems.TestMotorSubsystem;

public class DefaultTestMotorCommand extends Command {

    private final TestMotorSubsystem testMotorSubsystem;
    private final double rollerSpeed;
    private final XboxController subController;

    public DefaultTestMotorCommand(TestMotorSubsystem subsystem, double rollerSpeed) {
        this.testMotorSubsystem = subsystem;
        this.rollerSpeed = rollerSpeed;
        this.subController = new XboxController(OIConstants.kCoPilotControllerPort);

        // Declare subsystem dependencies
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // Stop the positional motor and set an initial roller speed
        testMotorSubsystem.setPositionalMotorDutyCycle(0);
        testMotorSubsystem.setRotationalMotorDutyCycle(rollerSpeed * 0.5);
    }

    @Override
    public void execute() {
        // Adjust roller speed based on right stick Y-axis input
        double rotationalInput = Math.abs(subController.getRightY());
        testMotorSubsystem.setRotationalMotorDutyCycle(rollerSpeed * rotationalInput);

        // Adjust positional motor speed based on left stick Y-axis input
        double positionalInput = Math.abs(subController.getLeftY()) * TestMotorConstants.kTestPosMotorSpeed;
        testMotorSubsystem.setPositionalMotorDutyCycle(positionalInput);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop both motors when the command ends
        testMotorSubsystem.stopRotationalMotor();
        testMotorSubsystem.stopPositionalMotor();
    }
}
