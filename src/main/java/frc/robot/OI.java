package frc.robot;

//import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class OI {
    /*
        Add your joysticks and buttons here
    */
    private Joystick primaryJoystick = new Joystick(0);
    private Joystick secondaryJoystick = new Joystick(1);

    public OI() {
        // Back button zeroes the drivetrain
        new JoystickButton(primaryJoystick, 7).whenPressed(new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyroscope()));
    }
    /**
     * This function returns the primary joystick.
     * @return Joystick port 0
     */
    public Joystick getPrimaryJoystick() {
        return primaryJoystick;
    }
    /**
     * This function returns the secondary joystick.
     * @return Joystick port 1
     */
    public Joystick getSecondaryJoystick() {
        return secondaryJoystick;
    }
}