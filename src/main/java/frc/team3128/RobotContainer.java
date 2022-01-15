package frc.team3128;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3128.commands.ArcadeDrive;
import frc.team3128.commands.Shoot;
import frc.team3128.commands.TestDrive;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.subsystems.NAR_Drivetrain;
import frc.team3128.subsystems.TestBenchSubsystem;
import frc.team3128.subsystems.Shooter.ShooterState;
import frc.team3128.subsystems.Shooter;
import frc.team3128.common.utility.Log;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    //private NAR_Drivetrain m_drive;
    private TestBenchSubsystem testBenchSubsystem;
    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;
    private Shooter shooter;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();
    private Command auto;

    private boolean DEBUG = false;

    public RobotContainer() {

        //m_drive = NAR_Drivetrain.getInstance();
        shooter = Shooter.getInstance();
        //Enable all PIDSubsystems so that useOutput runs

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);
        testBenchSubsystem = new TestBenchSubsystem();
        //m_commandScheduler.setDefaultCommand(testBenchSubsystem, new TestDrive(testBenchSubsystem));

        shooter.enable();

        configureButtonBindings();
        dashboardInit();
    }   

    private void configureButtonBindings() {
        SmartDashboard.putString("cog", " ");
        //m_rightStick.getButton(1).whenActive(new RunCommand(testBenchSubsystem::drive,testBenchSubsystem));
        //m_rightStick.getButton(1).whenReleased(new RunCommand(testBenchSubsystem::stop,testBenchSubsystem));

        //Shoot shootCmd = new Shoot(shooter, Shooter.ShooterState.LAUNCHPAD);

        m_rightStick.getButton(2).whenActive(new SequentialCommandGroup(new PrintCommand("button 2 active"), 
                                                                        new Shoot(shooter, Shooter.ShooterState.LAUNCHPAD)));
        // m_rightStick.getButton(2).whenReleased(new RunCommand(() -> {
        //     shootCmd.end(false);
        // }));

        m_rightStick.getButton(1).whenActive(new RunCommand(() -> {
            SmartDashboard.putString("cog", "potato - sohan");
        }));

    }

    private void dashboardInit() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            //SmartDashboard.putData("Drivetrain", m_drive);
        }
            
    }

    public void stopDrivetrain() {
        //m_drive.stop();
    }

    public Command getAutonomousCommand() {
        return auto;
    }
}
