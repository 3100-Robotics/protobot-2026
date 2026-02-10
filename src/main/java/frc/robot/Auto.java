package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Drivetrain;

public class Auto {
    private final Pose2d hubPose = new Pose2d(4.628518104553223, 4.035704612731934, new Rotation2d());

    
    private AutoFactory autoFactory;
    private AutoChooser autoChooser = new AutoChooser();

    // Subsystem refs
    private Drivetrain drivetrain;

    public Auto(Drivetrain drivetrain) {
        // autoChooser.addRoutine();
        this.drivetrain = drivetrain;
        autoFactory = new AutoFactory(
            this.drivetrain::getPos,
            this.drivetrain::resetPose,
            this.drivetrain::followTrajectory,
            true,
            this.drivetrain
        );

        autoChooser.addRoutine("Bump", this::bump);

        SmartDashboard.putData("Auton Selector", autoChooser);

        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    public Command aimAndShoot() {
        return Commands.run(() -> {}).withTimeout(5);
    }

    public AutoRoutine bump() {
        var routine = autoFactory.newRoutine("Bestie");
        routine.active()
            .onTrue(
                drivetrain.goToPoseCommand(new Pose2d()).withName("stage 0")
            );
        return routine;
    }
}
