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
        Pose2d INIT_POSE = new Pose2d(
                            3.709699869155884,
                            1.9885972738265991, Rotation2d.fromDegrees(90));
        var routine = autoFactory.newRoutine("Bump");
        routine.active()
            .onTrue(
                Commands.sequence(
                    Commands.runOnce(() -> drivetrain.resetPose(INIT_POSE)),
                    drivetrain.goToPoseCommand(() -> INIT_POSE),
                    Commands.runOnce(() -> SmartDashboard.putString("autostage", "stage 0")),
                    // Commands.waitSeconds(2),
                    Commands.runOnce(() -> SmartDashboard.putString("autostage", "done waitng")),
                    drivetrain.goToPoseCommand(() -> new Pose2d(
                        2.059431552886963,
                        2.451367139816284, Rotation2d.fromDegrees(90))
                    ),
                    drivetrain.pointAtPose(Constants.hubPose),
                    drivetrain.goToPoseCommand(() -> new Pose2d(
                        8.683216094970703,
                        2.8809804916381836,
                        Rotation2d.fromDegrees(90)
                    )),
                    Commands.runOnce(() -> SmartDashboard.putString("autostage", "across field")),
                    // drivetrain.pointAtPose(Constants.hubPose),
                    drivetrain.goToPoseCommand(() -> new Pose2d(
                        8.4885172843933105,
                        5.456998348236084,
                        Rotation2d.fromDegrees(90)
                    )),
                    // drivetrain.pointAtPose(Constants.hubPose),
                    drivetrain.goToPoseCommand(() -> new Pose2d(
                        1.6959257125854492,
                        5.471975326538086,
                        new Rotation2d()
                    )),
                    drivetrain.pointAtPose(Constants.hubPose),
                    drivetrain.pointAtPose(Constants.hubPose)
                )
            );
        return routine;
    }
}
