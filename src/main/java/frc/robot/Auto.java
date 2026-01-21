package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import frc.robot.subsystems.Drivetrain;

public class Auto {
    private AutoChooser autoChooser = new AutoChooser();
    private AutoFactory autoFactory;

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
    }

    // public AutoRoutine relative_ex3() {
        
    // }
}
