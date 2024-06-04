package frc.robot.subsystems.shooter;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.helpers.RangeTable.RangeEntry;
import frc.robot.subsystems.SubsystemCommands;
import frc.robot.Constants.*;

public class ShooterCommands extends SubsystemCommands{
    private Shooter shooter;
    public ShooterCommands(Shooter shooter){
        this.shooter = shooter;
    }
    /**
     * Command to spin up the flywheels to the specified RPMs
     *
     * @param leftRPM {@code int}: the target RPM of the left flywheel
     * (should be faster than the right if there is spin).
     * @param rightRPM {@code int}: the target RPM of the right flywheel
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted
     * to end
     */
    public Command primeCommand(int leftRPM, int rightRPM){
        return shooter.run(() -> {
            shooter.setShooterVelocity(leftRPM, rightRPM);
        });
    }

    /**
     * Command to spin up the flywheels to the specified RPMs
     *
     * @param leftRPM {@code IntSupplier}: a lambda that returns the target
     * RPM of the left flywheel (should be faster than the right if
     * there is spin).
     * @param rightRPM {@code IntSupplier}: a lambda that returns the target
     * RPM of the right flywheel.
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted
     * to end
     */
    public Command primeCommand(IntSupplier leftRPM, IntSupplier rightRPM){
        return shooter.run(() -> {
            shooter.setShooterVelocity(leftRPM.getAsInt(), rightRPM.getAsInt());
        });
    }

    /**
     * Command to spin up the flywheels to the RPMs stored in
     * the specified {@code RangeEntry}
     *
     * @param entry {@code RangeEntry}: the {@code RangeEntry} object containing 
     * the target flywheel speeds
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted
     * to end
     */
    public Command primeCommand(RangeEntry entry){
        return primeCommand(entry.leftFlywheelSpeed, entry.rightFlywheelSpeed);
    }

    /**
     * Command to spin up the flywheels to the RPMs in the supplied
     * {@code RangeEntry}
     *
     * @param entry {@code Supplier<RangeEntry>}: a lambda that returns a
     * {@code RangeEntry} object containing the target flywheel speeds.
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be interrupted
     * to end
     */
    public Command primeCommand(Supplier<RangeEntry> entry){
        return primeCommand(() -> entry.get().leftFlywheelSpeed, () -> entry.get().rightFlywheelSpeed);
    }

    /**
     * Command to run the note in the shooter (if there is one) into the flywheels.
     * There should be code elsewhere that blocks this from running if the flywheels
     * aren't spinning.
     *
     * @return the command
     *
     * @apiNote This command ends after {@link SHOOTER#SHOOT_SCORE_TIME} seconds
     */
    public Command fireCommand(){
        return shooter.run(() -> {
            shooter.feederMotor.setPercentOutput(SHOOTER.SHOOTING_FEEDER_POWER);
        }).withTimeout(SHOOTER.SHOOT_SCORE_TIME);
    }

    /**
     * Command to run the flywheels and feeders backwards to score in the amp. There
     * should be code elsewhere preventing this from running if the elevator isn't in
     * the amp position.
     *
     * @return the command
     *
     * @apiNote This command ends after {@link SHOOTER#AMP_SCORE_TIME} seconds
     */
    public Command ampScoreCommand(){
        return shooter.run(() -> {
            shooter.feederMotor.setVelocity(SHOOTER.AMP_FEEDER_SPEED);
            shooter.setFlywheelVelocity(SHOOTER.AMP_FLYWHEEL_SPEED);
        }).withTimeout(SHOOTER.AMP_SCORE_TIME);
    }

    /**
     * Command to stop the flywheels and feeder
     *
     * @return the command
     *
     * @apiNote This command runs instantly and
     * ends on the same frame
     */
    public Command stopCommand(){
        return shooter.runOnce(() -> {
            shooter.leftShooterMotor.setVelocity(0);
            shooter.leftTargetSpeed = 0;
            shooter.rightShooterMotor.setVelocity(0);
            shooter.rightTargetSpeed = 0;
            shooter.feederMotor.setVelocity(0);
        });
    }

    /**
     * Command to stop everything in preparation for auto
     *
     * @return the command
     *
     * @apiNote This command runs for one frame and ends immediately
     */
    public Command autonomousInitCommand(){
        return stopCommand();
    }


    /**
     * Command to run the no-note-yet part of the intake routine. This
     * should be run with the rest of the intake routine.
     *
     * @return the command
     *
     * @apiNote This command runs until {@link Shooter#isBottomBeamBreakTripped()}
     * returns {@code true}
     */
    public Command intakeNoContactCommand(){
        return shooter.run(() -> {
            shooter.feederMotor.setVelocity(SHOOTER.INTAKE_FEEDER_SPEED, 0);
            shooter.setFlywheelVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED);
        }).until(() -> shooter.isBottomBeamBreakTripped());
    }

    /**
     * Command to run the part of the intake routine that gets the note
     * from the of the feeder to the flywheels.
     *
     * @return the command
     *
     * @apiNote This command runs until {@link Shooter#isBottomBeamBreakTripped()}
     * returns {@code false}
     */
    public Command intakeWithContactCommand(){
        return shooter.run(() -> {
            shooter.feederMotor.setPercentOutput(SHOOTER.INTAKE_FEEDER_POWER);
            shooter.setFlywheelVelocity(SHOOTER.INTAKE_FLYWHEEL_SPEED);
        }).until(() -> !shooter.isBottomBeamBreakTripped());
    }

    /**
     * Command to adjust the note's position to get it in a good spot
     * for shooting.
     *
     * @return the command
     */
    public Command intakeAdjustNoteCommand(){
        return shooter.run(
            () -> {
                shooter.setFlywheelVelocity(SHOOTER.ALIGN_FLYWHEEL_SPEED);
                shooter.feederMotor.setVelocity(SHOOTER.ALIGN_FEEDER_SPEED, 1);
            }
        ).until(
            () -> shooter.isTopBeamBreakTripped() && shooter.feederMotor.getVelocity() < 0
        ).andThen(
            shooter.run(() -> {
                shooter.setFlywheelVelocity(SHOOTER.ALIGN_FLYWHEEL_SPEED);
                shooter.feederMotor.setVelocity(SHOOTER.ALIGN_FEEDER_SPEED, 1);
            }
        ).until(
            () -> !shooter.isTopBeamBreakTripped())
        );
    }

    /**
     * Command that runs the feeder and flywheels backwards. Usually
     * needs to be run with {@link Intake#outakeCommand()} as well.
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be
     * interrupted to end
     */
    public Command outakeCommand(){
        return shooter.run(() -> {
            shooter.setFlywheelVelocity(SHOOTER.OUTAKE_FLYWHEEL_SPEED);
            shooter.feederMotor.setVelocity(SHOOTER.OUTAKE_FEEDER_SPEED);
        });
    }

    /**
     * Command to run the passthrough routine. Usually needs to be
     * paired with the {@link Intake#intakeCommand()}.
     *
     * @return the command
     *
     * @apiNote This command doesn't end on its own; it must be
     * interrupted to end
     */
    public Command passThroughCommand(){
        return shooter.run(() -> {
            if(shooter.isBottomBeamBreakTripped()){
                shooter.feederMotor.setVelocity(1);
            }
            else{
                shooter.feederMotor.setVelocity(SHOOTER.INTAKE_FEEDER_SPEED, 0);
            }
            shooter.setFlywheelVelocity(5000);
        });
    }
}
