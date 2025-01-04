package epra.movement;

import java.util.ArrayList;
import java.util.Arrays;

import epra.location.Odometry;
import epra.location.Pose;

/**A scheduler for moving a DriveTrain in a set pattern based on odometry.
 * <p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class PoseScheduler {

    private DriveTrain drive;
    private Odometry odometry;

    private ArrayList<Pose> directions;
    private ArrayList<Double> pos_tol;
    private ArrayList<Double> ang_tol;

    /**A scheduler for moving a DriveTrain in a set pattern based on odometry.
     * @param driveTrain The DriveTrain.
     * @param odometry The Odometry.*/
    public PoseScheduler(DriveTrain driveTrain, Odometry odometry) {
        this.drive = driveTrain;
        this.odometry = odometry;
        directions = new ArrayList<>();
        pos_tol = new ArrayList<>();
        ang_tol = new ArrayList<>();
    }

    /**Adds a step to the scheduler's list of directions.
     * @param step A step in the form of a Pose.
     * @param posTolerance The tolerance for reaching the xy pose.
     * @param angleTolerance The tolerance for reaching the angle.*/
    public void addStep(Pose step, double posTolerance, double angleTolerance) {
        directions.add(step);
        pos_tol.add(posTolerance);
        ang_tol.add(angleTolerance);
    }
    /**Adds an array of steps to the scheduler's list of directions.
     * @param steps An array of steps in the form of Poses.
     * @param posTolerance An array of tolerances for reaching the xy poses.
     * @param angleTolerance An array of tolerances for reaching the angles.*/
    public void addStep(Pose[] steps, double[] posTolerance, double[] angleTolerance) {
        for (int i = 0; i < steps.length; i++) { addStep(steps[i], posTolerance[i], angleTolerance[i]); }
    }

    /**Runs the current step, moving the DriveTrain to that Pose.
     * @return True if the target Pose is reached or if there are no steps in the scheduler's list of directions.*/
    public boolean runStep(DriveTrain driveTrain) {
        if (directions.isEmpty()) { return true; }
        else { return driveTrain.posPIDMecanumDrive(odometry.getPose(), directions.get(0), pos_tol.get(0), ang_tol.get(0), (true)); }
    }

    /**Advances the directions by one step.
     * @return False if the list of directions has no steps remaining after advancing.*/
    public boolean nextStep() {
        if (!directions.isEmpty()) { directions.remove(0); }
        return !directions.isEmpty();
    }

    /**@return The number of steps remaining in the pose scheduler.*/
    public int stepsRemaining() { return directions.size(); }
}
