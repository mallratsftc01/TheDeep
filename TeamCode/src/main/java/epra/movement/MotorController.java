package epra.movement;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**Gives increase control over DcMotorExs.
 *<p></p>
 * Queer Coded by Striker-909. If you use this class or a method from this class in its entirety, please make sure to give credit.*/
public class MotorController {

    private Motor motor;

    private double velocity;
    private int savePos;
    private long saveTime;

    private PIDController pidT;
    private int startPos;
    private int targetPosition;

    private PIDController pidV;
    private double targetVelocity;
    /**Gives increase control over DcMotorExs.
     *@param motor The motor to be used by this MotorController.*/
    public MotorController(Motor motor) {
        this.motor = motor;
        velocity = 0;
        startPos = motor.getCurrentPosition();
        targetPosition = startPos;
        targetVelocity = 0;
        pidT = new PIDController(1, 0, 0);
        pidV = new PIDController(1, 0, 0);
        savePos = startPos;
        saveTime = System.currentTimeMillis();
    }

    /**Sets the motor to a certain power between -1.0 and 1.0.
     * @param power The power to set the motor to.*/
    public void setPower(double power) { motor.setPower(power); }
    /**Stops the motor.*/
    public void stop() { motor.setPower(0.0); }

    /**Saves motor data to internal logs.
     * @return The time, in milliseconds, since the last log.*/
    public long log() {
        int posChange = motor.getCurrentPosition() - savePos;
        long timeChange = System.currentTimeMillis() - saveTime;
        savePos += posChange;
        saveTime += timeChange;
        velocity = (double) posChange / (double) timeChange;
        return timeChange;
    }

    /**Returns the current reading of the motor's encoder in ticks relative to the start position.
     * These ticks are specific to the encoder of a certain motor; google the ticks/revolution for your motor for best results.
     * If the encoder wire for this motor is not connected to the motor (ie. it its instead connected to an odometry pod) this number will not reflect the movement of this encoder.
     * @return The current reading of the motor's encoder. */
    public int getCurrentPosition() { return motor.getCurrentPosition() - startPos; }
    /**Returns the recent average velocity of the motor's encoder in ticks per second.
     * These ticks are specific to the encoder of a certain motor; google the ticks/revolution for your motor for best results.
     * If the encoder wire for this motor is not connected to the motor (ie. it its instead connected to an odometry pod) this number will not reflect the movement of this encoder.
     * @return The recent average velocity of the motor's encoder. */
    public double getVelocity() { return velocity; }
    /**Returns the current power being sent to the motor as a double between -1.0 and 1.0.
     * @returns The current power being sent to the robot.*/
    public double getPower() { return motor.getPower(); }

    /**Sets a target position for the motor to try to move towards.
     * @param target The position for the motor to try to move to in motor-specific ticks.*/
    public void setTarget(int target) { targetPosition = target; }
    /**Moves the motor towards the set target.
     * @return True once the motor reaches its target, false until then.*/
    public boolean moveToTarget() {
        double power = pidT.runPID(getCurrentPosition(), targetPosition);
        if (Math.abs(power) > 0.001) {
            setPower(power);
            return false;
        } else {
            stop();
            return true;
        }
    }

    /**Tunes the PID loop used to reach a target.
     * @param k_p The P constant.
     * @param k_i The I constant.
     * @param k_d the D constant.*/
    public void tuneTargetPID(double k_p, double k_i, double k_d) {
        pidT.tuneP(k_p);
        pidT.tuneI(k_i);
        pidT.tuneD(k_d);
    }
    /**Resets the PID loop used to reach a target.*/
    public void resetTargetPID() { pidT.reset(); }

    /**Sets a target velocity for the motor to try to maintain.
     * Setting the target velocity higher than the motor's maximum velocity could lead to unwanted side effects.
     * @param target The velocity for the motor to try to maintain in motor-specific ticks/millisecond.*/
    public void setTargetVelocity(double target) { targetVelocity = target; }
    /**Maintains the targeted velocity.
     * @return True once the motor reaches its target, false until then.*/
    public boolean maintainVelocity() {
        double power = pidV.runPID(getVelocity(), targetVelocity);
        if (Math.abs(power) > 0.001) {
            setPower(power);
            return false;
        } else {
            stop();
            return true;
        }
    }

    /**Tunes the PID loop used to maintain a velocity.
     * @param k_p The P constant.
     * @param k_i The I constant.
     * @param k_d the D constant.*/
    public void tuneVelocityPID(double k_p, double k_i, double k_d) {
        pidV.tuneP(k_p);
        pidV.tuneI(k_i);
        pidV.tuneD(k_d);
    }
    /**Resets the PID loop used to maintain a velocity.*/
    public void resetVelocityPID() { pidV.reset(); }
}
