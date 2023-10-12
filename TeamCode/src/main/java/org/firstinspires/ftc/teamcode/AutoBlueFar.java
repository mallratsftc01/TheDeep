package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "AutoRedFar", group = "linear autoMode")

public class AutoBlueFar extends RobotLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private double speed;
    private final int oneRotaion = 540;
    private int newLFTarget;
    private int newLBTarget;
    private int newRFTarget;
    private int newRBTarget;

    @Override
    public void runOpMode() {
        declareHardwareProperties();
        waitForStart();
        encoderDrive(0.2, 1, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.6, 80, MOVEMENT_DIRECTION.STRAFE_LEFT);
    }
}