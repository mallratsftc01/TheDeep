package org.firstinspires.ftc.teamcode;

import epra.control.Controller;
import epra.movement.DriveTrain;
import epra.location.IMUExpanded;
import epra.movement.Motor;
import epra.movement.MotorController;
import epra.movement.DcMotorExFrame;
import epra.location.Odometry;
import epra.location.Pose;
import epra.math.geometry.Angle;
import epra.math.geometry.Point;
import epra.movement.PIDTuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class PIDTest extends LinearOpMode {
    private MotorController northEastMotor;
    private MotorController southEastMotor;
    private MotorController southWestMotor;
    private MotorController northWestMotor;

    private MotorController horizontalArmMotor;
    private MotorController verticalArmMotor;
    private MotorController climberMotor;

    private Servo horizontalClaw;
    private Servo horizontalWrist;
    private Servo verticalClaw;

    private Controller controller1;
    private Controller controller2;

    private IMU imu1;
    private IMU imu2;
    private IMUExpanded imuX;

    private Odometry odometry;

    FtcDashboard dashboard;

    private PIDTuner pidTuner;

    int deltaDiff = 0;

    long lastPing;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorExFrame neMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northeastMotor"));
        DcMotorExFrame nwMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "northwestMotor"));
        DcMotorExFrame seMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southeastMotor"));
        DcMotorExFrame swMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "southwestMotor"));
        swMotor.setDirection(Motor.Direction.REVERSE);
        seMotor.setDirection(Motor.Direction.REVERSE);

        northEastMotor = new MotorController(neMotor);
        northWestMotor = new MotorController(nwMotor);
        southEastMotor = new MotorController(seMotor);
        southWestMotor = new MotorController(swMotor);

        DcMotorExFrame haMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "horizontalMotor"));
        DcMotorExFrame vaMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "verticalMotor"));
        DcMotorExFrame cMotor = new DcMotorExFrame(hardwareMap.get(DcMotorEx.class, "climberMotor"));
        vaMotor.setDirection(Motor.Direction.REVERSE);

        horizontalArmMotor = new MotorController(haMotor);
        horizontalArmMotor.tuneTargetPID(0.9, 0.0000001, 0.003);
        verticalArmMotor = new MotorController(vaMotor);
        verticalArmMotor.tuneTargetPID(0.7, 0.0000005, 0.005);
        climberMotor = new MotorController(cMotor);

        horizontalClaw = hardwareMap.get(Servo.class, "horizontalClaw");
        //horizontalWrist = hardwareMap.get(Servo.class, "horizontalWrist");
        verticalClaw = hardwareMap.get(Servo.class, "verticalClaw");

        controller1 = new Controller (gamepad1, 0.05F);
        controller2 = new Controller (gamepad2, 0.05F);

        DriveTrain drive = new DriveTrain(new String[] {"north_west_motor", "north_east_motor", "south_west_motor", "south_east_motor"}, new MotorController[] {northWestMotor, northEastMotor, southWestMotor, southEastMotor}, new DriveTrain.Orientation[] {DriveTrain.Orientation.LEFT_FRONT, DriveTrain.Orientation.RIGHT_FRONT, DriveTrain.Orientation.LEFT_BACK, DriveTrain.Orientation.RIGHT_BACK}, DriveTrain.DriveType.MECANUM);
        drive.tuneAnglePID(1, 0.00025, 270);
        drive.tunePointPID(0.25, 0.00000001, 35);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu1 = hardwareMap.get(IMU.class, "imu 1");
        //imu2 = hardwareMap.get(IMU.class, "imu 2");
        imu1.initialize(new IMU.Parameters(orientationOnRobot));
        //imu2.initialize(new IMU.Parameters(orientationOnRobot));
        imuX = new IMUExpanded(imu1);

        odometry = new Odometry(northWestMotor, southWestMotor, northEastMotor,
                new Point(7.92784216, 3.75),
                new Point(-8, 3.75),
                new Point(0, 2.0),
                imuX,
                new Pose(
                        new Point(0, 64.0),
                        new Angle(180.0)
                )
        );

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Config config = new Config();

        pidTuner = new PIDTuner();
        double[] seed = {1.0, 0.0, 0.0};
        double[][] tests = new double[10][3];
        double[] best_p = {seed[0], 1.0};
        double[] best_d = {seed[1], 1.0};

        double start = 100.0;
        double target = 200.0;

        waitForStart();

        for (int i = 0; i < tests.length; i++) {
            tests[i][0] = seed[0] + (Math.random() * 2.0) - 1.0;
            tests[i][1] = seed[1] + (Math.random() * 2.0) - 1.0;
            tests[i][2] = seed[2];
        }
        for (double[] test : tests) {
            long saveTime = System.currentTimeMillis();
            verticalArmMotor.tuneTargetPID(test[0], test[1], test[2]);
            verticalArmMotor.setTarget(verticalArmMotor.getCurrentPosition() + 100);
            while (System.currentTimeMillis() - saveTime < 5000) {
                verticalArmMotor.moveToTarget(1.0, 0.01, true);

            }
        }

    }
}
