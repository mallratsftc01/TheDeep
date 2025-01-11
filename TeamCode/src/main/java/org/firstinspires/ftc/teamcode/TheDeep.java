package org.firstinspires.ftc.teamcode;

import epra.Controller;
import epra.movement.DriveTrain;
import epra.IMUExpanded;
import epra.movement.Motor;
import epra.movement.MotorController;
import epra.movement.DcMotorExFrame;
import epra.location.Odometry;
import epra.location.Pose;
import epra.math.geometry.Angle;
import epra.math.geometry.Point;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class TheDeep extends LinearOpMode {
    private MotorController northEastMotor;
    private MotorController southEastMotor;
    private MotorController southWestMotor;
    private MotorController northWestMotor;

    private MotorController horizontalArmMotor;
    private MotorController verticalArmMotor;

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
        vaMotor.setDirection(Motor.Direction.REVERSE);

        horizontalArmMotor = new MotorController(haMotor);
        horizontalArmMotor.tuneTargetPID(0.9, 0.0000001, 0.003);
        verticalArmMotor = new MotorController(vaMotor);
        verticalArmMotor.tuneTargetPID(0.7, 0.0000005, 0.005);

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

        waitForStart();
        lastPing = System.currentTimeMillis();
        while (opModeIsActive()) {
            controller1.update();
            controller2.update();

            odometry.estimatePose();

            //drive code

            drive.setDrivePower(controller1.analogDeadband(Controller.Key.RIGHT_STICK_X), controller1.analogDeadband(Controller.Key.LEFT_STICK_X), controller1.analogDeadband(Controller.Key.RIGHT_STICK_Y), controller1.analogDeadband(Controller.Key.LEFT_STICK_Y));

            //arm code

            //horizontalArmMotor.setPower(controller2.analogDeadband(Controller.Key.RIGHT_STICK_Y));
            verticalArmMotor.setPower(controller2.analogDeadband(Controller.Key.LEFT_STICK_Y));

            //claw code

            //horizontalClaw.setPosition((controller2.buttonToggleSingle(Controller.Key.Y)) ? -1.0 : 1.0);
            //horizontalWrist.setPosition((controller2.buttonToggleSingle(Controller.Key.A)) ? 1.0 : -1.0);
            //verticalClaw.setPosition((controller2.buttonToggleSingle(Controller.Key.UP)) ? 1.0 : -1.0);

            TelemetryPacket packet = new TelemetryPacket();
            /*odometry.drawPose(new Quadrilateral(
                            new Point(9.0, 9.0),
                            new Point(-9.0, 9.0),
                            new Point(-9.0, -9.0),
                            new Point(9.0, -9.0)
                    ), true
            );*/

            packet.fieldOverlay()
                    .setFill("blue")
                    .fillCircle(odometry.getPose().point.x, odometry.getPose().point.y, 2);

            packet.put("Ping Time", System.currentTimeMillis() - lastPing);
            lastPing = System.currentTimeMillis();

            packet.put("X", odometry.getPose().point.x);
            packet.put("Y", odometry.getPose().point.y);
            packet.put("Angle", odometry.getPose().angle.getDegree());

            packet.put("Lift Pos", verticalArmMotor.getCurrentPosition());
            telemetry.addData("Lift Pos: ", verticalArmMotor.getCurrentPosition());

            /*packet.put("Right Stick Angle", controller1.analogDeadband(Controller.Stick.RIGHT_STICK).getDegree());
            packet.put("Right Stick length", controller1.analogDeadband(Controller.Stick.RIGHT_STICK).getLength());
            packet.put("Right Stick x", gamepad1.right_stick_x);
            packet.put("Right Stick y", gamepad1.right_stick_y);*/

            /*packet.put("Right Encoder", odometry.getPos(Odometry.Orientation.RIGHT));
            packet.put("Left Encoder", odometry.getPos(Odometry.Orientation.LEFT));
            packet.put("Perpendicular Encoder", odometry.getPos(Odometry.Orientation.PERPENDICULAR));

            packet.put("Delta Right", odometry.getDelta(Odometry.Orientation.RIGHT));
            packet.put("Delta Left", odometry.getDelta(Odometry.Orientation.LEFT));
            packet.put("Delta Perpendicular", odometry.getDelta(Odometry.Orientation.PERPENDICULAR));

            packet.put("Phi", odometry.getPhi().getDegree());

            packet.put("Center Displacement", odometry.centerDisplacement());
            packet.put("Perpendicular Displacement", odometry.perpendicularDisplacement());*/

            //packet.put("Current Angle: ", imuX.getYaw().getDegree());
            //packet.put("Target Angle: ", controller1.analogDeadband(Controller.Stick.RIGHT_STICK).getDegree());
            //packet.put("Right Pow, direction, distance: ", Arrays.toString(drive.gyroMecanumDrive(controller1.analogDeadband(Controller.Key.LEFT_STICK_X), controller1.analogDeadband(Controller.Key.LEFT_STICK_Y), controller1.analogDeadband(Controller.Stick.RIGHT_STICK), imuX)));
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
