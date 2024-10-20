package org.firstinspires.ftc.teamcode;

import epra.Controller;
import epra.DriveTrain;
import epra.IMUExpanded;
import epra.location.Odometry;
import epra.location.Pose;
import epra.math.geometry.Angle;
import epra.math.geometry.Geometry;
import epra.math.geometry.Point;
import epra.math.geometry.Quadrilateral;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.Arrays;


@TeleOp
public class TheDeep extends LinearOpMode {

    private final Point LEFT_ENCODER = new Point(-8.0, 3.0);
    private final Point RIGHT_ENCODER = new Point(8.0, 3.0);

    private DcMotorEx northEastMotor;
    private DcMotorEx southEastMotor;
    private DcMotorEx southWestMotor;
    private DcMotorEx northWestMotor;

    private Controller controller1;
    private Controller controller2;

    private IMU imu1;
    private IMU imu2;
    private IMUExpanded imuX;

    private Odometry odometry;

    FtcDashboard dashboard;

    int deltaDiff = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        northEastMotor = hardwareMap.get(DcMotorEx.class, "northeastMotor");
        northWestMotor = hardwareMap.get(DcMotorEx.class, "northwestMotor");
        southEastMotor = hardwareMap.get(DcMotorEx.class, "southeastMotor");
        southWestMotor = hardwareMap.get(DcMotorEx.class, "southwestMotor");
        northWestMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        northEastMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        controller1 = new Controller (gamepad1, 0.05F);
        controller2 = new Controller (gamepad2, 0.05F);

        DriveTrain drive = new DriveTrain(new String[] {"north_west_motor", "north_east_motor", "south_west_motor", "south_east_motor"}, new DcMotorEx[] {northWestMotor, northEastMotor, southWestMotor, southEastMotor}, new DriveTrain.Orientation[] {DriveTrain.Orientation.LEFT_FRONT, DriveTrain.Orientation.RIGHT_FRONT, DriveTrain.Orientation.LEFT_BACK, DriveTrain.Orientation.RIGHT_BACK}, DriveTrain.DriveType.MECANUM);

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

        waitForStart();
        while (opModeIsActive()) {
            controller1.update();
            controller2.update();

            drive.setDrivePower(controller1.analogDeadband(Controller.Key.RIGHT_STICK_X), controller1.analogDeadband(Controller.Key.LEFT_STICK_X), controller1.analogDeadband(Controller.Key.RIGHT_STICK_Y), controller1.analogDeadband(Controller.Key.LEFT_STICK_Y));

            odometry.estimatePose();

            TelemetryPacket packet = new TelemetryPacket();
            /*odometry.drawPose(new Quadrilateral(
                            new Point(9.0, 9.0),
                            new Point(-9.0, 9.0),
                            new Point(-9.0, -9.0),
                            new Point(9.0, -9.0)
                    ), true
            );*/

            packet.put("X", odometry.getPose().point.x);
            packet.put("Y", odometry.getPose().point.y);
            packet.put("Angle", odometry.getPose().angle.getDegree());

            packet.put("Right Encoder", odometry.getPos(Odometry.Orientation.RIGHT));
            packet.put("Left Encoder", odometry.getPos(Odometry.Orientation.LEFT));
            packet.put("Perpendicular Encoder", odometry.getPos(Odometry.Orientation.PERPENDICULAR));

            packet.put("Delta Right", odometry.getDelta(Odometry.Orientation.RIGHT));
            packet.put("Delta Left", odometry.getDelta(Odometry.Orientation.LEFT));
            packet.put("Delta Perpendicular", odometry.getDelta(Odometry.Orientation.PERPENDICULAR));

            packet.put("Phi", odometry.getPhi().getDegree());

            packet.put("Center Displacement", odometry.centerDisplacement());
            packet.put("Perpendicular Displacement", odometry.perpendicularDisplacement());

            //packet.put("Current Angle: ", imuX.getYaw().getDegree());
            //packet.put("Target Angle: ", controller1.analogDeadband(Controller.Stick.RIGHT_STICK).getDegree());
            //packet.put("Right Pow, direction, distance: ", Arrays.toString(drive.gyroMecanumDrive(controller1.analogDeadband(Controller.Key.LEFT_STICK_X), controller1.analogDeadband(Controller.Key.LEFT_STICK_Y), controller1.analogDeadband(Controller.Stick.RIGHT_STICK), imuX)));
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
