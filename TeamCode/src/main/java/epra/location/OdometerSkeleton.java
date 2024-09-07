package epra.location;

import epra.DriveTrain;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class OdometerSkeleton extends DriveTrain {

    public OdometerSkeleton(String[] motorNames, DcMotorEx[] motors, Orientation[] orientations, DriveType driveTypeIn) {
        super(motorNames, motors, orientations, driveTypeIn);
        super.updatePos();
    }
}
