package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;

public class LimelightSnapshot extends BluLinearOpMode {

    private Limelight3A limelight;
    private int snapshotCount = 0;

    @Override
    public void initialize() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    @Override
    public void initializePeriodic(){

    }

    @Override
    public void periodic() {
        if (driver1.pressedA()) {
            limelight.captureSnapshot("snap_" + snapshotCount++);
            telemetry.addData("Snapshot", "Captured!");
            sleep(500); // debounce
        }
        telemetry.update();
    }
}
