package org.firstinspires.ftc.teamcode.blucru.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.SinglePressGamepad;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.SixWheelDrive;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.TagCamera;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.shooter.Shooter;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.Transfer;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.outtake.turret.Turret;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.LimelightObeliskTagDetector;
import org.firstinspires.ftc.teamcode.blucru.common.util.ObeliskTagDetector;

public abstract class BluLinearOpMode extends LinearOpMode {

    // ===============================
    // CORE ROBOT + SUBSYSTEMS
    // ===============================
    public Robot robot;

    public Drivetrain drivetrain;
    public SixWheelDrive sixWheel;
    public Shooter shooter;
    public Intake intake;
    public Elevator elevator;
    public Transfer transfer;
    public Turret turret;
    public ObeliskTagDetector obeliskTagDetector;
    public LimelightObeliskTagDetector llTagDetector;

    // ===============================
    // CONTROL FLAGS
    // ===============================
    protected boolean reportTelemetry = true;

    /**
     * 🔑 IMPORTANT FLAG
     * If false, BluLinearOpMode will NOT call:
     *   robot.read()
     *   CommandScheduler.run()
     *   robot.write()
     *
     * RoadRunner autos should set this to FALSE and manage IO themselves.
     */
    protected boolean manageRobotLoop = true;

    // ===============================
    // GAMEPADS
    // ===============================
    public SinglePressGamepad driver1, driver2;

    // ===============================
    // LOOP TIMING
    // ===============================
    private double lastTimeLoopWasRun = 0;
    private double loopTimeSegmentSum = 0;
    private double amountOfLoopsInSegment = 0;
    private double amountOfLoopsOverall = 0;

    // ===============================
    // MAIN OPMODE ENTRY
    // ===============================
    @Override
    public final void runOpMode() throws InterruptedException {

        Globals.matchTime = new ElapsedTime();
        Globals.hwMap = hardwareMap;
        Globals.telemetry = telemetry;

        telemetry.update();
        Globals.telemetry.update();

        // Clear FTCLib state
        CommandScheduler.getInstance().cancelAll();

        driver1 = new SinglePressGamepad(gamepad1);
        driver2 = new SinglePressGamepad(gamepad2);

        robot = Robot.getInstance();
        robot.setHwMap(hardwareMap);
        Globals.updateVoltage(robot.getVoltage());
        robot.clear();

        // ---- USER INIT ----
        initialize();
        robot.init();

        // ===============================
        // INIT LOOP
        // ===============================
        while (opModeInInit()) {
            driver1.update();
            driver2.update();

            initializePeriodic();

            if (manageRobotLoop) {
                CommandScheduler.getInstance().run();
            }

            telemetry.addLine("Initialized");
            if (reportTelemetry) {
                telemetry();
                telemetry.addData("Subsystems", robot.getAmountOfSubsystems());
            }
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        Globals.matchTime.reset();

        if (manageRobotLoop) {
            robot.read();
        }

        // ---- USER START ----
        onStart();

        // ===============================
        // MAIN LOOP
        // ===============================
        while (opModeIsActive() && !isStopRequested()) {

            driver1.update();
            driver2.update();

            periodic();

            if (manageRobotLoop) {
                robot.read();
                CommandScheduler.getInstance().run();
                robot.write();
            }

            if (reportTelemetry) {
                telemetry();
                robot.telemetry(telemetry);
            }
            double[] loopTimes = getLoopTimes();
            telemetry.addData("Loop (ms)", loopTimes[0]);
            telemetry.addData("Hz", loopTimes[1]);
            telemetry.update();
        }

        // ===============================
        // CLEAN SHUTDOWN
        // ===============================
        CommandScheduler.getInstance().cancelAll();
        robot.clear();
        end();
    }

    // ===============================
    // USER OVERRIDES
    // ===============================
    public void initialize() {}
    public void initializePeriodic() {}
    public void onStart() {}
    public void periodic() throws InterruptedException {}
    public void telemetry() {}
    public void end() {}

    // ===============================
    // SUBSYSTEM ADDERS
    // ===============================
    public void addDrivetrain() { drivetrain = robot.addDrivetrain(); }
    public void addShooter()    { shooter = robot.addShooter(); }
    public void addIntake()     { intake = robot.addIntake(); }
    public void addTransfer()   { transfer = robot.addTransfer(); }
    public void addTurret()     { turret = robot.addTurret(); }
    public void addElevator()   { elevator = robot.addElevator(); }
    public void addSixWheel()   { sixWheel = robot.addSixWheelDrivetrain(); }
    public void addObeliskTagDetector() { obeliskTagDetector = robot.addObeliskTagDetector(); }
    public void addLLTagDetector()      { llTagDetector = robot.addLLTagDetector(); }

    public void enableDash() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Globals.telemetry = telemetry;
    }

    // ===============================
    // LOOP TIMING UTILS
    // ===============================
    public double[] getLoopTimes() {
        double now = Globals.matchTime.milliseconds();
        loopTimeSegmentSum += now - lastTimeLoopWasRun;
        lastTimeLoopWasRun = now;

        amountOfLoopsInSegment++;
        amountOfLoopsOverall++;

        double[] res = new double[2];
        res[1] = amountOfLoopsOverall / Math.max(Globals.matchTime.seconds(), 0.001);

        if (amountOfLoopsInSegment > 20) {
            res[0] = loopTimeSegmentSum / amountOfLoopsInSegment;
            loopTimeSegmentSum = 0;
            amountOfLoopsInSegment = 0;
        }

        return res;
    }
}
