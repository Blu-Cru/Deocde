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
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.Transfer;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.Turret;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.LimelightObeliskTagDetector;
import org.firstinspires.ftc.teamcode.blucru.common.util.ObeliskTagDetector;

public abstract class BluLinearOpMode extends LinearOpMode {

    public Robot robot;
    protected boolean reportTelemetry = true;

    public Drivetrain drivetrain;
    public SixWheelDrive sixWheel;
    public Shooter shooter;
    public Intake intake;
    public Elevator elevator;
    public Transfer transfer;
    public Turret turret;
    public ObeliskTagDetector obeliskTagDetector;
    public LimelightObeliskTagDetector llTagDetector;

    //add all of the subsystems that need to be added to the robot here


    //2 gamepads are if there are 2 drivers, doesnt matter if there is just 1 driver
    public SinglePressGamepad driver1, driver2;


    //loop time information
     double lastTimeLoopWasRun = 0, loopTimeSegmentSum = 0,
            loopTimeSegmentTimeOffset = 0, amountOfLoopsInSegment = 0,
            amountOfLoopsOverall = 0;


    //the function is final because it should not change for any opMode with this,
    //all opmodes should be able to run the sm and loop
    //bc of this it runs at a low level without complicated logic
    public final void runOpMode() throws InterruptedException {
        Globals.matchTime = new ElapsedTime();
        Globals.hwMap = hardwareMap;
        Globals.telemetry = telemetry;
        telemetry.update();
        Globals.telemetry.update();

        //clears all possible running commands
        CommandScheduler.getInstance().cancelAll();

        driver1 = new SinglePressGamepad(gamepad1);
        driver2 = new SinglePressGamepad(gamepad2);


        robot = Robot.getInstance();
        robot.setHwMap(Globals.hwMap);
        Globals.updateVoltage(robot.getVoltage());
        robot.clear();


        initialize();
        robot.init();

        telemetry.addLine("here");
        while(opModeInInit()){
            //update gamepads
            driver1.update();
            driver2.update();

            //make sure that when switching controllers nothing happens in periodic
            //not using the single press gps because it is not single-press based to switch gp controllers
            if (!((gamepad1.start && gamepad1.b) || (gamepad2.start && gamepad2.a))){
                initializePeriodic();
            }

            //run any sent commands, in case they are sent
            CommandScheduler.getInstance().run();


            //always say init and update even if telemetry shouldnt be reported so that
            //the driver knows if the program is ready to run
            telemetry.addLine("Initialized");
            if (reportTelemetry){
                telemetry();
                telemetry.addData("match time", Globals.matchTime.milliseconds());
                telemetry.addData("Amount of Subsystems", robot.getAmountOfSubsystems());
            }
            telemetry.update();
        }
        waitForStart();
        //reset timer and get latest data
        Globals.matchTime.reset();
        robot.read();

        onStart();

        while(opModeIsActive()){
            driver1.update();
            driver2.update();

            //safety for switching controllers again
            if (!((gamepad1.start && gamepad1.b) || (gamepad2.start && gamepad2.a))){
                periodic();
            }

            robot.read();

            CommandScheduler.getInstance().run();

            robot.write();

            if (reportTelemetry){
                double[] loopTimes = getLoopTimes();
                telemetry();
                robot.telemetry(telemetry);
                telemetry.addData("Alliance: ", Globals.alliance);
                telemetry.addData("Segment Loop Time: ", loopTimes[0]);
                telemetry.addData("Overall Loop Time: ", loopTimes[1]);
                telemetry.update();
            }
        }


        telemetry.clearAll();
        end();
    }


    //runs the initialilzation of the opMode, before the loop starts
    public void initialize() {

    }
    /**
     * this is for things that should be looping while in initialization
     * ex. selecting alliance, selecting auto
     * */
    public void initializePeriodic(){

    }


    /**
     * this occurs right before the main loop starts
     */
    public void onStart(){

    }

    /**
     * this is called every loop that is unique to the opmode
     * ex. driving in tele
     */
    public void periodic(){

    }

    /**
     * this is for printing telemetry not already handled by the robot
     * */
    public void telemetry(){

    }

    /**
     * this is called at the end of an opmode
     * */
    public void end(){

    }

    public void addDrivetrain(){drivetrain = robot.addDrivetrain();}
    public void addShooter(){shooter = robot.addShooter();}
    public void addIntake(){
        intake = robot.addIntake();
    }
    public void addTransfer(){
        transfer = robot.addTransfer();
    }
    public void addTurret(){turret = robot.addTurret();}
    public void addObeliskTagDetector(){obeliskTagDetector = robot.addObeliskTagDetector();}
    public void addLLTagDetector(){llTagDetector = robot.addLLTagDetector();}
    public void addSixWheel(){sixWheel = robot.addSixWheelDrivetrain();}
    public void addElevator(){elevator = robot.addElevator();}
    public void enableDash(){
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        Globals.telemetry = telemetry;
    }

    public double[] getLoopTimes(){

        loopTimeSegmentSum += Globals.matchTime.milliseconds() - lastTimeLoopWasRun;

        lastTimeLoopWasRun = Globals.matchTime.milliseconds();

        amountOfLoopsInSegment++;
        amountOfLoopsOverall++;

        double[] res = new double[2];
        res[1] = (Globals.matchTime.milliseconds()) / amountOfLoopsOverall;

        //update loop frequency every 20 loops
        if (amountOfLoopsInSegment > 20){
            res[0] = (loopTimeSegmentSum) / amountOfLoopsInSegment;
            loopTimeSegmentSum = 0;
            amountOfLoopsInSegment = 0;
        }

        return res;


    }

}
