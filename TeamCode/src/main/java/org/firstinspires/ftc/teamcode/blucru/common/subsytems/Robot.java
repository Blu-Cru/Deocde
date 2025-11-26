package org.firstinspires.ftc.teamcode.blucru.common.subsytems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.intake.Intake;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.transfer.Transfer;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.motor.BluMotorWithEncoder;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluCRServo;
import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluPIDServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.turret.Turret;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.PDController;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.SixWheelDrive;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.SixWheelDriveBase;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.mecanumDrivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.util.LimelightObeliskTagDetector;
import org.firstinspires.ftc.teamcode.blucru.common.util.ObeliskTagDetector;

import java.util.ArrayList;
import java.util.List;

public class Robot {

    //add all subsystems here
    //class will be a singleton bc there is only 1 robot


    //list of subsystems
    static ArrayList<BluSubsystem> subsystems;
    public Drivetrain drivetrain;
    public Shooter shooter;
    public Intake intake;
    public Elevator elevator;
    public Transfer transfer;
    public Turret turret;
    public Drivetrain mecanumDrivetrain;
    public SixWheelDrive sixWheelDrivetrain;
    public ObeliskTagDetector obeliskTagDetector;
    public LimelightObeliskTagDetector llTagDetector;
    private static Robot instance;
    HardwareMap hwMap;
    List<LynxModule> hubs;
    public static Robot getInstance(){
        if (instance == null){
            instance = new Robot();
            Robot.subsystems.clear();
        }
        return instance;
    }

    //private constructor to init subsystems

    private Robot(){
        subsystems = new ArrayList<>();
    }

    public void setHwMap(HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public void init(){
        //store the hubs in an array
        hubs = hwMap.getAll(LynxModule.class);

        //set the hubs to auto bulk reads
        for (LynxModule hub : hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //init all subsystems
        for (BluSubsystem subsystem : subsystems){
            subsystem.init();
        }
    }

    public void read(){


        //clear caches of both hubs
        for (LynxModule hub:hubs){
            hub.clearBulkCache();
        }

        //read from each subsytem
        for (BluSubsystem subsystem: subsystems){
            subsystem.read();
        }
    }

    public void write(){
        //write to each subsystem
        for (BluSubsystem subsystem : subsystems){
            subsystem.write();
        }
    }

    public double getVoltage(){

        //start at 12 bc that is the max power, otherwsie the power correction returns values < 1
        double result = 12;
        //read voltage sensor and
        for (VoltageSensor sensor: hwMap.voltageSensor){
            double voltage = sensor.getVoltage();
            if (voltage > 0){
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void telemetry(Telemetry telemetry){
        for (BluSubsystem subsystem: subsystems){
            subsystem.telemetry(telemetry);
        }
    }
    public void idleRobot(){
        for (BluSubsystem subsystem: subsystems){
            subsystem.reset();
        }
    }

    public double getAmountOfSubsystems(){
        return subsystems.size();
    }
    public void clear(){
        subsystems.clear();
    }

    public Drivetrain addDrivetrain(){
        mecanumDrivetrain = new Drivetrain();
        subsystems.add(mecanumDrivetrain);
        return mecanumDrivetrain;
    }

    public SixWheelDrive addSixWheelDrivetrain(){
        sixWheelDrivetrain = new SixWheelDrive();
        subsystems.add(sixWheelDrivetrain);
        return sixWheelDrivetrain;
    }

    public Shooter addShooter() {
        shooter = new Shooter();
        subsystems.add(shooter);
        return shooter;
    }
    public Intake addIntake(){
        intake = new Intake("intake");
        subsystems.add(intake);
        return intake;
    }
    public Turret addTurret(){
        turret = new Turret(new BluCRServo("servoLeft"), new BluCRServo("servoRight"), new BluEncoder(Globals.flMotorName), new PDController(0,0));
        subsystems.add(turret);
        return turret;
    }

    public ObeliskTagDetector addObeliskTagDetector(){
        obeliskTagDetector = ObeliskTagDetector.getInstance();
        subsystems.add(obeliskTagDetector);
        return obeliskTagDetector;
    }

    public LimelightObeliskTagDetector addLLTagDetector(){
        llTagDetector = new LimelightObeliskTagDetector();
        subsystems.add(llTagDetector);
        return llTagDetector;
    }




}
