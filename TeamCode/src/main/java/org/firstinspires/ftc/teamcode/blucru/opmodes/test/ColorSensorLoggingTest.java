package org.firstinspires.ftc.teamcode.blucru.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.hardware.BluColorSensor;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp
public class ColorSensorLoggingTest extends LinearOpMode {

    private BluColorSensor sensor1;
    private BluColorSensor sensor2;
    private BluColorSensor sensor3;
    private BluColorSensor sensor4;
    private BluColorSensor sensor5;
    private BluColorSensor sensor6;

    private BufferedWriter writer;
    private String logFilePath;

    @Override
    public void runOpMode() {
        // Initialize Globals for BluColorSensor to use
        Globals.hwMap = hardwareMap;
        Globals.telemetry = telemetry;

        // Initialize sensors - using default thresholds (can be adjusted)
        // Thresholds format: {{purpleBottom}, {purpleTop}, {greenBottom}, {greenTop}}
        // Each threshold is [Red, Green, Blue]
        double[][] defaultThresholds = {{0, 0, 0}, {1, 1, 1}, {0, 0, 0}, {1, 1, 1}};

        sensor1 = new BluColorSensor("middleColorSensorRight", defaultThresholds);
        sensor2 = new BluColorSensor("middleColorSensorLeft", defaultThresholds);
        sensor3 = new BluColorSensor("rightColorSensorTop", defaultThresholds);
        sensor4 = new BluColorSensor("rightColorSensorBottom", defaultThresholds);
        sensor5 = new BluColorSensor("leftColorSensorTop", defaultThresholds);
        sensor6 = new BluColorSensor("leftColorSensorBottom", defaultThresholds);

        // Initialize file logging
        try {
            initializeLogging();
            telemetry.addData("Status", "Logging initialized at: " + logFilePath);
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to initialize logging: " + e.getMessage());
            e.printStackTrace();
        }

        waitForStart();

        while (opModeIsActive()) {
            try {
                // Read all sensors and log data
                logColorSensorData();

                // Display current readings on driver station
                displaySensorData("sensor1", sensor1);
                displaySensorData("sensor2", sensor2);
                displaySensorData("sensor3", sensor3);
                displaySensorData("sensor4", sensor4);
                displaySensorData("sensor5", sensor5);
                displaySensorData("sensor6", sensor6);

                telemetry.addData("Logging", "Active to: " + logFilePath);
                telemetry.update();
            } catch (IOException e) {
                telemetry.addData("Error", "Logging failed: " + e.getMessage());
                telemetry.update();
                e.printStackTrace();
            }
        }

        // Close the log file
        try {
            if (writer != null) {
                writer.close();
                telemetry.addData("Status", "Logging stopped");
                telemetry.update();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void initializeLogging() throws IOException {
        // Create log directory if it doesn't exist
        File logDir = new File("/sdcard/FIRST/color_logs/");
        if (!logDir.exists()) {
            logDir.mkdirs();
        }

        // Create timestamped log file
        String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        logFilePath = logDir.getAbsolutePath() + "/color_data_" + timestamp + ".csv";

        // Initialize writer and write header
        writer = new BufferedWriter(new FileWriter(logFilePath));
        writer.write("timestamp,sensor1_R,sensor1_G,sensor1_B,sensor2_R,sensor2_G,sensor2_B,sensor3_R,sensor3_G,sensor3_B,sensor4_R,sensor4_G,sensor4_B,sensor5_R,sensor5_G,sensor5_B,sensor6_R,sensor6_G,sensor6_B");
        writer.newLine();
        writer.flush();
    }

    private void logColorSensorData() throws IOException {
        if (writer == null) return;

        // Read all sensors first
        sensor1.read();
        sensor2.read();
        sensor3.read();
        sensor4.read();
        sensor5.read();
        sensor6.read();

        // Get current timestamp
        String timestamp = new SimpleDateFormat("HH:mm:ss.SSS").format(new Date());

        // Build CSV row
        StringBuilder row = new StringBuilder(timestamp);

        // Log each sensor's RGB values
        row.append(",").append(formatValue(sensor1));
        row.append(",").append(formatValue(sensor2));
        row.append(",").append(formatValue(sensor3));
        row.append(",").append(formatValue(sensor4));
        row.append(",").append(formatValue(sensor5));
        row.append(",").append(formatValue(sensor6));

        // Write to file
        writer.write(row.toString());
        writer.newLine();
        writer.flush();
    }

    private String formatValue(BluColorSensor sensor) {
        if (sensor == null) {
            return "0,0,0";
        }
        return String.format("%.4f,%.4f,%.4f", sensor.getRed(), sensor.getGreen(), sensor.getBlue());
    }

    private void displaySensorData(String name, BluColorSensor sensor) {
        if (sensor == null) {
            telemetry.addData(name, "NOT CONNECTED");
        } else {
            telemetry.addData(name + " RGB", "R:%.4f G:%.4f B:%.4f", sensor.getRed(), sensor.getGreen(), sensor.getBlue());
        }
    }
}
