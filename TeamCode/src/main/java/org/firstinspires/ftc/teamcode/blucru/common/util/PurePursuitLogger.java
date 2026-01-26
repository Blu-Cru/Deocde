package org.firstinspires.ftc.teamcode.blucru.common.util;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.sixWheelDrive.purePursuit.PurePursuitDebugData;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Handles persistent CSV logging of PurePursuit debug data to Android storage
 */
public class PurePursuitLogger {
    private BufferedWriter writer;
    private String currentLogFilePath;
    private boolean isLogging;

    private static final String LOG_DIRECTORY = "/sdcard/FIRST/purePursuit_logs/";

    public PurePursuitLogger() {
        this.writer = null;
        this.currentLogFilePath = null;
        this.isLogging = false;
    }

    /**
     * Start logging to a new CSV file
     * @param testPathName Name of the test path being logged
     * @return true if logging started successfully, false otherwise
     */
    public boolean startLogging(String testPathName) {
        try {
            // Create directory if it doesn't exist
            File directory = new File(LOG_DIRECTORY);
            if (!directory.exists()) {
                directory.mkdirs();
            }

            // Create filename with timestamp
            SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US);
            String timestamp = dateFormat.format(new Date());
            String filename = String.format("pp_debug_%s.csv", timestamp);
            currentLogFilePath = LOG_DIRECTORY + filename;

            // Open file for writing
            File logFile = new File(currentLogFilePath);
            writer = new BufferedWriter(new FileWriter(logFile));

            // Write header comment with test path name
            writer.write("# PurePursuit Debug Log\n");
            writer.write("# Test Path: " + testPathName + "\n");
            writer.write("# Timestamp: " + new Date().toString() + "\n");

            // Write CSV header
            writer.write(PurePursuitDebugData.toCSVHeader() + "\n");
            writer.flush();

            isLogging = true;
            return true;

        } catch (IOException e) {
            e.printStackTrace();
            isLogging = false;
            return false;
        }
    }

    /**
     * Log a single data point to the CSV file
     * @param data Debug data to log
     * @return true if logged successfully, false otherwise
     */
    public boolean logDataPoint(PurePursuitDebugData data) {
        if (!isLogging || writer == null) {
            return false;
        }

        try {
            writer.write(data.toCSVRow() + "\n");
            return true;
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Stop logging and close the file
     */
    public void stopLogging() {
        if (writer != null) {
            try {
                writer.flush();
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
            writer = null;
        }
        isLogging = false;
    }

    /**
     * Get the current log file path
     * @return Path to current log file, or null if not logging
     */
    public String getLogFilePath() {
        return currentLogFilePath;
    }

    /**
     * Check if currently logging
     * @return true if logging, false otherwise
     */
    public boolean isLogging() {
        return isLogging;
    }

    /**
     * Get the log filename (without path)
     * @return Filename of current log, or null if not logging
     */
    public String getLogFilename() {
        if (currentLogFilePath == null) {
            return null;
        }
        return new File(currentLogFilePath).getName();
    }
}
