package org.firstinspires.ftc.teamcode.blucru.common.util;

import org.firstinspires.ftc.teamcode.blucru.common.subsytems.drivetrain.sixWheelDrive.purePursuit.PurePursuitDebugData;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * Handles persistent CSV logging of PurePursuit debug data to Android storage
 * Uses a separate thread for file I/O to prevent blocking the main opmode
 */
public class PurePursuitLogger {
    private BufferedWriter writer;
    private String currentLogFilePath;
    private volatile boolean isLogging;
    private Thread loggingThread;
    private BlockingQueue<String> logQueue;

    private static final String LOG_DIRECTORY = "/sdcard/FIRST/purePursuit_logs/";
    private static final String POISON_PILL = "###STOP_LOGGING###";
    private static final int QUEUE_CAPACITY = 1000;

    public PurePursuitLogger() {
        this.writer = null;
        this.currentLogFilePath = null;
        this.isLogging = false;
        this.logQueue = new LinkedBlockingQueue<>(QUEUE_CAPACITY);
        this.loggingThread = null;
    }

    /**
     * Start logging to a new CSV file
     * @param testPathName Name of the test path being logged
     * @return true if logging started successfully, false otherwise
     */
    public boolean startLogging(String testPathName) {
        try {
            // Stop any existing logging thread
            if (loggingThread != null && loggingThread.isAlive()) {
                stopLogging();
            }

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

            // Clear queue and start logging
            logQueue.clear();
            isLogging = true;

            // Start the logging thread
            loggingThread = new Thread(new LoggingRunnable(), "PurePursuitLogger");
            loggingThread.setDaemon(true);
            loggingThread.start();

            return true;

        } catch (IOException e) {
            e.printStackTrace();
            isLogging = false;
            return false;
        }
    }

    /**
     * Log a single data point to the CSV file
     * Non-blocking - adds to queue for background thread to write
     * @param data Debug data to log
     * @return true if added to queue successfully, false otherwise
     */
    public boolean logDataPoint(PurePursuitDebugData data) {
        if (!isLogging) {
            return false;
        }

        try {
            // Non-blocking add to queue
            return logQueue.offer(data.toCSVRow() + "\n", 5, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return false;
        }
    }

    /**
     * Stop logging and close the file
     * Waits for the logging thread to finish processing the queue
     */
    public void stopLogging() {
        isLogging = false;

        // Signal the logging thread to stop
        if (loggingThread != null && loggingThread.isAlive()) {
            try {
                // Add poison pill to signal thread to stop
                logQueue.offer(POISON_PILL, 1, TimeUnit.SECONDS);

                // Wait for thread to finish (up to 2 seconds)
                loggingThread.join(2000);

                // If still alive, interrupt it
                if (loggingThread.isAlive()) {
                    loggingThread.interrupt();
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                loggingThread.interrupt();
            }
        }

        // Close writer if still open
        if (writer != null) {
            try {
                writer.flush();
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
            writer = null;
        }

        loggingThread = null;
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

    /**
     * Runnable for the background logging thread
     * Continuously processes the log queue and writes to file
     */
    private class LoggingRunnable implements Runnable {
        @Override
        public void run() {
            try {
                while (isLogging && !Thread.currentThread().isInterrupted()) {
                    try {
                        // Wait for log entries with timeout
                        String logEntry = logQueue.poll(100, TimeUnit.MILLISECONDS);

                        if (logEntry == null) {
                            // Timeout - continue waiting
                            continue;
                        }

                        // Check for poison pill (stop signal)
                        if (POISON_PILL.equals(logEntry)) {
                            break;
                        }

                        // Write to file
                        if (writer != null) {
                            writer.write(logEntry);

                            // Periodic flush to ensure data is written
                            // Flush every 50ms or when queue is empty
                            if (logQueue.isEmpty()) {
                                writer.flush();
                            }
                        }

                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        break;
                    } catch (IOException e) {
                        e.printStackTrace();
                        // Continue trying to log despite errors
                    }
                }

                // Final flush before closing
                if (writer != null) {
                    try {
                        writer.flush();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
