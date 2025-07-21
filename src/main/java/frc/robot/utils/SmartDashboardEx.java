package frc.robot.utils;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardEx {
        // 内部类，用于存储单个记录（时间戳和值）
    private static class RecordEntry {
        double timestamp;
        String value;

        RecordEntry(double timestamp, String value) {
            this.timestamp = timestamp;
            this.value = value;
        }
    }

    // 存储所有记录的数据结构：键 -> 记录列表（按时间顺序）
    private static final Map<String, List<RecordEntry>> recordMap = new HashMap<>();
    private static final boolean useFileLog = false;
    private static String logPathname;
    
    // 记录键值对（如果与最近一次记录相同则忽略）
    public static void record(String key, String value, boolean allowDuplicate) {
        List<RecordEntry> entries = recordMap.computeIfAbsent(key, k -> new ArrayList<>());
        
        if (!allowDuplicate) {
            // 检查是否与最近一次记录相同
            if (!entries.isEmpty()) {
                RecordEntry lastEntry = entries.get(entries.size() - 1);
                if (lastEntry.value.equals(value)) {
                    return; // 忽略重复记录
                }
            }            
        }
        
        // 添加新记录（带当前时间戳）
        entries.add(new RecordEntry(Timer.getFPGATimestamp(), value));
    }

    // 导出所有记录（按时间戳排序，每行格式：时间戳 key value）
    public static String exportToString() {
        // 收集所有记录（带键信息）
        List<String> lines = new ArrayList<>();
        for (Map.Entry<String, List<RecordEntry>> mapEntry : recordMap.entrySet()) {
            String key = mapEntry.getKey();
            for (RecordEntry entry : mapEntry.getValue()) {
                lines.add(entry.timestamp + "->" + key + ":" + entry.value);
            }
        }
        
        // 按时间戳排序（自然顺序，时间戳小在前）
        lines.sort((line1, line2) -> {
            double ts1 = Double.parseDouble(line1.split("->")[0]);
            double ts2 = Double.parseDouble(line2.split("->")[0]);
            return Double.compare(ts1, ts2);
        });
        
        // 拼接为多行字符串
        return String.join("\n", lines);
    }



    static {
        // 定义日期时间格式
        SimpleDateFormat formatter = new SimpleDateFormat("yyyyMMdd-HHmmss");
        // 获取当前日期时间并格式化
        String currentDateTime = formatter.format(new Date());
        
        logPathname =  "/home/lvuser/" + currentDateTime + "-5515.log";
        if (RobotBase.isSimulation()) {
            logPathname = "./" + currentDateTime + "-5515.log";
        }
    }

    public static boolean init() {
        if (useFileLog) {
        }
        return true;
    }

    public static void flush() {
        if (useFileLog) {
            try {
                // 如果文件不存在则创建空文件
                if (!Files.exists(Paths.get(logPathname))) {
                    Files.createFile(Paths.get(logPathname));
                }
                
                // 使用FileWriter(追加模式)和BufferedWriter写入内容
                try (BufferedWriter writer = new BufferedWriter(
                    new FileWriter(logPathname, true))) {
                    writer.write(exportToString());
                    recordMap.clear();
                }
            } catch (IOException e) {
                System.err.println("SmartDashboardEx 文件操作失败: " + e.getMessage());
            }
            return;
        }
    }

    private static void writeToFile(String key, String value, boolean allowDuplicate) {
        record(key, value, allowDuplicate);
    }


    public static boolean putBoolean(String key, boolean value) {
        return putBoolean(key, value, false, false);
    }

    public static boolean putString(String key, String value) {
        return putString(key, value, false, false);
    }

    public static boolean putNumber(String key, double value) {
        return putNumber(key, value, false, false);
    }

    public static void putData(String key, Sendable value) {
        putData(key, value, false, false);
    }

    public static boolean putBooleanSDOnly(String key, boolean value) {
        if (!useFileLog) {
            return SmartDashboard.putBoolean(key, value);
        }
        return false;
    }

    public static boolean putStringSDOnly(String key, String value) {
        if (!useFileLog) {
            return SmartDashboard.putString(key, value);
        }
        return false;
    }

    public static boolean putNumberSDOnly(String key, double value) {
        if (!useFileLog) {
            return SmartDashboard.putNumber(key, value);
        }
        return false;
    }

    public static void putDataSDOnly(String key, Sendable value) {
        if (!useFileLog) {
            SmartDashboard.putData(key, value);
            return;
        }
    }

    public static boolean putBoolean(String key, boolean value, boolean allowDuplicate, boolean forceUseSmartDashboard) {

        if (useFileLog) {
            writeToFile(key, String.valueOf(value), allowDuplicate);
            if (!forceUseSmartDashboard) {
                return true;
            }
        }

        return SmartDashboard.putBoolean(key, value);
    }

    public static boolean putString(String key, String value, boolean allowDuplicate, boolean forceUseSmartDashboard) {
        if (useFileLog) {
            writeToFile(key, value, allowDuplicate);
            if (!forceUseSmartDashboard) {
                return true;
            }
        }

        return SmartDashboard.putString(key, value);
    }

    public static boolean putNumber(String key, double value, boolean allowDuplicate, boolean forceUseSmartDashboard) {
        if (useFileLog) {
            writeToFile(key, String.valueOf(value), allowDuplicate);
            if (!forceUseSmartDashboard) {
                return true;
            }
        }

        return SmartDashboard.putNumber(key, value);

    }

    public static void putData(String key, Sendable value, boolean allowDuplicate, boolean forceUseSmartDashboard) {
        if (useFileLog) {
            writeToFile(key, value.toString(), allowDuplicate);
            if (!forceUseSmartDashboard) {
                return;
            }
        }

        SmartDashboard.putData(key, value);
    }
}
