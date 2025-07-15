package frc.robot.utils;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public final class MiscUtils {
    public static boolean isAllZero(double[] arr) {
        for (int i = 0; i < arr.length; ++i) {
            double n = arr[i];
            if (Math.abs(n - 0.0) > 1e-6) {
                return false;
            }
        }

        return true;
    }

    public static double EPSILON = 1e-6;
    public static float EPSILONF = 1e-6f;

    public static boolean compareDouble(double a, double b) {
        return Math.abs(a - b) < EPSILON;
    }

    public static boolean compareFloat(float a, float b) {
        return Math.abs(a - b) < EPSILONF;
    }

    public static double calcAngle2Point(double x1, double y1, double x2, double y2) {
        // 假设a和b的坐标
        // double x1 = ...; // 填入a点的x坐标
        // double y1 = ...; // 填入a点的y坐标
        // double x2 = ...; // 填入b点的x坐标
        // double y2 = ...; // 填入b点的y坐标

        // 计算连线ab与x轴正方向的夹角（弧度）
        double angleRad = Math.atan2(y2 - y1, x2 - x1);

        // 将弧度转换为度数
        double angleDeg = Math.toDegrees(angleRad);

        // 如果需要角度在[0, 360)范围内，则进行如下处理（可选）
        if (angleDeg < 0) {
            angleDeg += 360;
        }

        // 输出角度
        System.out.println("calcAngle2Point: " + angleDeg + " degrees");
        return angleDeg;
    }
    
    public static boolean isBeween(double val, double range1, double range2) {
        double min = Math.min(range1, range2);
        double max = Math.max(range1, range2);
        return val >= min && val <= max;
    }

    public static Constants.FieldInfo.APInfo[] getApInfoByAlliance() {
        long[] apIds = DriverStation.getAlliance().get().equals(Alliance.Blue) ? Constants.FieldInfo.blueApIds : Constants.FieldInfo.redApIds;
        // Map<Long, Constants.FieldInfo.APInfo> map = new HashMap<>();
        Constants.FieldInfo.APInfo[] ret = new Constants.FieldInfo.APInfo[apIds.length];
        int index = 0;
        for (long id : apIds) {
            if (Constants.FieldInfo.AP_MAP.containsKey(id)) {
                // map.put(id, Constants.FieldInfo.AP_MAP.get(id));
                ret[index++] = Constants.FieldInfo.AP_MAP.get(id);
            }
        }
        return ret;
    }

    // private static Pose2d calcOffsetPoint(double x, double y, double angle, double n, double offset, double newAngle) {
    //     double angleRad = Units.degreesToRadians(angle); // 将角度转换为弧度

    //     // 计算垂直偏移点
    //     double perpendicularAngleRad = angleRad + Math.PI / 2; // 垂直方向的角度（弧度）
    //     double px = x + n * Math.cos(perpendicularAngleRad); // 垂直偏移点的 x 坐标
    //     double py = y + n * Math.sin(perpendicularAngleRad); // 垂直偏移点的 y 坐标

    //     // 计算沿边偏移后的新点
    //     double newPx = px + offset * Math.cos(angleRad); // 新点的 x 坐标
    //     double newPy = py + offset * Math.sin(angleRad); // 新点的 y 坐标

    //     return new Pose2d(newPx, newPy, new Rotation2d(Units.degreesToRadians(newAngle))); 
    // }

    private static Pose2d calcOffsetPoint(double x, double y, double angle, double hOffset, double vOffset) {
        // String s = String.format("%f, %f, %f, %f, %f", x, y, angle, hOffset, vOffset);
        double radAng = Units.degreesToRadians(angle);
        double sin = Math.sin(radAng);
        double cos = Math.cos(radAng);
        double newPx = x - sin * hOffset - cos * vOffset;
        double newPy = y + cos * hOffset - sin * vOffset;

        return new Pose2d(newPx, newPy, new Rotation2d(radAng)); 
    }

    public static Pose2d getCoralShooterPos(long apId, boolean isLeft) {
        Constants.FieldInfo.APInfo info = Constants.FieldInfo.AP_MAP.get(apId);
        if (info == null) {
            return null;
        };
        
        double hOffset = isLeft ? Constants.FieldInfo.coralBranchOffset : -Constants.FieldInfo.coralBranchOffset;
        double vOffset = Constants.FieldInfo.coralVerticalOffset;
        
        Pose2d offsetPos = calcOffsetPoint(info.getX(), info.getY(), info.getTheta(), hOffset, vOffset);
        return offsetPos;
    }
    
    
    public static Pose2d getCoralBallPos(long apId) {
        Constants.FieldInfo.APInfo info = Constants.FieldInfo.AP_MAP.get(apId);
        if (info == null) {
            return null;
        }
        double hOffset = 0;
        double vOffset = Constants.FieldInfo.agleaVerticalOffset;
        
        Pose2d offsetPos = calcOffsetPoint(info.getX(), info.getY(), info.getTheta(), hOffset, vOffset);
        return offsetPos;
    }

    public static Pose2d getSourcePos(long apId) {
        Constants.FieldInfo.APInfo info = Constants.FieldInfo.AP_MAP.get(apId);
        if (info == null) {
            return null;
        }
        
        double hOffset = Constants.FieldInfo.sourceHorizontalOffset;
        double vOffset = Constants.FieldInfo.sourceVerticalOffset;;
        
        Pose2d offsetPos = calcOffsetPoint(info.getX(), info.getY(), info.getTheta(), hOffset, vOffset);

        return offsetPos;
    }

    public static Pose2d getSourcePosByAlliance(boolean isLeft) {
        long apId = DriverStation.getAlliance().get().equals(Alliance.Blue) ? 
            (isLeft ? 13 : 12) : (isLeft ? 1 : 2);
        
        return getSourcePos(apId);
    }

    public static List<String> listFilesWithSuffix(String directoryPath, String suffix) {
        List<String> matchingFiles = new ArrayList<>();
        File targetDir = new File(directoryPath);
        
        // 验证目录是否存在且可访问
        if (!targetDir.exists() || !targetDir.isDirectory()) {
            System.err.println("错误：目录不存在或不可访问 - " + directoryPath);
            return matchingFiles;
        }
        
        File[] files = targetDir.listFiles();
        if (files == null) {
            System.err.println("错误：无法读取目录内容 - " + directoryPath);
            return matchingFiles;
        }
        
        // 遍历并过滤文件
        for (File file : files) {
            if (file.isFile() && file.getName().toLowerCase().endsWith(suffix.toLowerCase())) {
                matchingFiles.add(file.getAbsolutePath());
            }
        }
        
        return matchingFiles;
    }
}


