package frc.lib.team2930;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogWriter;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.io.File;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;
import java.util.function.Consumer;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

public class LoggerGroup {
  private static final String logWriter_extraHeader = "AdvantageKit";
  private static final String logWriter_entryMetadata = "{\"source\":\"AdvantageKit\"}";

  private static final double logWriter_timestampUpdateDelay =
      5.0; // Wait several seconds after DS attached to

  // ensure timestamp/timezone is updated
  @SuppressWarnings("unused")
  private static final double logWriter_defaultWritePeriodRio = 0.1;

  @SuppressWarnings("unused")
  private static final double logWriter_defaultWritePeriodSim = 0.01;

  private static final Object g_lock = new Object();
  private static final Map<Class<?>, Struct<?>> structTypeCache;
  private static final List<Struct<?>> structTypePendingSchema;
  private static final List<Struct<?>> structTypePendingSchemaForDataLog;
  private static final List<Struct<?>> structTypeSchemaForDataLog;

  static {
    structTypeCache = new HashMap<>();
    structTypePendingSchema = new ArrayList<>();
    structTypePendingSchemaForDataLog = new ArrayList<>();
    structTypeSchemaForDataLog = new ArrayList<>();
  }

  static int dataLogIdNotInitialized = -2;

  @FunctionalInterface
  public interface EntrySupplier<T extends LoggerEntry> {
    T apply(String name, int updateFrequency);
  }

  private static final String rootKey = "/AdvantageKit";
  public static final String timestampKey = "/Timestamp";

  private static final NetworkTable akitTable;
  private static final IntegerPublisher timestampPublisher;
  private static double currentTimestmap;
  private static long currentTimestmapLog;

  private static DataLog dataLog_handle;
  private static int dataLog_flushDelay;
  private static String dataLog_folder;
  private static String dataLog_filename;
  private static boolean dataLog_syncedDriverStation;
  private static Double dataLog_initialTime;
  private static Double dataLog_dsAttachedTime;
  private static LocalDateTime dataLog_logDate;
  private static Throwable dataLog_failure;

  //

  private static final Object spoolerSignal = new Object();
  private static Thread spoolerThread;

  public static final LoggerGroup root;

  private final String path;
  private LoggerGroup[] groups = new LoggerGroup[0];
  private LoggerEntry[] entries = new LoggerEntry[0];

  static {
    root = new LoggerGroup("");
    akitTable = NetworkTableInstance.getDefault().getTable(rootKey);
    timestampPublisher =
        akitTable.getIntegerTopic(timestampKey.substring(1)).publish(PubSubOption.sendAll(true));
  }

  static Topic getNetworkTableTopic(LoggerEntry entry) {
    return akitTable.getTopic(entry.key);
  }

  static int getDataLogTopic(LoggerEntry entry) {
    if (dataLog_handle != null) {
      return dataLog_handle.start(
          "NT:/" + logWriter_extraHeader + "/" + entry.key,
          entry.getWpiLogType(),
          logWriter_entryMetadata,
          currentTimestmapLog);
    }

    return dataLogIdNotInitialized;
  }

  @SuppressWarnings("unchecked")
  static <T extends StructSerializable> edu.wpi.first.util.struct.Struct<T> findStructType(
      Class<T> classObj) {
    synchronized (g_lock) {
      var res = (edu.wpi.first.util.struct.Struct<T>) structTypeCache.get(classObj);
      if (res == null) {
        try {
          Field field = classObj.getDeclaredField("struct");
          res = (edu.wpi.first.util.struct.Struct<T>) field.get(null);

          structTypePendingSchema.add(res);
        } catch (NoSuchFieldException
            | SecurityException
            | IllegalArgumentException
            | IllegalAccessException ignored) {
        }

        structTypeCache.put(classObj, res);
      }

      return res;
    }
  }

  //

  static void emit(GenericPublisher publisher, byte[] value, int dataLogId) {
    publisher.setRaw(value, currentTimestmapLog);

    if (dataLog_handle != null && dataLogId != LoggerGroup.dataLogIdNotInitialized) {
      dataLog_handle.appendRaw(dataLogId, value, currentTimestmapLog);
    }
  }

  static void emit(GenericPublisher publisher, boolean value, int dataLogId) {
    publisher.setBoolean(value, currentTimestmapLog);

    if (dataLog_handle != null && dataLogId != LoggerGroup.dataLogIdNotInitialized) {
      dataLog_handle.appendBoolean(dataLogId, value, currentTimestmapLog);
    }
  }

  static void emit(GenericPublisher publisher, boolean[] value, int dataLogId) {
    publisher.setBooleanArray(value, currentTimestmapLog);

    if (dataLog_handle != null && dataLogId != LoggerGroup.dataLogIdNotInitialized) {
      dataLog_handle.appendBooleanArray(dataLogId, value, currentTimestmapLog);
    }
  }

  static void emit(GenericPublisher publisher, String value, int dataLogId) {
    publisher.setString(value, currentTimestmapLog);

    if (dataLog_handle != null && dataLogId != LoggerGroup.dataLogIdNotInitialized) {
      dataLog_handle.appendString(dataLogId, value, currentTimestmapLog);
    }
  }

  static void emit(GenericPublisher publisher, String[] value, int dataLogId) {
    publisher.setStringArray(value, currentTimestmapLog);

    if (dataLog_handle != null && dataLogId != LoggerGroup.dataLogIdNotInitialized) {
      dataLog_handle.appendStringArray(dataLogId, value, currentTimestmapLog);
    }
  }

  static void emit(GenericPublisher publisher, long value, int dataLogId) {
    publisher.setInteger(value, currentTimestmapLog);

    if (dataLog_handle != null && dataLogId != LoggerGroup.dataLogIdNotInitialized) {
      dataLog_handle.appendInteger(dataLogId, value, currentTimestmapLog);
    }
  }

  static void emit(GenericPublisher publisher, long[] value, int dataLogId) {
    publisher.setIntegerArray(value, currentTimestmapLog);

    if (dataLog_handle != null && dataLogId != LoggerGroup.dataLogIdNotInitialized) {
      dataLog_handle.appendIntegerArray(dataLogId, value, currentTimestmapLog);
    }
  }

  static void emit(GenericPublisher publisher, float value, int dataLogId) {
    publisher.setFloat(value, currentTimestmapLog);

    if (dataLog_handle != null && dataLogId != LoggerGroup.dataLogIdNotInitialized) {
      dataLog_handle.appendFloat(dataLogId, value, currentTimestmapLog);
    }
  }

  static void emit(GenericPublisher publisher, float[] value, int dataLogId) {
    publisher.setFloatArray(value, currentTimestmapLog);

    if (dataLog_handle != null && dataLogId != LoggerGroup.dataLogIdNotInitialized) {
      dataLog_handle.appendFloatArray(dataLogId, value, currentTimestmapLog);
    }
  }

  static void emit(GenericPublisher publisher, double value, int dataLogId) {
    publisher.setDouble(value, currentTimestmapLog);

    if (dataLog_handle != null && dataLogId != LoggerGroup.dataLogIdNotInitialized) {
      dataLog_handle.appendDouble(dataLogId, value, currentTimestmapLog);
    }
  }

  static void emit(GenericPublisher publisher, double[] value, int dataLogId) {
    publisher.setDoubleArray(value, currentTimestmapLog);

    if (dataLog_handle != null && dataLogId != LoggerGroup.dataLogIdNotInitialized) {
      dataLog_handle.appendDoubleArray(dataLogId, value, currentTimestmapLog);
    }
  }

  //

  public static void setDataLog(String filePath) {
    dataLog_folder = filePath;

    // Reset data
    dataLog_logDate = null;
  }

  private static void refreshLogFilename() {
    if (dataLog_syncedDriverStation) {
      return;
    }

    if (!RobotController.isSystemTimeValid()) {
      return;
    }

    double robotTime = RobotController.getFPGATime() / 1000000.0;

    if (dataLog_initialTime == null) {
      dataLog_initialTime = robotTime;
    }

    if (!dataLog_syncedDriverStation) {
      if (DriverStation.isDSAttached()) {
        if (dataLog_dsAttachedTime == null) {
          dataLog_dsAttachedTime = robotTime;
        } else if (robotTime - dataLog_dsAttachedTime > logWriter_timestampUpdateDelay) {
          dataLog_logDate = LocalDateTime.now();
          dataLog_syncedDriverStation = true;
        } else {
          dataLog_dsAttachedTime = null;
        }
      }

      if (RobotBase.isSimulation()) {
        dataLog_logDate = LocalDateTime.now();
        dataLog_syncedDriverStation = true;
      }
    }

    if (dataLog_logDate == null) {
      dataLog_logDate = LocalDateTime.now();
    }

    // Update filename
    StringBuilder newFilenameBuilder = new StringBuilder();
    newFilenameBuilder.append("Log_");
    var timeFormatter = DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss");
    newFilenameBuilder.append(timeFormatter.format(dataLog_logDate));
    // Update match
    var matchType = DriverStation.getMatchType();
    if (matchType != DriverStation.MatchType.None) {
      String logMatchText =
          switch (matchType) {
            case Practice -> "p";
            case Qualification -> "q";
            case Elimination -> "e";
            default -> "";
          };
      logMatchText += DriverStation.getMatchNumber();
      newFilenameBuilder.append("_");
      newFilenameBuilder.append(logMatchText);
    }

    newFilenameBuilder.append(".wpilog");
    String newFilename = newFilenameBuilder.toString();

    if (!newFilename.equals(dataLog_filename)) {
      System.out.printf("############ LOG FILE UPDATED TO %s\n", newFilename);
      closeLogFile();
      dataLog_filename = newFilename;
    }
  }

  private static void closeLogFile() {
    if (dataLog_handle != null) {
      dataLog_handle.close();
      dataLog_handle = null;

      synchronized (g_lock) {
        root.resetDataLogId();

        // Since we are closing the data log file, we need to resend all the structure schemas.
        structTypePendingSchemaForDataLog.clear();
        structTypePendingSchemaForDataLog.addAll(structTypeSchemaForDataLog);
      }
    }
  }

  private static void ensureLogFile() throws Exception {
    if (dataLog_handle != null) {
      return;
    }

    String dataLogFolder = dataLog_folder;
    if (dataLogFolder == null) {
      return;
    }

    // Create folder if necessary
    File logFolder = new File(dataLogFolder);
    if (!logFolder.exists()) {
      if (!logFolder.mkdirs()) {
        throw new RuntimeException("Unable to create folder " + logFolder.getAbsolutePath());
      }
    }

    // Delete log if it already exists
    File logFile = new File(dataLogFolder, dataLog_filename);
    if (logFile.exists()) {
      if (!logFile.delete()) {
        throw new RuntimeException("Unable to delete file " + logFile.getAbsolutePath());
      }
    }

    // Create new log
    dataLog_handle = new DataLogWriter(logFile.getAbsolutePath(), logWriter_extraHeader);
  }

  public static double getCurrentTimestmap() {
    return currentTimestmap;
  }

  public static void periodic() {
    currentTimestmap = Utils.getCurrentTimeSeconds();
    currentTimestmapLog = HALUtil.getFPGATime();
  }

  public static void publish() {
    timestampPublisher.set(currentTimestmapLog, currentTimestmapLog);

    if (spoolerThread == null) {
      var thread = new Thread(LoggerGroup::runSpooler);
      thread.setName("LoggerSpooler");
      thread.setDaemon(true);
      thread.start();
      spoolerThread = thread;
    }

    synchronized (spoolerSignal) {
      spoolerSignal.notify();
    }
  }

  private static void runSpooler() {
    while (true) {
      try {
        synchronized (spoolerSignal) {
          try {
            spoolerSignal.wait();
          } catch (InterruptedException ignored) {
          }
        }

        refreshLogFilename();
        ensureLogFile();

        if (dataLog_handle != null) {
          // Save timestamp
          dataLog_handle.appendInteger(0, currentTimestmapLog, currentTimestmapLog);
        }

        synchronized (g_lock) {
          if (!structTypePendingSchema.isEmpty()) {
            for (var struct : structTypePendingSchema) {
              publishSchema(struct, new HashSet<>());

              structTypePendingSchemaForDataLog.add(struct);
              structTypeSchemaForDataLog.add(struct);
            }
            structTypePendingSchema.clear();
          }

          if (dataLog_handle != null && !structTypePendingSchemaForDataLog.isEmpty()) {
            for (var struct : structTypePendingSchemaForDataLog) {
              dataLog_handle.addSchema(struct, currentTimestmapLog);
            }
            structTypePendingSchemaForDataLog.clear();
          }
        }

        root.publishInner();

        if (dataLog_handle != null) {
          if (dataLog_flushDelay++ > 50) {
            dataLog_handle.flush();
            dataLog_flushDelay = 0;
          }
        }
      } catch (Throwable e) {
        if (dataLog_failure == null) {
          dataLog_failure = e;
          e.printStackTrace();
        }
      }
    }
  }

  private static void publishSchema(edu.wpi.first.util.struct.Struct<?> struct, Set<String> seen) {
    String typeString = struct.getTypeString();
    if (!seen.add(typeString)) {
      throw new UnsupportedOperationException(typeString + ": circular reference with " + seen);
    }

    var topic = akitTable.getTopic(".schema/" + typeString);

    var publisher = topic.genericPublish("structschema", PubSubOption.sendAll(true));
    publisher.setRaw(struct.getSchema().getBytes(StandardCharsets.UTF_8), currentTimestmapLog);

    for (var inner : struct.getNested()) {
      publishSchema(inner, seen);
    }
    seen.remove(typeString);
  }

  private void resetDataLogId() {
    for (var group : groups) {
      group.resetDataLogId();
    }

    for (var entry : entries) {
      entry.resetDataLogId();
    }
  }

  private void publishInner() {
    for (var group : groups) {
      group.publishInner();
    }

    for (var entry : entries) {
      entry.publishIfNeeded();
    }
  }

  static double shouldRefresh(double lastRefresh, double updateFrequency) {
    if (lastRefresh + updateFrequency <= currentTimestmap) {
      return currentTimestmap + updateFrequency;
    }

    return Double.NaN;
  }

  private LoggerGroup(String path) {
    this.path = path;
  }

  @Override
  public String toString() {
    return path;
  }

  // -- //

  public LoggerEntry.Text buildString(String name) {
    return buildString(name, 50);
  }

  public LoggerEntry.Text buildString(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.Text.class, LoggerEntry.Text::new);
  }

  public LoggerEntry.TextArray buildStringArray(String name) {
    return buildStringArray(name, 50);
  }

  public LoggerEntry.TextArray buildStringArray(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.TextArray.class, LoggerEntry.TextArray::new);
  }

  // -- //

  public <E extends Enum<E>> LoggerEntry.EnumValue<E> buildEnum(String name) {
    return buildEnum(name, 50);
  }

  @SuppressWarnings("unchecked")
  public <E extends Enum<E>> LoggerEntry.EnumValue<E> buildEnum(
      String name, int updateFrequencyInSeconds) {
    //noinspection unchecked
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.EnumValue.class, LoggerEntry.EnumValue::new);
  }

  // -- //

  public LoggerEntry.Condition buildCondition(String name) {
    return new LoggerEntry.Condition(this, name);
  }

  // -- //

  public LoggerEntry.Bool buildBoolean(String name) {
    return buildBoolean(name, 50);
  }

  public LoggerEntry.Bool buildBoolean(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.Bool.class, LoggerEntry.Bool::new);
  }

  // -- //

  public LoggerEntry.BoolArray buildBooleanArray(String name) {
    return buildBooleanArray(name, 50);
  }

  public LoggerEntry.BoolArray buildBooleanArray(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.BoolArray.class, LoggerEntry.BoolArray::new);
  }

  // -- //

  public LoggerEntry.Integer buildInteger(String name) {
    return buildInteger(name, 50);
  }

  public LoggerEntry.Integer buildInteger(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.Integer.class, LoggerEntry.Integer::new);
  }

  public LoggerEntry.IntegerArray buildIntegerArray(String name) {
    return buildIntegerArray(name, 50);
  }

  public LoggerEntry.IntegerArray buildIntegerArray(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name,
        updateFrequencyInSeconds,
        LoggerEntry.IntegerArray.class,
        LoggerEntry.IntegerArray::new);
  }

  // -- //

  public LoggerEntry.DecimalFloat buildDecimalFloat(String name) {
    return buildDecimalFloat(name, 50);
  }

  public LoggerEntry.DecimalFloat buildDecimalFloat(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name,
        updateFrequencyInSeconds,
        LoggerEntry.DecimalFloat.class,
        LoggerEntry.DecimalFloat::new);
  }

  public LoggerEntry.DecimalFloatArray buildDecimalFloatArray(String name) {
    return buildDecimalFloatArray(name, 50);
  }

  public LoggerEntry.DecimalFloatArray buildDecimalFloatArray(
      String name, int updateFrequencyInSeconds) {
    return buildInner(
        name,
        updateFrequencyInSeconds,
        LoggerEntry.DecimalFloatArray.class,
        LoggerEntry.DecimalFloatArray::new);
  }

  // -- //

  public LoggerEntry.Decimal buildDecimal(String name) {
    return buildDecimal(name, 50);
  }

  public LoggerEntry.Decimal buildDecimal(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.Decimal.class, LoggerEntry.Decimal::new);
  }

  public LoggerEntry.DecimalArray buildDecimalArray(String name) {
    return buildDecimalArray(name, 50);
  }

  public LoggerEntry.DecimalArray buildDecimalArray(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name,
        updateFrequencyInSeconds,
        LoggerEntry.DecimalArray.class,
        LoggerEntry.DecimalArray::new);
  }

  // -- //

  public LoggerEntry.Mechanism buildMechanism2d(String name) {
    return buildMechanism2d(name, 50);
  }

  public LoggerEntry.Mechanism buildMechanism2d(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.Mechanism.class, LoggerEntry.Mechanism::new);
  }

  // -- //

  public <T extends StructSerializable> LoggerEntry.Struct<T> buildStruct(
      Class<T> clz, String name) {
    return buildStruct(clz, name, 50);
  }

  @SuppressWarnings("unchecked")
  public <T extends StructSerializable> LoggerEntry.Struct<T> buildStruct(
      Class<T> clz, String name, int updateFrequencyInSeconds) {
    //noinspection unchecked
    return buildInner(
        name,
        updateFrequencyInSeconds,
        LoggerEntry.Struct.class,
        (key, u) -> new LoggerEntry.Struct<>(clz, key, u));
  }

  // -- //

  public <T extends StructSerializable> LoggerEntry.StructArray<T> buildStructArray(
      Class<T> clz, String name) {
    return buildStructArray(clz, name, 50);
  }

  @SuppressWarnings("unchecked")
  public <T extends StructSerializable> LoggerEntry.StructArray<T> buildStructArray(
      Class<T> clz, String name, int updateFrequencyInSeconds) {
    //noinspection unchecked
    return buildInner(
        name,
        updateFrequencyInSeconds,
        LoggerEntry.StructArray.class,
        (key, u) -> new LoggerEntry.StructArray<>(clz, key, u));
  }

  public LoggerEntry.ByteArray buildBytes(String name) {
    return buildBytes(name, 50);
  }

  public LoggerEntry.ByteArray buildBytes(String name, int updateFrequencyInSeconds) {
    return buildInner(
        name, updateFrequencyInSeconds, LoggerEntry.ByteArray.class, LoggerEntry.ByteArray::new);
  }

  // -- //

  private synchronized <T extends LoggerEntry> T buildInner(
      String name, int updateFrequencyInSeconds, Class<T> clz, EntrySupplier<T> supplier) {
    var ptr = this;
    int pos = name.indexOf('/');
    if (pos >= 0) {
      var path = name.substring(0, pos);
      var subName = name.substring(pos + 1);

      if (!path.isEmpty()) {
        ptr = ptr.subgroup(path);
      }

      return ptr.buildInner(subName, updateFrequencyInSeconds, clz, supplier);
    }

    var key = path.isEmpty() ? name : path + "/" + name;
    for (var entry : entries) {
      if (entry.key.equals(key)) {
        return clz.cast(entry);
      }
    }

    var newEntry = supplier.apply(key, updateFrequencyInSeconds);
    newEntry.registerPublisher();
    entries = Arrays.copyOf(entries, entries.length + 1);
    entries[entries.length - 1] = newEntry;
    return newEntry;
  }

  public static LoggerGroup build(String name) {
    return root.subgroup(name);
  }

  public synchronized LoggerGroup subgroup(String name) {
    var ptr = this;
    int pos = name.indexOf('/');
    if (pos >= 0) {
      var path = name.substring(0, pos);
      var subName = name.substring(pos + 1);

      if (!path.isEmpty()) {
        ptr = ptr.subgroup(path);
      }

      return ptr.subgroup(subName);
    }

    var fullPath = path.isEmpty() ? name : path + "/" + name;
    for (var group : groups) {
      if (group.path.equals(fullPath)) {
        return group;
      }
    }

    var newGroup = new LoggerGroup(fullPath);
    groups = Arrays.copyOf(groups, groups.length + 1);
    groups[groups.length - 1] = newGroup;
    return newGroup;
  }

  public static class Redirector implements LogDataReceiver {
    private final HashMap<String, Consumer<LogTable.LogValue>> lookup = new HashMap<>();

    @Override
    public void putTable(LogTable table) {
      // Encode new/changed fields
      var newMap = table.getAll(false);

      for (var field : newMap.entrySet()) {
        // Check if field has changed
        var key = field.getKey();
        var value = field.getValue();

        var callback = lookup.get(key);
        if (callback == null) {
          switch (value.type) {
            case Raw:
              {
                var l = root.buildBytes(key);
                callback = (v) -> l.info(v.getRaw());
                break;
              }

            case Boolean:
              {
                var l = root.buildBoolean(key);
                callback = (v) -> l.info(v.getBoolean());
                break;
              }

            case BooleanArray:
              {
                var l = root.buildBooleanArray(key);
                callback = (v) -> l.info(v.getBooleanArray());
                break;
              }

            case Integer:
              {
                var l = root.buildInteger(key);
                callback = (v) -> l.info(v.getInteger());
                break;
              }

            case IntegerArray:
              {
                var l = root.buildIntegerArray(key);
                callback = (v) -> l.info(v.getIntegerArray());
                break;
              }

            case Float:
              {
                var l = root.buildDecimalFloat(key);
                callback = (v) -> l.info(v.getFloat());
                break;
              }

            case FloatArray:
              {
                var l = root.buildDecimalFloatArray(key);
                callback = (v) -> l.info(v.getFloatArray());
                break;
              }

            case Double:
              {
                var l = root.buildDecimal(key);
                callback = (v) -> l.info(v.getDouble());
                break;
              }

            case DoubleArray:
              {
                var l = root.buildDecimalArray(key);
                callback = (v) -> l.info(v.getDoubleArray());
                break;
              }

            case String:
              {
                var l = root.buildString(key);
                callback = (v) -> l.info(v.getString());
                break;
              }

            case StringArray:
              {
                var l = root.buildStringArray(key);
                callback = (v) -> l.info(v.getStringArray());
                break;
              }

            default:
              // Not supported.
              continue;
          }

          lookup.put(key, callback);
        }

        callback.accept(value);
      }
    }
  }
}
