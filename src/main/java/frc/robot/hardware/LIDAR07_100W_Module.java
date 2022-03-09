package frc.robot.hardware;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;
import java.util.zip.CRC32;

public class LIDAR07_100W_Module {

  I2C m_i2c;

  private static byte LIDAR_DEVICE_ADDRESS = 0x70;

  // TODO, use CRC
  // Docs: https://docs.oracle.com/javase/8/docs/api/java/util/zip/CRC32.html
  CRC32 m_crc32;

  public LIDAR07_100W_Module(Port port) {
    m_i2c = new I2C(port, LIDAR_DEVICE_ADDRESS);
    initLidar();
  }

  public void initLidar() {
    // Want to use continuous measurement mode so that the sensor
    // can read the most amount of data and reduce noise as much as
    // possible.

  }

  public boolean confirmVersion() {

    byte[] buffer =
        new byte[] {0x00, 0x43, 0x00, 0x00, 0x00, 0x00, 0x55, 0x10, (byte) 0xCD, (byte) 0x9a};

    // Read version
    m_i2c.writeBulk(buffer);

    Timer.delay(0.04);

    // TODO, should this be 0x8F? Not sure
    m_i2c.read(LIDAR_DEVICE_ADDRESS, 12, buffer);

    return true;
  }

  public int getDistanceMM() {

    // Example code for a different sensor, unverified
    /*
    byte[] buffer;
    buffer = new byte[2];

    // Read version
    m_i2c.write(0x00, 0x43, 0x00, 0x00, 0x00, 0x00, 0x55, 0x10, 0xCD, 0x9a);
    Timer.delay(0.04);
    m_i2c.read(0x8f, 2, buffer);


    return (int)Integer.toUnsignedLong(buffer[0] &lt;&lt; 8) + Byte.toUnsignedInt(buffer[1]);
    */

    return 0;
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }
}
