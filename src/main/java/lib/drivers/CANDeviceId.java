package lib.drivers;

// Credit 254
public class CANDeviceId {

    private final int mDeviceNumber;
    private final String mBus;

    public CANDeviceId(int deviceNumber, String bus) {
        mDeviceNumber = deviceNumber;
        mBus = bus;
    }

    public CANDeviceId(int deviceNumber) {
        this(deviceNumber, "");
    }

    public int getDeviceNumber() { return mDeviceNumber; }

    public String getBus() { return mBus; }

    public boolean equals(CANDeviceId other) {
        return other.mDeviceNumber == mDeviceNumber && other.mBus == mBus;
    }
    
}
