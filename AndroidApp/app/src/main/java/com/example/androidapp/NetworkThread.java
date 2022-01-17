package com.example.androidapp;
//-----------------------------//
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
//-----------------------------//
/**
 * @description
 * The class operates as a connector between Client and UI
 */
class NetworkThread extends HandlerThread {
    public NetworkThread(MainActivity.HandlerUI tHandler) {
        super("Network");
        mHandlerUI = tHandler;
    }

    /**
     * Main function that handles all the messages passed from the Client class and translates
     * them to the UI thread
     */
    @Override
    protected void onLooperPrepared() {
        super.onLooperPrepared();

        mHandler = new Handler(getLooper()) {
            @Override
            public void handleMessage(Message tMsg) {
                super.handleMessage(tMsg);
                Message Msg = mHandler.obtainMessage();

                switch (Header.fromInt(tMsg.what)) {
                    case REQUEST_CONN:
                        mClientUDP.sendRequest();

                        break;
                    case PING:
                        if (!mConnected) {
                            mConnected = true;
                        }

                        Msg.what = Header.PING.getValue();
                        mHandlerUI.sendMessage(Msg);

                        Bundle PingBund = tMsg.getData();
                        byte Stm32State = PingBund.getByte("stm32");
                        byte RecordState = PingBund.getByte("record");
                        byte SensorState = PingBund.getByte("sensor");

                        Message StmMsg = mHandler.obtainMessage();

                        if (Stm32State == 0) {
                            StmMsg.what = Header.STM32_DISCONNECTED.getValue();
                        } else {
                            StmMsg.what = Header.STM32_ONLINE.getValue();
                        }

                        mHandlerUI.sendMessage(StmMsg);

                        //----------//

                        Message RecordMsg = mHandler.obtainMessage();

                        if (RecordState == 0) {
                            RecordMsg.what = Header.RECORD_INACTIVE.getValue();
                        } else {
                            RecordMsg.what = Header.RECORD_ACTIVE.getValue();
                        }

                        mHandlerUI.sendMessage(RecordMsg);

                        //----------//

                        Message SensorMsg = mHandler.obtainMessage();

                        if (RecordState == 0) {
                            SensorMsg.what = Header.SENSOR_INACTIVE.getValue();
                        } else {
                            SensorMsg.what = Header.SENSOR_ACTIVE.getValue();
                        }

                        mHandlerUI.sendMessage(SensorMsg);

                        break;
                    case DISCONNECTED:
                        mConnected = false;

                        Msg.what = Header.DISCONNECTED.getValue();
                        mHandlerUI.sendMessage(Msg);

                        break;
                    case JOYSTICK_COORDS:
                        if (mConnected) {
                            Bundle Bund = tMsg.getData();
                            float[] Coords = Bund.getFloatArray("Coords");

                            mClientUDP.sendCoords(Coords[0], Coords[1]);

                            mCoordsObtainState = false;

                            try {
                                Thread.sleep(20);
                            } catch (InterruptedException tExcept) {
                                tExcept.printStackTrace();
                            }

                            mCoordsObtainState = true;
                        }

                        break;
                    case LATENCY:
                        if (mConnected) {
                            mClientUDP.sendLatencyTest();
                        }

                        break;
                    case TOGGLE_RECORD:
                        if (mConnected) {
                            mClientUDP.sendRecordState();
                        }

                        break;
                    case TOGGLE_SENSOR:
                        if (mConnected) {
                            mClientUDP.sendSensorState();
                        }

                        break;
                    case LATENCY_RESPONSE:
                    case ENCODER:
                        Msg.copyFrom(tMsg);
                        mHandlerUI.sendMessage(Msg);

                        break;
                }
            }
        };

        mClientUDP = new Client(32, 2000, mHandler);
    }

    void close() {
        System.out.println("Closing...");
        if (mClientUDP.isRunning()) {
            mClientUDP.close();
        }
    }

    /**
     * Function tries to establish connection with a server
     * @param tIP Server address
     * @param tPort Server port
     * @throws SocketException Not handled properly
     * @throws UnknownHostException Not handled properly
     */
    void sendConnectionRequest(String tIP, int tPort) throws SocketException, UnknownHostException {
        String IpRange = "(?:0|[1-9]|[1-9][0-9]|1[0-9]?[0-9]|2[0-4][0-9]|25[0-5])";
        Pattern IpPattern = Pattern.compile("^" + IpRange + "\\." + IpRange + "\\." + IpRange + "\\." + IpRange + "$");

        Pattern PortPattern = Pattern.compile("^([1-9]|[1-9][0-9]{1,3}|[1-5][0-9]{4}|6[0-4][0-9]{3}|65[0-4][0-9]{2}|655[0-2][0-9]|6553[0-5])$");

        Matcher IpMatch = IpPattern.matcher(tIP);
        Matcher PortMatch = PortPattern.matcher(String.valueOf(tPort));

        if (!IpMatch.matches() || !PortMatch.matches()) {
            return;
        }

        if (mHandler != null) {
            Message Msg = mHandler.obtainMessage();

            if (Msg != null) {
                if (mClientUDP.isRunning()) {
                    mClientUDP.close();
                }

                mClientUDP.init(tIP, tPort);

                System.out.println("InitRequest");

                Msg.what = Header.REQUEST_CONN.getValue();
                mHandler.sendMessage(Msg);
            }
        }
    }

    void sendJoystickCoords(float tPosX, float tPosY) {
        if (!mCoordsObtainState && (tPosX != 0 || tPosY != 0)) {
            return;
        }

        if (mHandler != null) {
            Message Msg = mHandler.obtainMessage();

            if (Msg != null) {
                Bundle Bund = new Bundle(1);
                float[] Coords = new float[2];

                Coords[0] = tPosX;
                Coords[1] = tPosY;

                Bund.putFloatArray("Coords", Coords);
                Msg.setData(Bund);
                Msg.what = Header.JOYSTICK_COORDS.getValue();

                mHandler.sendMessage(Msg);
            }
        }
    }

    /**
     * Latency test packet goes to the server, then to stm32 and back
     */
    void sendLatencyTest() {
        if (mHandler != null) {
            Message Msg = mHandler.obtainMessage();

            if (Msg != null) {
                Msg.what = Header.LATENCY.getValue();
                mHandler.sendMessage(Msg);
            }
        }
    }
    void sendRecordState() {
        if (mHandler != null) {
            Message Msg = mHandler.obtainMessage();

            if (Msg != null) {
                Msg.what = Header.TOGGLE_RECORD.getValue();
                mHandler.sendMessage(Msg);
            }
        }
    }
    void sendSensorState() {
        if (mHandler != null) {
            Message Msg = mHandler.obtainMessage();

            if (Msg != null) {
                Msg.what = Header.TOGGLE_SENSOR.getValue();
                mHandler.sendMessage(Msg);
            }
        }
    }

    //----------//

    private Handler                             mHandler                = null;
    private final MainActivity.HandlerUI        mHandlerUI;
    private Client                              mClientUDP              = null;

    private boolean                             mConnected              = false;
    private boolean                             mCoordsObtainState      = true;
}