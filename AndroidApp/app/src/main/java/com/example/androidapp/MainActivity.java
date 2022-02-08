package com.example.androidapp;
//-----------------------------//
import androidx.appcompat.app.AppCompatActivity;
import androidx.lifecycle.Lifecycle;
import androidx.lifecycle.LifecycleObserver;
import androidx.lifecycle.OnLifecycleEvent;
import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.text.InputFilter;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import java.lang.ref.WeakReference;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
//-----------------------------//
/**
 * Retrieves objects from activity_main.xml, sets colors for all the indicators.
 * Important to notice that network thread is closed when this app is minimized due
 * to safety reasons (control is lost otherwise)
 */
public class MainActivity extends AppCompatActivity {
    class HandlerUI extends Handler {
        HandlerUI() {
            super(Looper.getMainLooper());
        }

        @Override
        public void handleMessage(Message tMsg) {
            super.handleMessage(tMsg);

            System.out.println(Header.fromInt(tMsg.what));

            switch (Header.fromInt(tMsg.what)) {
                case PING:
                    if (!mStateServerLED) {
                        mServerLED.get().setEnableState(true);
                        mStateServerLED = true;
                    }

                    if (!mConnected) {
                        mConnected = true;
                    }

                    break;
                case DISCONNECTED:
                    if (mStateServerLED) {
                        mServerLED.get().setEnableState(false);
                        mStateServerLED = false;
                    }

                    if (mStateStm32LED) {
                        mStm32LED.get().setEnableState(false);
                        mStateStm32LED = false;
                    }

                    if (mStateRecordLED) {
                        mRecordLED.get().setEnableState(false);
                        mStateRecordLED = false;
                    }

                    if (mStateSensorLED) {
                        mSensorLED.get().setEnableState(false);
                        mStateSensorLED = false;
                    }

                    //----------//

                    mSensorsButton.get().setEnabled(false);
                    mRecordButton.get().setEnabled(false);
                    mLatencyButton.get().setEnabled(false);

                    mSensorsButton.get().setAlpha(0.5f);
                    mRecordButton.get().setAlpha(0.5f);
                    mLatencyButton.get().setAlpha(0.5f);

                    mSensorsButtonActive = false;
                    mRecordButtonActive = false;
                    mLatencyButtonActive = false;

                    //----------//

                    mConnected = false;

                    break;
                case STM32_ONLINE:
                    if (!mStateStm32LED) {
                        mStm32LED.get().setEnableState(true);
                        mStateStm32LED = true;
                    }

                    if (!mLatencyButtonActive) {
                        mLatencyButtonActive = true;
                        mLatencyButton.get().setEnabled(true);
                        mLatencyButton.get().setAlpha(1.0f);
                    }

                    break;
                case STM32_DISCONNECTED:
                    if (mStateStm32LED) {
                        mStm32LED.get().setEnableState(false);
                        mStateStm32LED = false;
                    }

                    if (mLatencyButtonActive) {
                        mLatencyButtonActive = false;
                        mLatencyButton.get().setEnabled(false);
                        mLatencyButton.get().setAlpha(0.5f);
                    }

                    break;
                case RECORD_ACTIVE:
                    if (!mStateRecordLED) {
                        mRecordLED.get().setEnableState(true);
                        mStateRecordLED = true;
                    }

                    if (mConnected) {
                        if (!mRecordState && mRecordButton.get().getText() == getString(R.string.RecordButtonStart)) {
                            mRecordButton.get().setText(R.string.RecordButtonStop);
                            mRecordState = true;
                        }

                        if (!mRecordButtonActive) {
                            mRecordButtonActive = true;
                            mRecordButton.get().setAlpha(1.0f);
                            mRecordButton.get().setEnabled(true);
                        }
                    }

                    break;
                case RECORD_INACTIVE:
                    if (mStateRecordLED) {
                        mRecordLED.get().setEnableState(false);
                        mStateRecordLED = false;
                    }

                    if (mConnected) {
                        if (mRecordState && mRecordButton.get().getText() == getString(R.string.RecordButtonStop)) {
                            mRecordButton.get().setText(R.string.RecordButtonStart);
                            mRecordState = false;
                        }

                        if (!mRecordButtonActive) {
                            mRecordButtonActive = true;
                            mRecordButton.get().setAlpha(1.0f);
                            mRecordButton.get().setEnabled(true);
                        }
                    }

                    break;
                case SENSOR_ACTIVE:
                    if (!mStateSensorLED) {
                        mSensorLED.get().setEnableState(true);
                        mStateSensorLED = true;
                    }

                    if (mConnected) {
                        if (!mSensorsState && mSensorsButton.get().getText() == getString(R.string.SensorsButtonStart)) {
                            mSensorsButton.get().setText(R.string.SensorsButtonStop);
                            mSensorsState = true;
                        }

                        if (!mSensorsButtonActive) {
                            mSensorsButtonActive = true;
                            mSensorsButton.get().setAlpha(1.0f);
                            mSensorsButton.get().setEnabled(true);
                        }

                    }

                    break;
                case SENSOR_INACTIVE:
                    if (mStateSensorLED) {
                        mSensorLED.get().setEnableState(false);
                        mStateSensorLED = false;
                    }

                    if (mConnected) {
                        if (mSensorsState && mSensorsButton.get().getText() == getString(R.string.SensorsButtonStop)) {
                            mSensorsButton.get().setText(R.string.SensorsButtonStart);
                            mSensorsState = false;
                        }

                        if (!mSensorsButtonActive) {
                            mSensorsButtonActive = true;
                            mSensorsButton.get().setAlpha(1.0f);
                            mSensorsButton.get().setEnabled(true);
                        }
                    }

                    break;
                case TOGGLE_RECORD:
                    mRecordButtonActive = false;
                    mRecordButton.get().setEnabled(false);
                    mRecordButton.get().setAlpha(0.5f);

                    break;
                case TOGGLE_SENSOR:
                    mSensorsButtonActive = false;
                    mSensorsButton.get().setEnabled(false);
                    mSensorsButton.get().setAlpha(0.5f);

                    break;
                case LATENCY_RESPONSE:
                    Bundle Bund = tMsg.getData();
                    double Latency = Bund.getDouble("Latency");
                    LatencyText.get().setText(String.valueOf(Latency));

                    break;
                case ENCODER:
                    Bundle EncoderBund = tMsg.getData();
                    long LeftEncoder = EncoderBund.getLong("LeftEncoder");
                    long RightEncoder = EncoderBund.getLong("RightEncoder");

                    LeftEncoderText.get().setText(String.valueOf(LeftEncoder));
                    RightEncoderText.get().setText(String.valueOf(RightEncoder));

                    break;
            }
        }

        void setServerLED(ConnIndicator tLED) {
            mServerLED = new WeakReference <>(tLED);
        }
        void setStmLED(ConnIndicator tLED) {
            mStm32LED = new WeakReference <>(tLED);
        }
        void setRecordLED(ConnIndicator tLED) {
            mRecordLED = new WeakReference <>(tLED);
        }
        void setSensorLED(ConnIndicator tLED) {
            mSensorLED = new WeakReference <>(tLED);
        }

        //----------//

        void setLatencyButton(Button tButton) {
            mLatencyButton = new WeakReference <>(tButton);
        }
        void setRecordButton(Button tButton) {
            mRecordButton = new WeakReference <>(tButton);
        }
        void setSensorsButton(Button tButton) {
            mSensorsButton = new WeakReference <>(tButton);
        }

        //----------//

        void setLatencyText(TextView tView) {
            LatencyText = new WeakReference <>(tView);
        }

        void setLeftEncoderText(TextView tView) {
            LeftEncoderText = new WeakReference <>(tView);
        }
        void setRightEncoderText(TextView tView) {
            RightEncoderText = new WeakReference <>(tView);
        }

        //----------//

        private boolean                         mConnected              = false;

        //----------//

        private boolean                         mStateServerLED         = false;
        private WeakReference <ConnIndicator>   mServerLED;

        private boolean                         mStateStm32LED          = false;
        private WeakReference <ConnIndicator>   mStm32LED;

        private boolean                         mStateRecordLED         = false;
        private WeakReference <ConnIndicator>   mRecordLED;

        private boolean                         mStateSensorLED         = false;
        private WeakReference <ConnIndicator>   mSensorLED;

        //----------//

        private boolean                         mLatencyButtonActive    = false;
        private WeakReference <Button>          mLatencyButton;

        private boolean                         mRecordButtonActive     = false;
        private boolean                         mRecordState            = false;
        private WeakReference <Button>          mRecordButton;

        private boolean                         mSensorsButtonActive    = false;
        private boolean                         mSensorsState           = false;
        private WeakReference <Button>          mSensorsButton;

        //----------//

        private WeakReference <TextView>        LatencyText;

        private WeakReference <TextView>        LeftEncoderText;
        private WeakReference <TextView>        RightEncoderText;
    }

    //----------//

    public class MinimizeObserver implements LifecycleObserver {
        @OnLifecycleEvent(Lifecycle.Event.ON_STOP)
        void onAppBackgrounded() {
            mNetworkThread.close();
        }
    }

    //----------//

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getLifecycle().addObserver(new MinimizeObserver());

        InputFilter IpFilter = (tSrc, tSrcStart, tSrcEnd, tDest, tDestStart, tDestEnd) -> {
            String IpRange = "(?:0|[1-9]|[1-9][0-9]|1[0-9]?[0-9]|2[0-4][0-9]|25[0-5])";
            Pattern IpPattern = Pattern.compile("^" + IpRange + "(\\.(" + IpRange + "(\\.(" + IpRange + "(\\.(" + IpRange + ")?)?)?)?)?)?$");

            String destTxt = tDest.toString();
            String resultingTxt = destTxt.substring(0, tDestStart) + tSrc.subSequence(tSrcStart, tSrcEnd) + destTxt.substring(tDestEnd);

            Matcher Match = IpPattern.matcher(resultingTxt);

            if (!Match.matches()) {
                return "";
            }

            return null;
        };

        InputFilter PortFilter = (tSrc, tSrcStart, tSrcEnd, tDest, tDestStart, tDestEnd) -> {
            Pattern PortPattern = Pattern.compile("^([1-9]|[1-9][0-9]{1,3}|[1-5][0-9]{4}|6[0-4][0-9]{3}|65[0-4][0-9]{2}|655[0-2][0-9]|6553[0-5])$");

            String destTxt = tDest.toString();
            String resultingTxt = destTxt.substring(0, tDestStart) + tSrc.subSequence(tSrcStart, tSrcEnd) + destTxt.substring(tDestEnd);

            Matcher Match = PortPattern.matcher(resultingTxt);

            if (!Match.matches()) {
                return "";
            }

            return null;
        };

        Joystick Joy = findViewById(R.id.joystick);
        EditText IpLine = findViewById(R.id.ipText);
        EditText PortLine = findViewById(R.id.portText);
        Button RequestButton = findViewById(R.id.requestButton);

        //----------//

        Button SensorButton = findViewById(R.id.sensorsButton);
        Button RecordButton = findViewById(R.id.recordButton);
        Button LatencyButton = findViewById(R.id.latencyButton);

        SensorButton.setEnabled(false);
        RecordButton.setEnabled(false);
        LatencyButton.setEnabled(false);

        SensorButton.setAlpha(0.5f);
        RecordButton.setAlpha(0.5f);
        LatencyButton.setAlpha(0.5f);

        //----------//

        SlidePanel SliderPanel = findViewById(R.id.sliderPanel);

        IpLine.setFilters(new InputFilter[] {IpFilter});
        PortLine.setFilters(new InputFilter[] {PortFilter});

        ConnIndicator StmIndicator = findViewById(R.id.stm32Indicator);
        StmIndicator.setEnableColor(200, 200, 0);
        StmIndicator.setDisableColor(100, 100, 0);
        StmIndicator.setBackgroundColor(50, 50, 0);

        ConnIndicator RecordIndicator = findViewById(R.id.recordIndicator);
        RecordIndicator.setEnableColor(200, 0, 0);
        RecordIndicator.setDisableColor(100, 0, 0);
        RecordIndicator.setBackgroundColor(50, 0, 0);

        ConnIndicator SensorIndicator = findViewById(R.id.sensorsIndicator);
        SensorIndicator.setEnableColor(0, 0, 255);
        SensorIndicator.setDisableColor(0, 0, 150);
        SensorIndicator.setBackgroundColor(0, 0, 70);

        mHandler.setServerLED(findViewById(R.id.serverIndicator));
        mHandler.setStmLED(StmIndicator);
        mHandler.setRecordLED(RecordIndicator);
        mHandler.setSensorLED(SensorIndicator);

        mHandler.setLatencyButton(LatencyButton);
        mHandler.setRecordButton(RecordButton);
        mHandler.setSensorsButton(SensorButton);

        mHandler.setLatencyText(findViewById(R.id.latencyValue));

        mHandler.setLeftEncoderText(findViewById(R.id.leftEncoder));
        mHandler.setRightEncoderText(findViewById(R.id.rightEncoder));

        RequestButton.setOnClickListener(tView -> {
            IpLine.clearFocus();
            PortLine.clearFocus();

            InputMethodManager imm = (InputMethodManager)getSystemService(Context.INPUT_METHOD_SERVICE);
            imm.hideSoftInputFromWindow(tView.getWindowToken(), 0);

            ///---TODO: Add validator---///
            if (!IpLine.getText().toString().matches("") && !PortLine.getText().toString().matches("")) {
                try {
                    mNetworkThread.sendConnectionRequest(IpLine.getText().toString(), Integer.parseInt(PortLine.getText().toString()));
                } catch (SocketException | UnknownHostException e) {
                    e.printStackTrace();
                }

                SliderPanel.smoothSlideTo(0.0f);
            }
        });

        SliderPanel.setCustomObjectListener(() -> {
            IpLine.clearFocus();
            PortLine.clearFocus();

            SliderPanel.closeKeyboard();
        });

        Joy.setCustomObjectListener((tPosX, tPosY, tID) -> {
            mNetworkThread.sendJoystickCoords(tPosX, tPosY);
        });

        LatencyButton.setOnClickListener(tView -> {
            mNetworkThread.sendLatencyTest();
        });

        RecordButton.setOnClickListener(tView -> {
            mNetworkThread.sendRecordState();
        });

        SensorButton.setOnClickListener(tView -> {
            mNetworkThread.sendSensorState();
        });
    }
    @Override
    protected void onStart() {
        super.onStart();

        mNetworkThread = new NetworkThread(mHandler);
        mNetworkThread.start();
    }

    //----------//

    HandlerUI       mHandler            = new HandlerUI();
    NetworkThread   mNetworkThread;
}


