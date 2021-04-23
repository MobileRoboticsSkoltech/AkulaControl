package com.example.androidapp;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;

import java.net.SocketException;
import java.net.UnknownHostException;

public class MainActivity extends AppCompatActivity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        NetworkThread ClientThread = new NetworkThread();
        ClientThread.start();

        Joystick Joy = findViewById(R.id.joystick);
        EditText IpLine = findViewById(R.id.ipText);
        EditText PortLine = findViewById(R.id.portText);
        Button RequestButton = findViewById(R.id.requestButton);

        SlidePanel SliderPanel = findViewById(R.id.sliderPanel);

        RequestButton.setOnClickListener(tView -> {
            IpLine.clearFocus();
            PortLine.clearFocus();

            InputMethodManager imm = (InputMethodManager)getSystemService(Context.INPUT_METHOD_SERVICE);
            imm.hideSoftInputFromWindow(tView.getWindowToken(), 0);

            if (!IpLine.getText().toString().matches("") && !PortLine.getText().toString().matches("")) {                                       //---Add validator---//
                try {
                    ClientThread.sendConnectionRequest(IpLine.getText().toString(), Integer.parseInt(PortLine.getText().toString()));
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

        });
    }
}

class NetworkThread extends HandlerThread {
    public NetworkThread() {
        super("Network");
    }

    @Override
    protected void onLooperPrepared() {
        super.onLooperPrepared();

        mHandler = new Handler(getLooper()) {
            @Override
            public void handleMessage(Message tMsg) {
                super.handleMessage(tMsg);

                switch (Header.fromInt(tMsg.what)) {
                    case REQUEST_CONN:
                        mClientUDP.sendRequest();

                        break;
                }
            }
        };
    }

    // Very very dangerous without proper threads finishing
    /// TODO: add restart function to the client
    void sendConnectionRequest(String tIP, int tPort) throws SocketException, UnknownHostException {
        if (mHandler != null) {
            Message Msg = mHandler.obtainMessage();

            if (Msg != null) {
                mClientUDP = new Client(tIP, tPort, 32);

                Msg.what = Header.REQUEST_CONN.getValue();

                mHandler.sendMessage(Msg);
            }
        }
    }

    //----------//

    private Handler         mHandler                = null;
    private Client          mClientUDP              = null;

    private String          mIP;
    private int             mPort;
}