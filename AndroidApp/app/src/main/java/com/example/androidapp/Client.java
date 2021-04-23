package com.example.androidapp;

import java.io.IOException;
import java.net.*;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;
//-----------------------------//
public class Client {
    Client(String tAddress, int tPort, int tPacketSize) throws SocketException, UnknownHostException {
        mAddress        = InetAddress.getByName(tAddress);
        mPort           = tPort;
        mPacketSize     = tPacketSize;

        mSocket         = new DatagramSocket();

        mReadBuffer     = new byte[tPacketSize];
        mWriteBuffer    = new byte[tPacketSize];

        //----------//

        mSocket.setSoTimeout(1000);

        mReadThread.start();
        mWriteThread.start();
        mProcessThread.start();

        mTimerThread.start();
    }

    //----------//

    public void sendRequest() {
        byte[] Packet = new byte[mPacketSize];

        Header Tag = Header.REQUEST_CONN;
        byte[] Zeros = new byte[mPacketSize - 4];

        ByteBuffer Buffer = ByteBuffer.wrap(Packet);

        Buffer.putInt(Tag.getValue());
        Buffer.put(Zeros);

        fillWriteBuffer(Packet);
    }

    //----------//

    void readFunc() {
        byte[]              Packet      = new byte[mPacketSize];
        DatagramPacket      PacketUDP   = new DatagramPacket(Packet, Packet.length);
        boolean             NewData     = false;

        while (!mTerminate.get()) {
            if (mTerminate.get()) {
                ///---TODO: Add proper termination handling---///
            }

            while (!NewData) {
                try {
                    mSocket.receive(PacketUDP);
                    NewData = true;
                } catch (SocketTimeoutException tExcept) {
                    System.err.println("readFunc: receive timeout!");
                } catch (IOException tExcept) {
                    System.err.println("readFunc: receive exception!");
                }
            }

            System.out.println("Ping!");

            fillReadBuffer(Packet);
            NewData = false;
        }
    }
    void writeFunc() {
        byte[] Packet = new byte[mPacketSize];

        while (!mTerminate.get()) {
            synchronized (mWriteMonitor) {
                while (!mWriteState && !mTerminate.get()) {
                    try {
                        mWriteMonitor.wait();
                    } catch (InterruptedException tExcept)  {
                        Thread.currentThread().interrupt();
                        System.err.println("writeFunc: thread interrupted!");
                    }
                }

                if (mTerminate.get()) {
                    ///---TODO: Add proper termination handling---///
                }

                getWriteBuffer(Packet);

                DatagramPacket PacketUDP = new DatagramPacket(Packet, Packet.length, mAddress, mPort);

                try {
                    mSocket.send(PacketUDP);
                } catch (IOException tExcept) {
                    System.err.println("writeFunc: send exception!");
                }

                mWriteState = false;
            }
        }
    }
    void processFunc() {
        byte[] Packet = new byte[mPacketSize];
        Header Tag;

        while (!mTerminate.get()) {
            synchronized (mProcessMonitor) {
                while (!mProcessState && !mTerminate.get()) {
                    try {
                        mProcessMonitor.wait();
                    } catch (InterruptedException tExcept)  {
                        Thread.currentThread().interrupt();
                        System.err.println("writeFunc: thread interrupted!");
                    }
                }

                if (mTerminate.get()) {
                    ///---TODO: Add proper termination handling---///
                }

                getReadBuffer(Packet);

                ByteBuffer Buffer = ByteBuffer.wrap(Packet);

                Tag = Header.fromInt(Integer.reverseBytes(Buffer.getInt()));

                switch (Tag) {
                    case REQUEST_CONN:
                    case JOYSTICK_COORDS:
                        break;
                    case PING:
                        byte[] PingPacket = new byte[mPacketSize];

                        Header PingTag = Header.PING;
                        byte[] Zeros = new byte[mPacketSize - 4];

                        ByteBuffer PingBuffer = ByteBuffer.wrap(PingPacket);

                        PingBuffer.putInt(Integer.reverseBytes(PingTag.getValue()));
                        PingBuffer.put(Zeros);

                        fillWriteBuffer(PingPacket);

                        break;
                    case STATUS:
                        break;
                    case INVALID:
                }

                mProcessState = false;
            }
        }
    }

    //----------//

    boolean fillReadBuffer(byte[] tBuffer) {
        synchronized (mReadBufferMonitor) {
            while (mReadBufferState && !mTerminate.get()) {
                try {
                    mReadBufferMonitor.wait();
                } catch (InterruptedException tExcept)  {
                    Thread.currentThread().interrupt();
                    System.err.println("fillReadBuffer: thread interrupted!");
                }
            }

            if (mTerminate.get()) {
                return false;
            }

            System.arraycopy(tBuffer, 0, mReadBuffer, 0, mPacketSize);

            synchronized (mProcessMonitor) {
                mProcessState = true;
                mProcessMonitor.notify();
            }

            mReadBufferState = true;
            mReadBufferMonitor.notifyAll();
        }

        return true;
    }
    boolean getReadBuffer(byte[] tBuffer) {
        synchronized (mReadBufferMonitor) {
            while (!mReadBufferState && !mTerminate.get()) {
                try {
                    mReadBufferMonitor.wait();
                } catch (InterruptedException tExcept)  {
                    Thread.currentThread().interrupt();
                    System.err.println("fillReadBuffer: thread interrupted!");
                }
            }

            if (mTerminate.get()) {
                return false;
            }

            System.arraycopy(mReadBuffer, 0, tBuffer, 0, mPacketSize);

            mReadBufferState = false;
            mReadBufferMonitor.notifyAll();
        }

        return true;
    }

    boolean fillWriteBuffer(byte[] tBuffer) {
        synchronized (mWriteBufferMonitor) {
            while (mWriteBufferState && !mTerminate.get()) {
                try {
                    mWriteBufferMonitor.wait();
                } catch (InterruptedException tExcept)  {
                    Thread.currentThread().interrupt();
                    System.err.println("fillWriteBuffer: thread interrupted!");
                }
            }

            if (mTerminate.get()) {
                return false;
            }

            System.arraycopy(tBuffer, 0, mWriteBuffer, 0, mPacketSize);

            synchronized (mWriteMonitor) {
                mWriteState = true;
                mWriteMonitor.notify();
            }

            mWriteBufferState = true;
            mWriteBufferMonitor.notifyAll();
        }

        return true;
    }
    boolean getWriteBuffer(byte[] tBuffer) {
        synchronized (mWriteBufferMonitor) {
            while (!mWriteBufferState && !mTerminate.get()) {
                try {
                    mWriteBufferMonitor.wait();
                } catch (InterruptedException tExcept)  {
                    Thread.currentThread().interrupt();
                    System.err.println("fillWriteBuffer: thread interrupted!");
                }
            }

            if (mTerminate.get()) {
                return false;
            }

            System.arraycopy(mWriteBuffer, 0, tBuffer, 0, mPacketSize);

            mWriteBufferState = false;
            mWriteBufferMonitor.notifyAll();
        }

        return true;
    }

    //----------//

    void timerFunc() {
        System.out.println("Timer");
    }

    //----------//

    private AtomicBoolean       mTerminate                  = new AtomicBoolean(false);
    private AtomicBoolean       mConnected                  = new AtomicBoolean(false);

    //----------//

    private DatagramSocket      mSocket;

    private final int           mPort;
    private final InetAddress   mAddress;

    //----------//

    private Integer             mPacketSize;

    private byte[]              mReadBuffer;
    private byte[]              mWriteBuffer;

    //----------//

    private final Thread        mReadThread                 = new Thread(this::readFunc);
    private final Thread        mWriteThread                = new Thread(this::writeFunc);
    private final Thread        mProcessThread              = new Thread(this::processFunc);

    private final Thread        mTimerThread                = new Thread(this::timerFunc);

    private final Object        mWriteMonitor               = new Object();
    private final Object        mProcessMonitor             = new Object();

    private boolean             mWriteState                 = false;
    private boolean             mProcessState               = false;

    //----------//

    private final Object        mReadBufferMonitor          = new Object();
    private final Object        mWriteBufferMonitor         = new Object();

    private boolean             mReadBufferState            = false;
    private boolean             mWriteBufferState           = false;
}
