package com.example.wcook.androidusb;

import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;
import android.widget.TextView;
import android.widget.Button;
import android.widget.ScrollView;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.WindowManager;
import android.util.Log;


import com.hoho.android.usbserial.driver.CdcAcmSerialDriver;
import com.hoho.android.usbserial.driver.ProbeTable;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static android.graphics.Color.rgb;
import static java.lang.Math.abs;

public class MainActivity extends AppCompatActivity implements TextureView.SurfaceTextureListener  {
    SeekBar myControl;
    TextView myTextView;
    Button button;
    TextView myTextView2;
    ScrollView myScrollView;
    TextView myTextView3;
    private Camera mCamera;
    private TextureView mTextureView;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView fpsTextView;
    public int thresh = 0;


    public double xWeight = 1.0;
    public double yWeight = .02;
    static long prevtime = 0; // for FPS calculation
    private UsbManager manager;
    private UsbSerialPort sPort;
    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();
    private SerialInputOutputManager mSerialIoManager;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); // keeps the screen from turning off
        myControl = (SeekBar) findViewById(R.id.seek1);

        fpsTextView = (TextView) findViewById(R.id.textView01);
        fpsTextView.setText("Hello Senior Lecturer Nicholas");

        myTextView2 = (TextView) findViewById(R.id.textView02);
        myScrollView = (ScrollView) findViewById(R.id.ScrollView01);
        myTextView3 = (TextView) findViewById(R.id.textView03);
        button = (Button) findViewById(R.id.button1);
        Log.w("Myapp", "Before requesting Permissions ");
        //ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.CAMERA}, 1);
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            mSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
            mSurfaceHolder = mSurfaceView.getHolder();

            mTextureView = (TextureView) findViewById(R.id.textureview);
            mTextureView.setSurfaceTextureListener(this);

            // set the paintbrush for writing text on the image
            paint1.setColor(0xffff0000); // red
            paint1.setTextSize(24);

            fpsTextView.setText("started camera");

            Log.w("Myapp", "Started Camera");
        } else {
            fpsTextView.setText("no camera permissions");

            Log.w("Myapp", "No camera :( ");
        }

        Log.w("Myapp", "After checking Permissions ");
        button.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                myTextView2.setText("value on click is "+myControl.getProgress());
                /*String sendString = String.valueOf(myControl.getProgress()) + '\n';
                try {
                    sPort.write(sendString.getBytes(), 10); // 10 is the timeout
                } catch (IOException e) { }*/
            }
        });
        myControl.setMax(100); // maybe 255 is better? consider changing
        manager = (UsbManager) getSystemService(Context.USB_SERVICE);

        Log.w("Myapp", "After USB manager ");
        setMyControlListener();

        Log.w("Myapp", "After control listener ");
    }
    private void setMyControlListener() {
        myControl.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            int progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {

                Log.w("Myapp", "Progress change");
                progressChanged = progress;

                thresh = progress;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

                Log.w("Myapp", "Start touch tracking ");
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }
    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {

        Log.w("Myapp", "Surface texture available ");
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        parameters.setPreviewSize(640, 480);
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY); // no autofocusing
        parameters.setAutoExposureLock(true); // keep the white balance constant
        mCamera.setParameters(parameters);
        mCamera.setDisplayOrientation(90); // rotate to portrait mode

        try {
            mCamera.setPreviewTexture(surface);
            mCamera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened

            Log.w("Myapp", "Camera didnt' start preview");
        }
    }

    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        mCamera.release();
        return true;
    }

    // the important function
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // every time there is a new Camera preview frame
        mTextureView.getBitmap(bmp);
        float xMiddle = 0;
        float yMiddle = 0;
        int numPixels = 0;
        Log.w("Myapp", "Starting on Surface texture updated ");
        final Canvas c = mSurfaceHolder.lockCanvas();
        if (c != null) {
            //int thresh = 0; // comparison value
            int[] pixels = new int[bmp.getWidth()]; // pixels[] is the RGBA data
            //int startY = 100; // which row in the bitmap to analyze to read
            int rowGap = 7; // how many rowth to be at
            int colGap = 2;
            // really should do like 0 to bmp.getHeight() at some point, but would probably have to raise row gap

            for(int startY = 0; startY < bmp.getHeight() / 2 ; startY += rowGap) {
                bmp.getPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);

                // in the row, see if there is more green than red
                for (int i = 0; i < bmp.getWidth(); i+= colGap) {
                    double avg = (green (pixels[i]) + red(pixels[i]) + blue(pixels[i])) / 3.0;
                    int sum =0 ;
                    sum += abs(green(pixels[i]) - avg);
                    sum += abs(red(pixels[i]) - avg);
                    sum += abs(blue(pixels[i]) - avg);
                    //if ((green(pixels[i]) - red(pixels[i])) > thresh) {
                    if (sum <= thresh && (red(pixels[i]) > 100) && ((green(pixels[i]) + red(pixels[i]) + blue(pixels[i])) < 740)){
                        pixels[i] = rgb(0, 255, 0); // over write the pixel with pure green
                        xMiddle += i;
                        yMiddle += startY;
                        ++numPixels;
                    }
                }

                // update the row
                bmp.setPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);
            }

        }

        // draw a circle at some position
        int pos = 50;
        xMiddle /= numPixels;
        yMiddle /= numPixels;
        canvas.drawCircle(xMiddle, yMiddle, 5, paint1); // x position, y position, diameter, color

        // write the pos as text
        canvas.drawText("xpos = " + xMiddle + " ypos = " + yMiddle, 10, 200, paint1);
        double xDiff = (bmp.getWidth() / 2)  - xMiddle;
        double yDiff = (bmp.getHeight() / 2) - yMiddle;
        int pwmDiff = (int)(xDiff * xWeight + yDiff * yWeight);
        String usbSend = String.valueOf(pwmDiff) + '\n';

       try {
            sPort.write(usbSend.getBytes(), 10); // 10 is the timeout
        } catch (IOException e) { }
        c.drawBitmap(bmp, 0, 0, null);
        mSurfaceHolder.unlockCanvasAndPost(c);

        // calculate the FPS to see how fast the code is running
        long nowtime = System.currentTimeMillis();
        long diff = nowtime - prevtime;
        //fpsTextView.setText("FPS " + 1000 / diff);
        fpsTextView.setText(usbSend);
        prevtime = nowtime;

        Log.w("Myapp", "Got through surface update ");
    }
    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {
                @Override
                public void onRunError(Exception e) {

                }

                @Override
                public void onNewData(final byte[] data) {
                    MainActivity.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            MainActivity.this.updateReceivedData(data);
                        }
                    });
                }
            };

    @Override
    protected void onPause(){
        super.onPause();
        stopIoManager();
        if(sPort != null){
            try{
                sPort.close();
            } catch (IOException e){ }
            sPort = null;
        }
        finish();
    }

    @Override
    protected void onResume() {
        super.onResume();

        ProbeTable customTable = new ProbeTable();
        customTable.addProduct(0x04D8,0x000A, CdcAcmSerialDriver.class);
        UsbSerialProber prober = new UsbSerialProber(customTable);

        final List<UsbSerialDriver> availableDrivers = prober.findAllDrivers(manager);

        if(availableDrivers.isEmpty()) {
            //check
            return;
        }

        UsbSerialDriver driver = availableDrivers.get(0);
        sPort = driver.getPorts().get(0);

        if (sPort == null){
            //check
        }else{
            final UsbManager usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
            UsbDeviceConnection connection = usbManager.openDevice(driver.getDevice());
            if (connection == null){
                //check
                PendingIntent pi = PendingIntent.getBroadcast(this, 0, new Intent("com.android.example.USB_PERMISSION"), 0);
                usbManager.requestPermission(driver.getDevice(), pi);
                return;
            }

            try {
                sPort.open(connection);
                sPort.setParameters(9600, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);

            }catch (IOException e) {
                //check
                try{
                    sPort.close();
                } catch (IOException e1) { }
                sPort = null;
                return;
            }
        }
        onDeviceStateChange();
    }

    private void stopIoManager(){
        if(mSerialIoManager != null) {
            mSerialIoManager.stop();
            mSerialIoManager = null;
        }
    }

    private void startIoManager() {
        if(sPort != null){
            mSerialIoManager = new SerialInputOutputManager(sPort, mListener);
            mExecutor.submit(mSerialIoManager);
        }
    }

    private void onDeviceStateChange(){
        stopIoManager();
        startIoManager();
    }

    private void updateReceivedData(byte[] data) {
        //do something with received data

        //for displaying:
        String rxString = null;
        try {
            rxString = new String(data, "UTF-8"); // put the data you got into a string
            myTextView3.append(rxString);
            myScrollView.fullScroll(View.FOCUS_DOWN);
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        }
    }

}
