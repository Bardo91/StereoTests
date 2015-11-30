package com.aeroarms.grvc.remoteviewer;

import android.graphics.Bitmap;
import android.os.Bundle;
import android.support.v7.app.ActionBarActivity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;


public class MainActivity extends ActionBarActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mDisplay = (ImageView) findViewById(R.id.display);
        mIpIntput = (EditText) findViewById(R.id.ipInput);
        mConnectButton = (Button) findViewById(R.id.connectButton);

        mConnectButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(mReceiver != null) {
                    mReceiver.stop();
                }
                mReceiver = new ImageReceiver(mIpIntput.getText().toString(), 5098);
            }
        });

        mStopButton = (Button) findViewById(R.id.stopButton);
        mStopButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if(mReceiver != null) {
                    mReceiver.stop();
                }
            }
        });

        Thread displayThread = new Thread(new Runnable() {
            @Override
            public void run() {
                for (;;) {
                    if (mReceiver != null && mReceiver.isConnected()) {
                        Bitmap frame = mReceiver.lastFrame();
                        if (frame == null)
                            continue;

                        int width =  mDisplay.getWidth();
                        int height = width*mReceiver.mHeigth/mReceiver.mWidth;
                        final Bitmap scaled = Bitmap.createScaledBitmap(frame, width, height, true);

                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                mDisplay.setImageBitmap(scaled);
                            }
                        });

                        try {
                            Thread.sleep(10);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }
            }
        });
        displayThread.start();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }

    private EditText mIpIntput;
    private Button  mConnectButton;
    private Button  mStopButton;
    private ImageReceiver mReceiver = null;
    private ImageView mDisplay;
}
