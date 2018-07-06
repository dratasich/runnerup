/*
 * Copyright (C) 2014 jonas.oreland@gmail.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package org.runnerup.tracker.component;

import android.Manifest;
import android.annotation.TargetApi;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.SystemClock;
import android.preference.PreferenceManager;
import android.support.v4.content.ContextCompat;
import android.util.Log;

import org.runnerup.R;
import org.runnerup.common.tracker.TrackerState;
import org.runnerup.tracker.GpsStatus;
import org.runnerup.tracker.Tracker;
import org.runnerup.tracker.filter.Coordinates;
import org.runnerup.tracker.filter.GPSAccKalmanFilter;
import org.runnerup.tracker.filter.GeoPoint;
import org.runnerup.util.TickListener;

import java.util.ArrayList;
import java.util.Locale;

import static android.location.LocationManager.GPS_PROVIDER;
import static android.location.LocationManager.NETWORK_PROVIDER;
import static android.location.LocationManager.PASSIVE_PROVIDER;

/**
 * Created by jonas on 12/11/14.
 */
@TargetApi(Build.VERSION_CODES.FROYO)
public class TrackerGPS extends DefaultTrackerComponent
        implements TickListener, LocationListener, SensorEventListener {

    /** Logging tag. */
    public static final String TAG = "TrackerGPS";

    private boolean mWithoutGps = false;
    private int frequency_ms = 0;
    private final Tracker tracker;

    private static final String NAME = "GPS";
    private GpsStatus mGpsStatus;
    private Callback mConnectCallback;

    /** Kalman filter fusing GPS and acceleration for a smooth track. */
    private GPSAccKalmanFilter mKalmanFilter;
    /** Filtered last locations used to calculate the bearing/heading of the movement. */
    private Location mLastLocation;
    private Location m2ndLastLocation;
    /** Round speed and acceleration to avoid high precision floating
     * shouldn't be necessary because KF should filter it out - however, for debugging reasons. */
    private static final boolean mRound = true;

    /** Rotation matrix to transform acceleration to the world coordinate system.*/
    private float[] mRinv = new float[16];
    /** Acceleration in world coordinate system. **/
    private float[] mAcceleration = new float[4];
    /** Raw acceleration from accelerometer. **/
    private float[] mAccelerationRaw = new float[3];
    /** Raw magnetic field values. */
    private float[] mGeomagnetic = new float[3];
    /** Estimate of the gravity along x,y,z (gravity removed from the accelerometer values). */
    private float[] mGravity = new float[3];

    /** Sensor types to request from the SensorManager. */
    private static int[] mSensorTypes = {
            Sensor.TYPE_ACCELEROMETER,
            Sensor.TYPE_MAGNETIC_FIELD
            // TODO: API9
            //Sensor.TYPE_LINEAR_ACCELERATION,
            //Sensor.TYPE_ROTATION_VECTOR
    };
    /** Sensors. */
    private ArrayList<Sensor> mSensors = new ArrayList<Sensor>();


    @Override
    public String getName() {
        return NAME;
    }

    public TrackerGPS(Tracker tracker) {
        this.tracker = tracker;
    }

    public boolean startSensors(Context context) {

        return true;
    }

    public void stopSensors(Context context) {

    }

    @Override
    public ResultCode onInit(final Callback callback, Context context) {
        try {
            LocationManager lm = (LocationManager) context.getSystemService(Context.LOCATION_SERVICE);
            if (lm == null) {
                return ResultCode.RESULT_NOT_SUPPORTED;
            }
            if (lm.getProvider(LocationManager.GPS_PROVIDER) == null) {
                return ResultCode.RESULT_NOT_SUPPORTED;
            }

            // init sensors
            SensorManager sm = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
            for (Integer st : mSensorTypes) {
                Sensor sensor = sm.getDefaultSensor(st);
                if (sensor == null) {
                    Log.e(TAG, String.format("No sensor for type %d (skip acceleration data)", st));
                    // strange -- there is no accelerometer or we cannot get the phone's rotation
                    // so let's set acceleration to 0 and let the KF run without it
                    mAcceleration[0] = mAcceleration[1] = mAcceleration[2] = 0;
                    break;
                }
                mSensors.add(sensor);
            }
        } catch (Exception ex) {
            return ResultCode.RESULT_ERROR;
        }
        return ResultCode.RESULT_OK;
    }

    @Override
    public ResultCode onConnecting(final Callback callback, Context context) {
        if (ContextCompat.checkSelfPermission(this.tracker,
                Manifest.permission.ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {
            mWithoutGps = true;
        }
        try {
            // register sensor listeners (accelerometer, geomagnetic)
            SensorManager sm = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
            for (Sensor sensor : mSensors) {
                if (!sm.registerListener(this, sensor, 100000)) { // 100ms
                    Log.e(TAG, String.format("Cannot register listener for sensor type %d", sensor.getType()));
                    break;
                }
            }

            LocationManager lm = (LocationManager) context.getSystemService(Context.LOCATION_SERVICE);
            SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(context);
            frequency_ms = Integer.valueOf(preferences.getString(context.getString(
                    R.string.pref_pollInterval), "2000"));
            if (!mWithoutGps) {
                String frequency_meters = preferences.getString(context.getString(
                        R.string.pref_pollDistance), "5");
                lm.requestLocationUpdates(GPS_PROVIDER,
                        frequency_ms,
                        Integer.valueOf(frequency_meters),
                        this);
                mGpsStatus = new GpsStatus(context);
                mGpsStatus.start(this);
                mConnectCallback = callback;
                return ResultCode.RESULT_PENDING;
            } else {
                String list[] = {
                        GPS_PROVIDER,
                        NETWORK_PROVIDER,
                        PASSIVE_PROVIDER };
                mLastLocation = null;
                for (String s : list) {
                    Location tmp = lm.getLastKnownLocation(s);
                    if (mLastLocation == null || tmp.getTime() > mLastLocation.getTime()) {
                        mLastLocation = tmp;
                    }
                }
                if (mLastLocation != null) {
                    mLastLocation.removeSpeed();
                    mLastLocation.removeAltitude();
                    mLastLocation.removeAccuracy();
                    mLastLocation.removeBearing();
                }
                gpsLessLocationProvider.run();
                return ResultCode.RESULT_OK;
            }
        } catch (Exception ex) {
            return ResultCode.RESULT_ERROR;
        }
    }

    @Override
    public boolean isConnected() {
        return (mWithoutGps) ||
                (mGpsStatus != null) && mGpsStatus.isFixed();
    }

    @Override
    public ResultCode onEnd(Callback callback, Context context) {
        if (!mWithoutGps) {
            LocationManager lm = (LocationManager) context.getSystemService(Context.LOCATION_SERVICE);
            try {
                lm.removeUpdates(this);
            } catch (SecurityException ex) {
                ex.printStackTrace();
            } catch (Exception ex) {
                ex.printStackTrace();
            }

            if (mGpsStatus != null) {
                mGpsStatus.stop(this);
            }
            mGpsStatus = null;
            mConnectCallback = null;
        }

        // unregister sensors
        SensorManager sm = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        for (Sensor sensor : mSensors) {
            sm.unregisterListener(this, sensor);
        }

        return ResultCode.RESULT_OK;
    }

    private final Runnable gpsLessLocationProvider = new Runnable() {

        Location location = null;
        final Handler handler = new Handler();

        @Override
        public void run() {
            if (location == null) {
                location = new Location(mLastLocation);
                mLastLocation = null;
            }
            location.setTime(System.currentTimeMillis());
            switch (tracker.getState()) {
                case INIT:
                case CLEANUP:
                case ERROR:
                    /* end loop be returning directly here */
                    return;
                case INITIALIZING:
                case INITIALIZED:
                case STARTED:
                case PAUSED:
                    /* continue looping */
                    break;
            }
            tracker.onLocationChanged(location);
            handler.postDelayed(this, frequency_ms);
        }
    };

    @Override
    public void onTick() {
        if (mGpsStatus == null)
            return;

        if (!mGpsStatus.isFixed())
            return;

        if (mConnectCallback == null)
            return;

        Callback tmp = mConnectCallback;

        mConnectCallback = null;
        mGpsStatus.stop(this);
        //note: Don't reset mGpsStatus, it's used for isConnected()

        tmp.run(this, ResultCode.RESULT_OK);
    }

    private void logLocation(String description, Location location) {
        String strLocation = String.format(Locale.getDefault(),
                "time=%d, lat=%f, long=%f, alt=%f, acc=%f, bearing=%f, speed=%f",
                location.getTime(),
                location.getLatitude(),
                location.getLongitude(),
                location.getAltitude(),
                location.getAccuracy(),
                location.getBearing(),
                location.getSpeed()
        );
        Log.d(TAG, description + ": " + strLocation);
    }

    /* LocationListener implementation -- start */
    @Override
    public void onLocationChanged(Location location) {
        if (location == null) return;
        // filter as soon as we have location data to let the KF converge
        // (even if workout hasn't started yet)

        logLocation("raw", location);

        // retrieve data from location
        // TODO: use location.getElapsedRealtimeNanos() exact timestamp of the measurement (API26)
        double timestamp = SystemClock.elapsedRealtime(); // monotonic clock (ms)
        double latitude, longitude, x, y, vx, vy, sigma, heading, v;
        longitude = location.getLongitude(); // east/west
        latitude = location.getLatitude(); // north/south
        // the Kalman filter only uses meters (speed is given in meters/second)
        // so we have to transform the coordinates lat/long to y/x
        x = Coordinates.longitudeToMeters(longitude);
        y = Coordinates.latitudeToMeters(latitude);
        // use the accuracy (m) given by the location manager as measurement variance of x/y
        sigma = location.getAccuracy();
        // the location manager also provides us the speed (no need to derive it ;)
        v = location.getSpeed();
        // split speed into x/y direction using the bearing/heading of the movement
        // TODO: estimate heading with KF (see also AP18 TYPE_ROTATION_VECTOR)
        if (location.hasBearing()) // from location
            heading = location.getBearing();
        else if (mLastLocation != null  &&  m2ndLastLocation != null)
            // based on the last filtered locations
            heading = m2ndLastLocation.bearingTo(mLastLocation);
        else
            heading = 0;
        // the KF uses the speed to forward estimate the position (at the next time step)
        vx = v * Math.cos(heading);
        vy = v * Math.sin(heading);
        // TODO: use location.getSpeedAccuracyMetersPerSecond() (API26)
        double v_sigma = sigma * 0.1;

        // initialize when we get the first location
        // or re-initialize if the last location is far from the current one
        if (mKalmanFilter == null ||
                location.distanceTo(mLastLocation) > 2*(sigma + mLastLocation.getAccuracy())) {
            Log.d(TAG, "(re-)initialize KF");
            // initialize the acceleration variance with a high value as we don't provide controls
            mKalmanFilter = new GPSAccKalmanFilter(false,
                    x, y, vx, vy, 1, sigma*sigma, timestamp,
                    1, 1);
        }

        // filter
        // forward estimate the position and velocity
        mKalmanFilter.predict(timestamp, mAcceleration[0], mAcceleration[1]);
        // correct the estimate given the current location
        mKalmanFilter.update(timestamp, x, y, vx, vy, sigma, v_sigma);

        // set filtered location
        Location filteredLocation = new Location(location);
        GeoPoint p = Coordinates.metersToGeoPoint(
                mKalmanFilter.getCurrentX(),
                mKalmanFilter.getCurrentY());
        filteredLocation.setLongitude(p.Longitude);
        filteredLocation.setLatitude(p.Latitude);
        vx = mKalmanFilter.getCurrentXVel();
        vy = mKalmanFilter.getCurrentYVel();
        v = (vx + vy) / 2;
        if (mRound)
            v = Math.round(v * 1000.0) / 1000.0; // avoid high precision floating
        heading = Math.atan2(vy, vx);
        filteredLocation.setSpeed((float) v);
        filteredLocation.setBearing((float) heading);
        tracker.onLocationChanged(filteredLocation);

        logLocation("filtered", filteredLocation);
        mLastLocation = filteredLocation;
        m2ndLastLocation = mLastLocation;
    }

    @Override
    public void onStatusChanged(String s, int i, Bundle bundle) {

    }

    @Override
    public void onProviderEnabled(String s) {

    }

    @Override
    public void onProviderDisabled(String s) {

    }
    /* LocationListener implementation -- end */

    private void logAcceleration(String description, float[] acceleration) {
        assert acceleration.length >= 3;
        long timestamp = SystemClock.elapsedRealtime(); // monotonic clock (ms)
        String strAcceleration = String.format(Locale.getDefault(),
                "time=%d, ax=%f, ay=%f, az=%f", timestamp,
                acceleration[0], acceleration[1], acceleration[2]
        );
        Log.d(TAG, description + ": " + strAcceleration);
    }

    /* SensorEventListener implementation -- start */
    @Override
    public void onSensorChanged(SensorEvent event) {
        float[] R = new float[16];
        float[] linearAcceleration = new float[4];

        switch (event.sensor.getType()) {
            case Sensor.TYPE_MAGNETIC_FIELD:
                System.arraycopy(event.values, 0, mGeomagnetic, 0, event.values.length);
                SensorManager.getRotationMatrix(R, mRinv, mAccelerationRaw, mGeomagnetic);
                break;
            case Sensor.TYPE_ACCELEROMETER:
                System.arraycopy(event.values, 0, mAccelerationRaw, 0, event.values.length);
                //logAcceleration("raw acceleration", mAccelerationRaw);

                // low-pass filter to remove gravity from accelerometer values
                float alpha = 0.8f;

                mGravity[0] = alpha * mGravity[0] + (1 - alpha) * event.values[0];
                mGravity[1] = alpha * mGravity[1] + (1 - alpha) * event.values[1];
                mGravity[2] = alpha * mGravity[2] + (1 - alpha) * event.values[2];

                linearAcceleration[0] = event.values[0] - mGravity[0];
                linearAcceleration[1] = event.values[1] - mGravity[1];
                linearAcceleration[2] = event.values[2] - mGravity[2];
                //logAcceleration("linear acceleration", linearAcceleration);
                // fall through
            case Sensor.TYPE_LINEAR_ACCELERATION:
                // API9
                //System.arraycopy(event.values, 0, linearAcceleration, 0, event.values.length);
                // transform the acceleration to world coordinates: world = inv(R) * linAcc
                android.opengl.Matrix.multiplyMV(mAcceleration, 0, mRinv, 0, linearAcceleration, 0);
                // avoid high precision floating for debugging (acceleration drift?)
                if (mRound) {
                    mAcceleration[0] = Math.round(mAcceleration[0] * 10.0f) / 10.0f;
                    mAcceleration[1] = Math.round(mAcceleration[1] * 10.0f) / 10.0f;
                    mAcceleration[2] = Math.round(mAcceleration[2] * 10.0f) / 10.0f;
                }
                //logAcceleration("world acceleration", mAcceleration);
                break;
            case Sensor.TYPE_ROTATION_VECTOR:
                // API9
                //SensorManager.getRotationMatrixFromVector(R, event.values);
                //android.opengl.Matrix.invertM(Rinv, 0, R, 0);
                break;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
    /* SensorEventListener implementation -- end */
}
