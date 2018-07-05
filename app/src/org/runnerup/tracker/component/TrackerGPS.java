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

import java.util.Locale;

import static android.location.LocationManager.GPS_PROVIDER;
import static android.location.LocationManager.NETWORK_PROVIDER;
import static android.location.LocationManager.PASSIVE_PROVIDER;

/**
 * Created by jonas on 12/11/14.
 */
@TargetApi(Build.VERSION_CODES.FROYO)
public class TrackerGPS extends DefaultTrackerComponent implements TickListener, LocationListener {

    public static final String TAG = "TrackerGPS";

    private boolean mWithoutGps = false;
    private int frequency_ms = 0;
    private Location mLastLocation;
    private Location m2ndLastLocation;
    private final Tracker tracker;

    private static final String NAME = "GPS";
    private GpsStatus mGpsStatus;
    private Callback mConnectCallback;

    private GPSAccKalmanFilter mKalmanFilter;

    @Override
    public String getName() {
        return NAME;
    }

    public TrackerGPS(Tracker tracker) {
        this.tracker = tracker;
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
                    x, y, vx, vy, 0.1, sigma*sigma, timestamp,
                    1, 1);
        }

        // filter
        // forward estimate the position and velocity
        // TODO: get accelerometer data as control input
        mKalmanFilter.predict(timestamp, 0, 0);
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
        v = Math.round(v*1000) / 1000; // avoid high precision floating
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
}
