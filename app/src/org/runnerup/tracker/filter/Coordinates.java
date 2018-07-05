/*
Copyright (c) 2017-2018 Mad Devs

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Extracted from https://github.com/maddevsio/mad-location-manager
*/

package org.runnerup.tracker.filter;


/**
 * Created by lezh1k on 2/13/18.
 */


public class Coordinates {
    private static final double EARTH_RADIUS = 6371.0 * 1000.0; // meters

    public static double distanceBetween(double lon1, double lat1, double lon2, double lat2) {
        double deltaLon = Math.toRadians(lon2 - lon1);
        double deltaLat = Math.toRadians(lat2 - lat1);
        double a =
                Math.pow(Math.sin(deltaLat / 2.0), 2.0) +
                        Math.cos(Math.toRadians(lat1)) *
                                Math.cos(Math.toRadians(lat2)) *
                                Math.pow(Math.sin(deltaLon/2.0), 2.0);
        double c = 2.0 * Math.atan2(Math.sqrt(a), Math.sqrt(1.0-a));
        return EARTH_RADIUS * c;
    }

    public static double longitudeToMeters(double lon) {
        double distance = distanceBetween(lon, 0.0, 0.0, 0.0);
        return distance * (lon < 0.0 ? -1.0 : 1.0);
    }

    public static double latitudeToMeters(double lat) {
        double distance = distanceBetween(0.0, lat, 0.0, 0.0);
        return distance * (lat < 0.0 ? -1.0 : 1.0);
    }

    public static GeoPoint metersToGeoPoint(double lonMeters,
                                            double latMeters) {
        GeoPoint point = new GeoPoint(0.0, 0.0);
        GeoPoint pointEast = pointPlusDistanceEast(point, lonMeters);
        GeoPoint pointNorthEast = pointPlusDistanceNorth(pointEast, latMeters);
        return pointNorthEast;
    }

    private static GeoPoint pointPlusDistanceEast(GeoPoint point, double distance) {
        return getPointAhead(point, distance, 90.0);
    }

    private static GeoPoint pointPlusDistanceNorth(GeoPoint point, double distance) {
        return getPointAhead(point, distance, 0.0);
    }

    private static GeoPoint getPointAhead(GeoPoint point,
                                          double distance,
                                          double azimuthDegrees) {

        double radiusFraction = distance / EARTH_RADIUS;
        double bearing = Math.toRadians(azimuthDegrees);
        double lat1 = Math.toRadians(point.Latitude);
        double lng1 = Math.toRadians(point.Longitude);

        double lat2_part1 = Math.sin(lat1) * Math.cos(radiusFraction);
        double lat2_part2 = Math.cos(lat1) * Math.sin(radiusFraction) * Math.cos(bearing);
        double lat2 = Math.asin(lat2_part1 + lat2_part2);

        double lng2_part1 = Math.sin(bearing) * Math.sin(radiusFraction) * Math.cos(lat1);
        double lng2_part2 = Math.cos(radiusFraction) - Math.sin(lat1) * Math.sin(lat2);
        double lng2 = lng1 + Math.atan2(lng2_part1, lng2_part2);

        lng2 = (lng2 + 3.0*Math.PI) % (2.0*Math.PI) - Math.PI;

        GeoPoint res = new GeoPoint(Math.toDegrees(lat2), Math.toDegrees(lng2));
        return res;
    }
}