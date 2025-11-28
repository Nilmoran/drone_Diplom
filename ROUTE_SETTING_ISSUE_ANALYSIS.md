# Route Setting Issue - Analysis & Solution

## Problem Description

Users cannot set multiple routes - they cannot designate a destination (point B) either during or after executing a route.

## Root Cause Analysis

### Primary Issue: Conditional Route Fetching

**Location**: `templates/map_template.html`, line 429

```javascript
if (droneLocation && calculateDistanceMeters(droneLocation.lat, droneLocation.lon, endPoint.lat, endPoint.lon) > 10) { 
    fetchRoute(droneLocation, endPoint); 
}
```

### Problems Identified

1. **Requires `droneLocation` to exist**
   - If the drone hasn't received a position update yet, `droneLocation` is `null`
   - Clicking on the map sets the marker but doesn't fetch a route
   - User sees the destination marker but no route appears

2. **10-meter minimum distance requirement**
   - If user clicks within 10 meters of current drone position, route is not fetched
   - This prevents setting routes when drone is near a waypoint
   - Too restrictive for urban navigation where waypoints can be close

3. **No fallback mechanism**
   - If `droneLocation` is null, there's no alternative way to fetch a route
   - Should use last known position or allow route from any point

4. **No route update during flight**
   - Once a route is set, clicking again might not update it if conditions aren't met
   - Should allow route updates at any time

## Impact

- **User Experience**: Users click on map, see destination marker, but no route appears
- **Functionality Loss**: Cannot set new routes during or after flight
- **Workflow Disruption**: Breaks the expected behavior of "click to set destination"

## Solution Strategy

1. **Always allow route fetching** - Remove dependency on `droneLocation` being available
2. **Use fallback position** - If `droneLocation` is null, use last known position or route start
3. **Reduce distance threshold** - Change from 10m to 1m (just to prevent same-point routes)
4. **Allow route updates** - Always fetch route when destination is clicked, regardless of flight state
5. **Better error handling** - Provide user feedback when route cannot be fetched

## Implementation

See code changes in `templates/map_template.html` for the fix.

