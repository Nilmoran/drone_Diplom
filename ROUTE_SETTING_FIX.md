# Route Setting Functionality - Fix & Troubleshooting Guide

## Issue Summary

**Problem**: Users cannot set multiple routes - they cannot designate a destination (point B) either during or after executing a route.

**Status**: ✅ **FIXED**

## Root Cause

The map click handler had a restrictive condition that prevented route fetching:

```javascript
// OLD CODE (BROKEN)
if (droneLocation && calculateDistanceMeters(...) > 10) { 
    fetchRoute(droneLocation, endPoint); 
}
```

### Problems:
1. **Required `droneLocation` to exist** - If drone hasn't received position yet, route couldn't be set
2. **10-meter minimum distance** - Too restrictive, prevented close waypoints
3. **No fallback mechanism** - No alternative if drone position unavailable
4. **No route updates during flight** - Couldn't change destination mid-flight

## Solution Implemented

### 1. Intelligent Fallback System
- **Primary**: Use current drone position if available
- **Fallback 1**: Use start of current route if drone position not available
- **Fallback 2**: Use map center as last resort

### 2. Reduced Distance Threshold
- Changed from 10 meters to 1 meter minimum
- Allows closer waypoints while preventing same-point routes

### 3. Always Allow Route Setting
- Route can be set at any time (before, during, or after flight)
- No dependency on drone position being available

### 4. Better Error Handling
- User feedback when route cannot be fetched
- Console logging for debugging
- WebSocket connection status warnings

## Changes Made

### File: `templates/map_template.html`

**Lines 414-450**: Updated map click handler with:
- Intelligent fallback position selection
- Reduced distance threshold (10m → 1m)
- Better error messages
- Console logging for debugging

**Lines 286-310**: Enhanced `fetchRoute` function with:
- Better error handling
- WebSocket connection status checks
- User feedback for connection issues
- Validation for empty route results

## How to Use

### Setting a New Route

1. **Click anywhere on the map** to set destination (point B)
2. **Destination marker appears** immediately
3. **Route is automatically calculated** and displayed
4. **Route is sent to drone** if WebSocket is connected

### Setting Routes During Flight

1. **Click on map** while drone is flying
2. **New route is calculated** from current drone position
3. **Route updates immediately** - drone will follow new path
4. **Previous route is replaced** automatically

### Setting Routes Before Drone Position Available

1. **Click on map** even if drone hasn't received position yet
2. **System uses fallback** (route start or map center)
3. **Route is calculated** and displayed
4. **Route updates** when drone position becomes available

## Troubleshooting

### Issue: Clicking on map doesn't show route

**Possible Causes:**
1. **WebSocket not connected**
   - Check server status indicator (should be "Connected")
   - Refresh page if disconnected
   - Restart server if needed

2. **2GIS API error**
   - Check browser console for error messages
   - Verify API key is valid
   - Check internet connection

3. **Points too close together**
   - Ensure destination is at least 1 meter from start
   - Try selecting a destination further away

**Solutions:**
- Open browser console (F12) to see error messages
- Check server console for WebSocket connection status
- Verify 2GIS API key in `main.py` (line 27)
- Try clicking a different location on the map

### Issue: Route appears but drone doesn't follow it

**Possible Causes:**
1. **WebSocket not connected**
   - Route is displayed but not sent to server
   - Check server status indicator

2. **Server not running**
   - Ensure `main.py` is running
   - Check for error messages in server console

**Solutions:**
- Check WebSocket connection status
- Restart server if needed
- Verify route was sent (check server console for "Route received" message)

### Issue: Destination marker appears but no route line

**Possible Causes:**
1. **Route API returned empty result**
   - Destination might be unreachable
   - API might be rate-limited

2. **Route calculation failed**
   - Check browser console for errors
   - Try a different destination

**Solutions:**
- Check browser console (F12) for error messages
- Try selecting a destination closer to roads
- Wait a few seconds and try again (API rate limiting)

### Issue: Cannot set route after drone reaches destination

**This should now work!** The fix allows route setting at any time.

**If it still doesn't work:**
1. Check that `droneLocation` is not null (should be set after first position update)
2. Verify WebSocket is still connected
3. Try clicking "Reset Route" button first, then set new destination

## Testing Checklist

- [ ] **Before flight**: Click on map → Route should appear
- [ ] **During flight**: Click on map → New route should replace old one
- [ ] **After flight**: Click on map → New route should be set
- [ ] **No drone position**: Click on map → Route should still be calculated (using fallback)
- [ ] **Close waypoints**: Click near current position → Should work if > 1m away
- [ ] **WebSocket disconnected**: Click on map → Should show warning but still display route

## Technical Details

### Fallback Priority

1. **`droneLocation`** - Current drone position (if available)
2. **`currentRouteWaypoints[0]`** - Start of current route (if route exists)
3. **`map.getCenter()`** - Map center (last resort)

### Distance Threshold

- **Minimum**: 1 meter (prevents same-point routes)
- **Previous**: 10 meters (too restrictive)

### Route Update Behavior

- **During flight**: New route immediately replaces old route
- **Server handling**: Server accepts new routes at any time via `set_route` message
- **Drone behavior**: Drone will follow new route from current position

## Code Changes Summary

### Before (Broken)
```javascript
if (droneLocation && calculateDistanceMeters(...) > 10) { 
    fetchRoute(droneLocation, endPoint); 
}
```

### After (Fixed)
```javascript
// Intelligent fallback system
let routeStart = droneLocation || 
                 currentRouteWaypoints[0] || 
                 { lat: map.getCenter()[1], lon: map.getCenter()[0] };

if (calculateDistanceMeters(...) > 1.0) {
    fetchRoute(routeStart, endPoint);
} else {
    alert('Destination too close to start point...');
}
```

## Verification

To verify the fix is working:

1. **Start the server**: `python main.py`
2. **Open browser**: Navigate to `http://localhost:8000/drone_route_map.html`
3. **Test route setting**:
   - Click on map before drone position is available → Should work
   - Click on map during flight → Should update route
   - Click on map after flight → Should set new route
4. **Check console**: Open browser console (F12) to see route calculation logs

## Additional Notes

- The fix maintains backward compatibility
- No changes needed to server-side code
- All existing functionality preserved
- Enhanced error handling and user feedback

## Support

If issues persist after applying this fix:

1. Check browser console (F12) for JavaScript errors
2. Check server console for WebSocket/route errors
3. Verify 2GIS API key is valid and not rate-limited
4. Ensure all files are saved and server is restarted

