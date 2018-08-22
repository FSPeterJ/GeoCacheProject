# GeoCacheProject


This was a 1-month group project to program a GPS system using an arduino and a 5x8 full color LED display.

The very low memory constraint of the device was an interesting part as any heap corruption from having too much on the stack or fragmented heap produced a very colorful mess on the display.  

Smoothing the current position was also tough to lock down to a good enough feel to make it useful.  A lot of wandering around outside with the laptop and the device sitting on top tweaking values and reading console output. 

The end result was a display of a single blue dot on a squarish circle with an occasional trailing orange dot to indicate an angle between the two points.  A 5 dot bar at the bottom would slowly peel off its color with distance with stacked color bars underneath to facilitate longer distances.

The accuracy was fairly decent.  There was some confusion at one point with the flag being on the second story of a building but we figured it out.
