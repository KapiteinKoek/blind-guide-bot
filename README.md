# blind-guide-bot
Code to implement 'guide dog' functionality to Turtle soccer robots.

## How to compile
- Use the makefile
  1. Move to the correct directory: `cd blind-guide-bot`
  2. Execute the make command: `make`
- Manually compile the binary
- Include blindguide.h in an existing project and compile that

## How to use
- For standalone testing, use the main function (by simply running the compiled binary)
- When used in another project:
  1. Add borders:
     - Either call `initializeBorders()` which uses the border coordinates as specified in the `borderCoordinates` array
     - Or call `addBorder(bottomX, bottomY, topX, topY, goodSide)` (See blindguide.h for parameter explanation)
  2. Get the resistance coefficient for a given robot position and force vector by calling `getResistance(x, y, phi, forceX, forceY)`
  3. When necessary, call `addBorder(bottomX, bottomY, topX, topY, goodSize)` to add further borders.
  4. When no further use is required, call `cleanup()` to free allocated memory

## Important information
- `getResistance()` returns a double between (and including) 0 and 1.
  - 0 means that the robot should easily move along with the given force
  - 1 means that the robot should fully resist the given force
- All coordinate units are expected to be meters
- All force units are expected to be Newtons
- The `phi` parameter for the `getResistance()` function is currently not used
- At this time, border removal is impossible
