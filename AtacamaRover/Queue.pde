import processing.serial.*; //<>//
import java.util.Iterator;


class Queue {

  PApplet sketch;
  Rover rover;
  Serial myPort;
  boolean newCommands;
  RoverCommand currentCommand;
  float destinationHeading;
  float commandTotalDistance = 0;
  int checkCt = 0;
  PVector location;
  PVector moveStartLocation;

  ArrayList<Byte> byteList;
  ArrayList<RoverCommand> commandList;


  Queue(PApplet sketch_, Rover rover_, String serial) {
    byteList = new ArrayList<Byte>();
    commandList = new ArrayList<RoverCommand>();
    newCommands = false;

    rover = rover_;
    sketch = sketch_;
    myPort = new Serial(sketch, serial, 115200);
    moveStartLocation = new PVector();
  }
  void update() {
    if ( myPort.available() > 0) { // If data is available,
      byte[] mainQueue = new byte[5];
      byte[] funcQueue = new byte[5];
      byte[] inBuffer = new byte[12];
      byte interesting = 16; //endByte
      inBuffer = myPort.readBytesUntil(interesting);
      if (inBuffer != null) {
        myPort.readBytes(inBuffer);

        for (int i = 0; i < 5; i++) {
          mainQueue[i] = inBuffer[i];
        }
        for (int i = 0; i < 5; i++) {
          funcQueue[i] = inBuffer[i+6];
        }
        myPort.clear();
        parseCodingBlocks(mainQueue, funcQueue);
        parseCommandList();
        newCommands = true;
      }
    }
  }
  void parseCodingBlocks( byte[] mainQueue, byte[] funcQueue ) {
    boolean function = false;
    int cmdCount = 0;
    int funcCount = 0;
    byte tempByte;

    byteList.clear();

    while (cmdCount < 5) {
      if (!function) {
        tempByte = mainQueue[cmdCount];
        if (tempByte == 113) //"function"
          function = true;
        else if (isValid(tempByte, function)) {
          byteList.add(tempByte);
        }
        if (!function) {
          cmdCount++;
        }
      }
      if (function) {
        while (funcCount < 5) {
          tempByte = funcQueue[funcCount];
          if (isValid(tempByte, function)) {
            { //ignore recursive functions and invalid commands
              byteList.add(tempByte);
            }
            funcCount++;
          }
        }
        function = false;
        funcCount = 0;
        cmdCount++;
      }
    }
  }
  void initClearCommandList(){
   for (RoverCommand rc: commandList){
     rc.h.fillin = false;
   }
  }
  void parseCommandList() {

    PVector lastXY = rover.location;
    int cardinalHeading = roundHeading(rover.heading);
    Hexagon hexLoc = hexGrid.pixelToHex((int)lastXY.x, (int)lastXY.y);
    PVector hexKey = new PVector();
    hexKey.set(hexLoc.getKey());
    boolean drive = false;
    initClearCommandList();
    for (byte cmd : byteList) {
      if (cmd == 119) { // 'w' forward
        drive = true;
      } else if (cmd == 97) { // 'a' counterclockwise
        drive = false;
        cardinalHeading -= 1;
      } else if (cmd == 115) { // 's' back
        drive = false;
        cardinalHeading += 3;
      } else if (cmd == 100) { // 'd' right/clockwise
        drive = false;
        cardinalHeading += 1;
      }
      if (cardinalHeading<0) {
        cardinalHeading +=6; //<>//
      }
      if (cardinalHeading > 6) {
        cardinalHeading-=6;
      }
      if (drive) {
        while (cardinalHeading < 0 || cardinalHeading >= 6) {
          if (cardinalHeading < 0) {
            cardinalHeading += 6;
          }
          if (cardinalHeading >=6) {
            cardinalHeading -= 6;
          }
        }
        hexKey.add(hexGrid.neighbors[cardinalHeading]);
      }
      Hexagon h = hexGrid.getHex(hexKey);
      RoverCommand rc = new RoverCommand(h, cardinalHeading, drive);
      commandList.add(rc);
    }
    if (isActiveCommand()) {
      currentCommand = commandList.get(0);
    }
  }
  int roundHeading(float heading_) {
    int cHeading = 0;
    if (degrees(heading_) > 330 || degrees(heading_) <= 30 ) { //refactor this into radians probably
      cHeading = 0;
    } else if (degrees(heading_) >  30 && degrees(heading_) <= 90 ) {
      cHeading = 1;
    } else if (degrees(heading_) >  90 && degrees(heading_) <= 150) {
      cHeading = 2;
    } else if (degrees(heading_) > 150 && degrees(heading_) <= 210) {
      cHeading = 3;
    } else if (degrees(heading_) > 210 && degrees(heading_) <= 270) {
      cHeading = 4;
    } else if (degrees(heading_) > 270 && degrees(heading_) <= 330) {
      cHeading = 5;
    }
    return (int) cHeading;
  }

  float cardDirToRadians(int cardD) {
    float[] cardHtoTheta = {0, 60, 120, 180, 240, 300};
    while (cardD < 0 || cardD >= 6) {
      if (cardD < 0) { 
        cardD += 6;
      }
      if (cardD >= 6) {
        cardD -= 6;
      }
    }
    return cardHtoTheta[cardD];
  }
  boolean checkNext() {
    if (commandList.isEmpty()) {
      //println("empty");
      return false;
    } else {
      return true;
    }
  }

  boolean checkNew() {
    if (newCommands) {
      newCommands = false;
      return true;
    } else {
      return false;
    }
  }
  boolean isActiveCommand() {
    return(!commandList.isEmpty());
    //check whether there is a command underway
  }
  float getHeading() {
    if (currentCommand.driveStatus()) {
      if (checkCt < 1) { //only calculate the heading once bc the angles get extreme when close to the destination
        PVector destination = currentCommand.getXY();
        float dy = destination.y - location.y;
        float dx = destination.x - location.x;
        moveStartLocation.set(location);
        commandTotalDistance = abs(PVector.dist(location, destination));
        destinationHeading = (atan2(dy, dx)+.5*PI);
        while (destinationHeading < 0 || destinationHeading > TWO_PI) {
          if (destinationHeading < 0) {
            destinationHeading += TWO_PI;
          }
          if (destinationHeading > TWO_PI) {
            destinationHeading -= TWO_PI;
          }
        }
        checkCt ++;
      }
    } else if (currentCommand.reorientStatus()) { //if drive portion is complete, check for reorientation turns
      destinationHeading = currentCommand.getRadianDir();
      moveStartLocation.set(location); //set the destination to the rover's current position
    }
    return destinationHeading;
  }

  float compareDistances(PVector roverDest) { //<>//
    float distTraveled = abs(PVector.dist(moveStartLocation, location));
    float turnDistToTravel = abs(PVector.dist(moveStartLocation, roverDest));
    float distCompare =turnDistToTravel - distTraveled; //negative number means it has gone too far

    return distCompare;
  }



  void updateLocation(PVector location_) {
    location = location_;
  }
  PVector getDestination() {
    if (currentCommand.driveStatus()) {
      PVector destination = currentCommand.getXY(); 
      return destination;
    } else {
      return location;
    }
  }

  boolean driveStatus() {
    return currentCommand.driveStatus();
  }
  boolean reorientStatus() {
    return currentCommand.reorientStatus();
  }
  void moveComplete() {
    if (currentCommand.moveComplete()){
      commandComplete();
    }
  }

  void commandComplete() {
    checkCt = 0;
    if (!commandList.isEmpty()) {
      commandList.remove(0);
    }
    nextCommand();
  }
  void nextCommand() {
    if (!commandList.isEmpty()) {
      currentCommand = commandList.get(0);
    }
  }

  boolean isValid(byte tempByte, boolean function) {

    if (tempByte == 119) {        // 'w' forward
      return true;
    } else if (tempByte ==97) {   // 'a' counterclockwise
      return true;
    } else if (tempByte == 115) { // 's' back
      return true;
    } else if (tempByte == 100) { // 'd' right/clockwise
      return true;
    } else if (tempByte == 101) { // 'e' scan for life
      return true;
    } else if (!function && tempByte ==  113) {// 'q' queue function
      return true;
    } else {
      return false;
    }
  }
}
