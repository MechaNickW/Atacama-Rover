import processing.serial.*; //<>// //<>// //<>// //<>// //<>// //<>// //<>//
import java.util.Iterator;
import boofcv.processing.*;
PGraphics GUI;

class Queue {

  PApplet sketch;
  CardList cardList;
  Rover rover;
  Serial myPort;
  Hexgrid hexgrid;
  Hexagon scanDest;
  boolean newCommands;
  RoverCommand currentCommand;
  float destinationHeading;
  float commandTotalDistance = 0;
  int checkCt = 0;
  PVector location;
  Se3_F64 roverToCamera;
  PVector moveStartLocation;

  ArrayList<Byte> byteList;
  ArrayList<RoverCommand> commandList;


  Queue(PApplet sketch_, CardList cardList_, Hexgrid hexgrid_, String serial, PGraphics GUI_) {
    cardList = cardList_;
    hexgrid = hexgrid_;
    byteList = new ArrayList<Byte>();
    commandList = new ArrayList<RoverCommand>();
    newCommands = false;
    sketch = sketch_;
    GUI = GUI_;
    myPort = new Serial(sketch, serial, 115200);
    moveStartLocation = new PVector();
    pickScanDest();
    location = new PVector(camWidth/2, camHeight/2);
  }

  void initRover(Rover rover_) {
    rover = rover_;
  }

  void update() {

    if ( myPort.available() > 0) { // If data is available,
      byte[] mainQueue = new byte[5];
      byte[] funcQueue = new byte[5];
      byte[] inBuffer = new byte[12];
      byte interesting = 16; //endByte
      inBuffer = myPort.readBytesUntil(interesting);
      myPort.clear();
      //println(inBuffer);
      if (inBuffer == null || inBuffer[0] == 'n') {
        //println("null stop");
        myPort.write('s');
        rover.stop();
        initClearCommandList();
        commandList.clear();
        return;
      } else if (inBuffer.length == 12) {
        for (int i = 0; i < 5; i++) {
          mainQueue[i] = inBuffer[i];
          funcQueue[i] = inBuffer[i+6];
        }
        parseCodingBlocks(mainQueue, funcQueue);
        if (byteList.isEmpty()) { //
          println("null stop");
          myPort.write('s');
          rover.stop();
          initClearCommandList();
          commandList.clear();
          return;
        }
        boolean execute;
        //myPort.readBytes(inBuffer);
        println(inBuffer);
        //println("inbuffer[0]" + inBuffer[0]);
        //println(inBuffer.length);

        if (inBuffer[5] == 'n') {//the user has pressed the button

          if (isExecutableCommand() || inBuffer[0] == 'n') { //if the rover is already driving:
            execute = false;
            println("stop received");
            myPort.write('s');
            rover.stop();
            initClearCommandList();
            commandList.clear();
            return;
          } else { // problem when it gets an executable command that is empty
            //println("button");
            execute = true;
            newCommands=true;
            println("data in -- execute");
            myPort.write('g'); //tell the reader board that the rover is driving
          }
        } else {
          execute = false;
          println("data in -- no execute");
          myPort.write('s'); //tell the reader board that the rover is stopped
        }

        parseCommandList(execute);
        //println("parsing");
      }
    }
    if (commandList.isEmpty()) {
    }
    updateGUI();
    //println("command list length: " + commandList.size());
  }



  void pickScanDest() {
    Hexagon h;
    Object[] keys = hexgrid.allHexes.keySet().toArray();
    do {
      Object randHexKey = keys[new Random().nextInt(keys.length)];
      h = hexgrid.getHex((PVector)randHexKey);
    } while (!h.inBounds && h!= scanDest);
    scanDest = h;
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
        if (tempByte == 113) { //"function"
          println("Function");
          function = true;
        } else if (isValid(tempByte, function)) {
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
          }
          funcCount++;
        }
        function = false;
        funcCount = 0;
        cmdCount++;
      }
    }
  }
  void initClearCommandList() {
    for (RoverCommand rc : commandList) {
      rc.h.fillin = false;
    }
  }
  void parseCommandList(boolean execute) {
    commandList.clear();
    //println(byteList);
    //if (rover.watchdog <= 5) {
    if (true) {
      PVector lastXY = rover.location;
      int cardinalHeading = roundHeading(rover.heading);
      Hexagon hexLoc = hexgrid.pixelToHex((int)lastXY.x, (int)lastXY.y);
      PVector hexKey = new PVector();
      hexKey.set(hexLoc.getKey()); //freezes if rover is offscreen

      initClearCommandList();
      for (byte cmd : byteList) {
        String iconName = "";
        boolean drive = false;
        boolean scan = false;
        if (cmd == 119) { // 'w' forward
          drive = true;
          iconName = "forward.jpg";
        } else if (cmd == 97) { // 'a' counterclockwise
          drive = false;
          iconName = "counterclockwise.jpg";
          cardinalHeading -= 1;
        } else if (cmd == 115) { // 's' back
          iconName = "uturn.jpg";
          drive = false;
          cardinalHeading += 3;
        } else if (cmd == 100) { // 'd' right/clockwise
          iconName = "clockwise.jpg";
          drive = false;
          cardinalHeading += 1;
        } else if (cmd==101) { // 'e' scan for life
          iconName = "scan.jpg";
          drive = false;
          scan = true;
        }
        while (cardinalHeading < 0 || cardinalHeading >= 6) {
          if (cardinalHeading < 0) {
            cardinalHeading += 6;
          }
          if (cardinalHeading >=6) {
            cardinalHeading -= 6;
          }
        }
        if (drive) {

          hexKey.add(hexgrid.neighbors[cardinalHeading]);
        }
        if (hexgrid.checkHex(hexKey)) {
          Hexagon h = hexgrid.getHex(hexKey);
          RoverCommand rc = new RoverCommand(h, cardinalHeading, drive, scan, iconName, execute);
          //Hexagon h_, int cardinalDir_, boolean drive_, boolean scan_, String iconName
          commandList.add(rc);
          //println("new command added");
        }
      }
      if (isActiveCommand()) {
        currentCommand = commandList.get(0);
      }
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

  void updateGUI() {
    GUI.beginDraw();
    GUI.background(0, 255, 255);
    GUI.pushMatrix();
    GUI.translate(50, GUI.height*.5);
    GUI.imageMode(CENTER);
    for (RoverCommand rc : commandList) {
      PImage icon = rc.getIcon();
      GUI.image(icon, 0, 0, 50, 50);
      GUI.translate(75, 0);
    }
    GUI.popMatrix();
    GUI.endDraw();
  }

  void drawHexes(PGraphics buffer) {
    buffer.beginDraw();
    for (RoverCommand rc : commandList) {
      if (rc.execute) {
        Hexagon h = rc.getHex();
        h.drawHexFill(buffer);
      }
    }
    scanDest.blinkHex(buffer);
    buffer.endDraw();
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
      myPort.write('s');
      println("empty");
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

  boolean isExecutableCommand() { //return true if the queue is executable
    if (isActiveCommand()) {
      RoverCommand rc = commandList.get(0);
      if (rc.execute) {
        return true;
      }
    }
    return false;
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
        checkCt++;
      }
    } else if (currentCommand.reorientStatus()) { //if drive portion is complete, check for reorientation turns
      destinationHeading = currentCommand.getRadianDir();
      moveStartLocation.set(location); //set the destination to the rover's current position
    } else {
      destinationHeading = rover.heading;
    }
    return destinationHeading;
  }

  float compareDistances(PVector roverDest) {
    float distTraveled = abs(PVector.dist(moveStartLocation, location));
    float turnDistToTravel = abs(PVector.dist(moveStartLocation, roverDest));
    float distCompare =turnDistToTravel - distTraveled; //negative number means it has gone too far

    return distCompare;
  }



  void updateLocation(FiducialFound f) {
    roverToCamera=f.getFiducialToCamera();
    location.x = ((float) f.getImageLocation().x);
    location.y = ((float) f.getImageLocation().y);
    rover.location = location;
    rover.drive();
  }
  PVector getDestination() {
    if (currentCommand.driveStatus()) {
      PVector destination = currentCommand.getXY();
      return destination;
    } else {
      return location;
    }
  }
  double getDistance() {
    Vector3D_F64 translation = roverToCamera.getT();
    double dist = currentCommand.h.getDist(translation);
    return dist;
  }

  boolean driveStatus() {
    return currentCommand.driveStatus();
  }
  boolean reorientStatus() {
    return currentCommand.reorientStatus();
  }
  void moveComplete() {
    if (currentCommand.scan) {

      if (cardList.scan(currentCommand.getHex(), scanDest)) {
        pickScanDest();
      }
      commandComplete();
    } else if (currentCommand.moveComplete()) {
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
      if (isExecutableCommand()) {
        myPort.write('g');
        println("driving");
      }
    } else {
      myPort.write('s');
      println("not driving");
    }
  }

  boolean isValid(byte tempByte, boolean function) {

    if (tempByte == 119) {    // 'w' forward
      return true;
    } else if (tempByte ==97) { // 'a' counterclockwise
      return true;
    } else if (tempByte == 115) { // 's' back
      return true;
    } else if (tempByte == 100) { // 'd' right/clockwise
      return true;
    } else if (tempByte == 101) { // 'e' scan for life
      return true;
    } else if (tempByte == 32) { // ' ' for stop
      rover.stop();
      return false;
    } else if (!function && tempByte ==  113) {// 'q' queue function ignores recursive functions
      return true;
    } else {
      return false;
    }
  }
}
