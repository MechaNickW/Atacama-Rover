//class to hold command info for each rover step //<>//

class RoverCommand extends Hexagon { // should this extend Hexagon class?
  int cardinalDir;
  float radianDir;
  boolean reorient, drive, function, execute, inBounds;
  boolean turnToHeading = true;
  boolean scan = false;
  //PVector xy;
  PImage icon;
  int headingCheckCt = 0;
  byte cmdByte;

  //Hexagon(Hexgrid hexgrid_, int hexQ_, int hexR_, int size_) {
  RoverCommand(Hexgrid hexgrid_, PVector hexKey_, int cardinalDir_, byte cmd_, boolean function_, boolean execute_) {
    super(hexgrid_, int(hexKey_.x), int(hexKey_.z));
    cmdByte = cmd_;
    execute = execute_;
    reorient = !drive;
    function = function_;
    inBounds = hexgrid.checkHex(hexKey_);
    while (cardinalDir_ < 0 || cardinalDir_ >= 6) {
      if (cardinalDir_ < 0) {
        cardinalDir_ += 6;
      }
      if (cardinalDir_ >= 6) {
        cardinalDir_ -= 6;
      }
    }
    cardinalDir = cardinalDir_;
    float[] cardHtoTheta = {0, 60, 120, 180, 240, 300};
    radianDir = radians(cardHtoTheta[cardinalDir]);
    String iconName = "";
    if (cmd == 119) { // 'w' forward
      iconName = "forward.jpg";
    } else if (cmd == 97) { // 'a' counterclockwise
      iconName = "counterclockwise.jpg";
    } else if (cmd == 115) { // 's' back
      iconName = "uturn.jpg";
    } else if (cmd == 100) { // 'd' right/clockwise
      iconName = "clockwise.jpg";
    } else if (cmd==101) { // 'e' scan for life
      iconName = "scan.jpg";
      scan = true;
    }
    String path = sketchPath() + "/data/icons/" + iconName;
    icon= loadImage(path);
    super.fillin = execute;
    //println("rc created");
  }

  float getRadianDir() {
    return radianDir;
  }
  int getCardinalDir() {
    return cardinalDir;
  }
  //PVector getXY() {
  //  return xy;
  //}
  boolean driveStatus() {
    return drive;
  }
  boolean scanStatus() {
    return scan;
  }
  boolean reorientStatus() {
    return reorient;
  }
  boolean turnToHeadingStatus() {
    return turnToHeading;
  }
  PImage getIcon() {
    return icon;
  }

  boolean moveComplete() {
    if (turnToHeading) {
      turnToHeading = false;
      return false;
    } else if (drive) {
      drive = false;
      return false;
    } else if (reorient) {
      reorient = false;
    }
    if (!turnToHeading && !drive && !reorient) {
      fillin = false;
      return true;
    } else {
      return false;
    }
  }
}
