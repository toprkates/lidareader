#include <iostream>
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <vector>
#include <string>
#include <cmath>

#include "constants.h"

//create dots
int addDot (sf::RenderWindow& window, float size, bool dotAlign, float posX, 
            float posY, sf::Color color) {
    sf::CircleShape circle(size, 60); //default number of corners for circle shape
    circle.setFillColor(color); //default color of circle

    //centralize the origin by moving the origin to the circle's center
    if (dotAlign) circle.setOrigin({circle.getRadius(), circle.getRadius()});
    circle.setPosition({posX, posY});

    window.draw(circle);
    return 1;
}

//create texts
int addText (sf::RenderWindow& window, sf::Font& font, std::string message, 
            float size, sf::Color color, bool textAlign, float posX, float posY, bool setRotate) {
    sf::Text text(font, message, size);
    text.setFillColor(color);
    sf::FloatRect bounds = text.getLocalBounds();
    
    //if text wants to be aligned, we will put the origin of this text to its center
    if (textAlign) text.setOrigin(bounds.size/2.f);
    text.setPosition({posX, posY});

    if (setRotate) text.setRotation(sf::degrees(90.f));

    window.draw(text);
    return 1;
}

//create lines
int addLine (sf::RenderWindow& window, float sizeX, float sizeY, float posX, float posY, sf::Color color, float angle, bool centerContent) {
    sf::RectangleShape rect({sizeX, sizeY});
    rect.setFillColor(color);
    if (centerContent) rect.setOrigin({sizeX/2, sizeY/2});
    rect.setPosition({posX, posY});
    rect.setRotation(sf::degrees(angle));

    window.draw(rect);

    return 0;
}

// every 100.f is 1 in this coordinate scaling (its defined by gridscale)
float convertCoordinateX(float x, float scale) {
    float convx = x*scale + originX;  
    return convx;
}

float convertCoordinateY(float y, float scale) {
    float convy = -y*scale + originY;  // Note the negative sign - inverts Y axis
    return convy;
}

//creating the main frame of our graph
void mainFrame(sf::RenderWindow& window, sf::Font& font, float sizeX, float sizeY,
               float posX, float posY, int thickness) {
    //creating the frame that will hold the graph, transparent by default for now
    sf::RectangleShape rect({sizeX, sizeY});
    rect.setFillColor(sf::Color::Transparent);
    rect.setOutlineThickness(thickness);
    rect.setOutlineColor(sf::Color::Transparent);
    rect.setPosition({posX, posY});
    
    //putting numbers on x-axis with with relevant numbers
    int xAxis = -3;
    for (float i=margin_X; i<=sizeX + margin_X; i+=gridscale) {
        if (xAxis != -3) addText(window, font, std::to_string(xAxis), thickness*5, sf::Color::Black, false, i, screen_Y - margin_Y+5.f, false);
        xAxis++;
    }

    //putting numbers on y-axis with relevant numbers
    int yAxis = +3;
    for (float j=margin_Y; j<=sizeY + margin_Y; j+=gridscale) {
        addText(window, font, std::to_string(yAxis), thickness*5, sf::Color::Black, false, margin_X-15.f, j, false);
        yAxis--;
    }

    window.draw(rect);
}


//dots layout
void screen(sf::RenderWindow& window, float scale, sf::Font font, 
            float startX, float startY, float endX, float endY) {
    std::string numStr;
    
    //showcasing the corners actual coordinates
    /* 
    addText(window, font, std::to_string((int) startX) + "." + std::to_string((int) startY), 10, sf::Color::Black, false, startX, startY, 0);
    addText(window, font, std::to_string((int) endX) + "." + std::to_string((int) startY), 10, sf::Color::Black, false, endX, startY, 0);
    addText(window, font, std::to_string((int) startX) + "." + std::to_string((int) endY), 10, sf::Color::Black, false, startX, endY, 0);
    addText(window, font, std::to_string((int) endX) + "." + std::to_string((int) endY), 10, sf::Color::Black, false, endX, endY, 0);
    */

    //putting dots seperately
    for (float i=startX; i<=endX; i+=5.f*scale) {
        for (float j=startY; j<=endY; j+=5.f*scale) {
            //check for the corners
            if ((i==startX && j==startY) || (i==endX && j==startY) || (i ==startX && j==endY) || (i==endX && j==endY)) 
                addDot(window, 2.f, true, i, j, gray);
            else addDot(window, 2.f, true, i, j, gray);
        }
    }
    //Vertical lines (vary X)
    for (float x=startX+100.f; x<endX; x+=100.f) {
        if (!(x==startX || x==endX)) {
            if (x == margin_X+(endX-startX)/2) addLine(window, 2.f, endY-startY, x, startY, darkGray, 0.f, false);
            else addLine(window, 2.f, endY-startY, x, startY, gray, 0.f, false);
        }
    }
    //Horizontal lines (vary Y)
    for (float y=startY+100.f; y<endY; y+=100.f) { //the window's y axis + 100f get us the start of the actual start of frame's y axis, until the end of the graph we put lines  
        if (!(y==startY || y==endY)){
            if (y == margin_Y+(endY-startY)/2) addLine(window, endX-startX, 2.f, startX, y, darkGray, 0.f, false);
            else addLine(window, endX-startX, 2.f, startX, y, gray, 0.f, false); // the horizontal(yatay) lines are end - start size, x varies here
        } 
    }
}

//Draws Dots that are in range
void drawInRangeDots(sf::RenderWindow& window, sf::Font& font, Point2D& point, int precision, sf::Color color) {
    double posX=point.x, posY=point.y;
    std::string strX, strY;

    posX = convertCoordinateX(posX, gridscale);
    posY = convertCoordinateY(posY, gridscale);

    strX = std::to_string(point.x).substr(0, precision);
    strY = std::to_string(point.y).substr(0, precision);

    addDot(window, 3, true, posX, posY, color);
    //addText(window, font, strX + "," + strY, 10, color, false, posX, posY, false);
}

//create the robot
void robot(sf::RenderWindow& window, sf::Font& font) {
    addDot(window, 7, true, frame_X/2+margin_X, frame_Y/2+margin_Y, sf::Color::Red);
    addText(window, font, "Robot", 10, sf::Color::Red, true, frame_X/2+margin_X, frame_Y/2+margin_Y+10.f, false);
}


void drawLineBetweenPoints(sf::RenderWindow& window, const Point2D& p1, const Point2D& p2,
                          sf::Color color, float thickness = 2.0f) {
    //Convert world coordinates to screen coordinates
    float screenX1 = convertCoordinateX(p1.x, gridscale);
    float screenY1 = convertCoordinateY(p1.y, gridscale);
    float screenX2 = convertCoordinateX(p2.x, gridscale);
    float screenY2 = convertCoordinateY(p2.y, gridscale);
    
    //Calculate line length and angle
    float dx = screenX2 - screenX1;
    float dy = screenY2 - screenY1;
    float length = std::sqrt(dx * dx + dy * dy);
    float angle = std::atan2(dy, dx) * 180.0f / M_PI;
    
    //Draw the line using addLine function
    addLine(window, length, thickness, screenX1, screenY1, color, angle, false);
}

void drawDashedLineBetweenPoints(sf::RenderWindow& window, const Point2D& p1, const Point2D& p2,
                                 sf::Color color, float thickness = 2.0f, float dashLength = 10.0f) {
    // Convert to screen coordinates
    float screenX1 = convertCoordinateX(p1.x, gridscale);
    float screenY1 = convertCoordinateY(p1.y, gridscale);
    float screenX2 = convertCoordinateX(p2.x, gridscale);
    float screenY2 = convertCoordinateY(p2.y, gridscale);
    
    // Calculate distance and direction
    float dx = screenX2 - screenX1;
    float dy = screenY2 - screenY1;
    float totalLength = std::sqrt(dx * dx + dy * dy);
    float angle = std::atan2(dy, dx) * 180.0f / M_PI;
    
    // Normalize direction
    float ndx = dx / totalLength;
    float ndy = dy / totalLength;
    
    // Draw dashes
    bool drawDash = true;
    for (float d = 0; d < totalLength; d += dashLength) {
        if (drawDash) {
            float currentX = screenX1 + ndx * d;
            float currentY = screenY1 + ndy * d;
            float currentDashLength = std::min(dashLength * 0.6f, totalLength - d);
            
            addLine(window, currentDashLength, thickness, currentX, currentY, color, angle, false);
        }
        drawDash = !drawDash;
    }
}


void drawDetectedLine(sf::RenderWindow& window, const std::vector<Point2D>& points, 
                     const Line& line, sf::Color color) {
    // Need at least 2 points to draw a line
    if (line.pointIndices.size() < 2) return;
    
    // Find the two endpoints (first and last point in the line)
    Point2D firstPoint = points[line.pointIndices.front()];
    Point2D lastPoint = points[line.pointIndices.back()];
    
    // Draw line connecting the endpoints
    drawLineBetweenPoints(window, firstPoint, lastPoint, color, 3.0f);
}


void drawIntersectionMarker(sf::RenderWindow& window, sf::Font& font, 
                           const Intersection& intersection, bool showLabel = true) {
    // Convert to screen coordinates
    float screenX = convertCoordinateX(intersection.point.x, gridscale);
    float screenY = convertCoordinateY(intersection.point.y, gridscale);
    
    // Draw bullseye effect (red circle with white center)
    addDot(window, 12, true, screenX, screenY, sf::Color::Red);      // Outer red circle
    addDot(window, 8, true, screenX, screenY, sf::Color::White);     // Middle white circle
    addDot(window, 4, true, screenX, screenY, sf::Color::Red);       // Inner red circle
    
    // Optionally draw label with angle and distance info
    if (showLabel) {
        std::string angleStr = std::to_string((int)intersection.angle_degrees) + "°";
        std::string distStr = std::to_string((int)(intersection.distance_to_robot * 100)) + "cm";
        std::string label = angleStr + " " + distStr;
        
        // Draw label slightly offset from the marker
        addText(window, font, label, 10, sf::Color::Red, false, 
               screenX + 15, screenY - 5, false);
    }
}

//side bar for showing
void drawLegend(sf::RenderWindow& window, sf::Font& font, int numLines, int numIntersections) {
    float legendX = margin_X + 10;
    float legendY = margin_Y + 10;
    float lineHeight = 18;
    float currentY = legendY;
    
    // Title
    addText(window, font, "Legend:", 12, sf::Color::Black, false, legendX, currentY, false);
    currentY += lineHeight + 5;
    
    // Raw points
    addDot(window, 3, true, legendX + 5, currentY + 5, sf::Color(200, 200, 200));
    addText(window, font, "Raw LIDAR Points", 10, sf::Color::Black, false, 
           legendX + 15, currentY, false);
    currentY += lineHeight;
    
    // Detected lines
    addDot(window, 5, true, legendX + 5, currentY + 5, sf::Color::Green);
    std::string linesText = "Detected Lines (" + std::to_string(numLines) + ")";
    addText(window, font, linesText, 10, sf::Color::Black, false, 
           legendX + 15, currentY, false);
    currentY += lineHeight;
    
    // Intersections
    addDot(window, 8, true, legendX + 5, currentY + 5, sf::Color::Red);
    std::string intersText = "Intersections (" + std::to_string(numIntersections) + ")";
    addText(window, font, intersText, 10, sf::Color::Black, false, 
           legendX + 15, currentY, false);
    currentY += lineHeight;
    
    // Robot
    addDot(window, 7, true, legendX + 5, currentY + 5, sf::Color::Red);
    addText(window, font, "Robot Position", 10, sf::Color::Black, false, 
           legendX + 15, currentY, false);
}



