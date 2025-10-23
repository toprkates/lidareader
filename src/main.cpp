#include <iostream>
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <vector>
#include <string>

#include "constants.h"
#include "file_read.h"
#include "operations.h"
#include "screen.h"

#define TEST_DATA "scan_data_NaN.toml"


void drawAllLines(sf::RenderWindow& window, sf::Font& arial,
                  const std::vector<Point2D>& dotsPOS,
                  const std::vector<Line>& detectedLines,
                  const sf::Color color) {
    
    for (size_t i = 0; i < detectedLines.size(); ++i) {
        // Draw points belonging to this line
        for (int idx : detectedLines[i].pointIndices) {
            Point2D point = dotsPOS[idx];
            drawInRangeDots(window, arial, point, 5, sf::Color::Green);
        }
        
        // Draw line segment
        drawDetectedLine(window, dotsPOS, detectedLines[i], color);
        
        // Add label
        if (!detectedLines[i].pointIndices.empty()) {
            size_t midIdx = detectedLines[i].pointIndices.size() / 2;
            Point2D midPoint = dotsPOS[detectedLines[i].pointIndices[midIdx]];
            float screenX = convertCoordinateX(midPoint.x, gridscale);
            float screenY = convertCoordinateY(midPoint.y, gridscale);
            
            std::string label = "L" + std::to_string(i + 1);
            addText(window, arial, label, 12, color, true, screenX, screenY - 15, false);
        }
    }
}

void drawAllIntersections(sf::RenderWindow& window, sf::Font& arial,
                         const std::vector<Intersection>& validIntersections) {
    
    Point2D robotPos = {0.0, 0.0};
    
    for (const Intersection& inter : validIntersections) {
        // Draw dashed line from robot to intersection
        drawDashedLineBetweenPoints(window, robotPos, inter.point, 
                                   sf::Color::Red, 2.0f, 15.0f);
        
        // Draw intersection marker with label
        drawIntersectionMarker(window, arial, inter, true);
    }
}

void drawUIElements(sf::RenderWindow& window, sf::Font& arial) {
    screen(window, 2.f, arial, margin_X, margin_Y, frame_X+margin_X, frame_Y+margin_Y);
    mainFrame(window, arial, frame_X, frame_Y, margin_X, margin_Y, 2);
    addText(window, arial, std::to_string((int)frame_X), 10, sf::Color::Black, 
            false, frame_X/2+margin_X, margin_Y-30.f, false);
    addText(window, arial, std::to_string((int)frame_Y), 10, sf::Color::Black, 
            false, margin_X-50.f, screen_Y/2, false);
}

int main() {
    //Load the Head of the File, Lidar Scanned Parameters, Ranges and Intensities
    Header header = readHeader(TEST_DATA);
    Scan scan = readScan(TEST_DATA);
    std::vector<double> rangesPositions = readRanges(TEST_DATA);
    std::vector<double> intensities = readIntensities(TEST_DATA);
    std::vector<Point2D> dotsPOS = convertToCarterisan(rangesPositions, scan);

    RANSACparameters ransacConfig;
    ransacConfig.minPoints = 8;              // Minimum points to form a line
    ransacConfig.distanceThreshold = 0.01;   // 5 cm tolerance
    ransacConfig.maxIterations = 10000;       // Number of random samples

    std::vector<Line> detectedLines = detectLines(dotsPOS, ransacConfig);
    std::vector<Intersection> validIntersections = findValidIntersections(detectedLines, 60.0);

    std::cout << "\n=== RANSAC Results ===" << std::endl;
    std::cout << "Points: " << dotsPOS.size() << std::endl;
    std::cout << "Lines: " << detectedLines.size() << std::endl;
    std::cout << "Intersections: " << validIntersections.size() << std::endl;

    //create the window for drawing
    sf::RenderWindow window(sf::VideoMode({(unsigned int) screen_X, (unsigned int) screen_Y}), header.frame_id + " " + header.stamp);

    //loading font
    sf::Font arial;
    if (!arial.openFromFile("assets/visuals/arial.ttf")) {
        std::cerr << "Font Arial could not be opened" << std::endl;
        return -1;
    }

    //main loop for screen
    while (window.isOpen()) {
        // Handle events
        while (std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
            if (auto key = event->getIf<sf::Event::KeyPressed>()) {
                if (key->code == sf::Keyboard::Key::Escape)
                    window.close();
            }
        }
        
        window.clear(sf::Color::White);

        // Draw everything in layers
        drawUIElements(window, arial);
        
        // Draw all raw points (gray background)
        for (Point2D point : dotsPOS)
            drawInRangeDots(window, arial, point, 5, sf::Color(200, 200, 200));
        
        // Draw detected lines (colored)
        drawAllLines(window, arial, dotsPOS, detectedLines, sf::Color(150, 100, 100));
        
        // Draw intersections (red markers with labels)
        drawAllIntersections(window, arial, validIntersections);
        
        // Draw robot and legend
        robot(window, arial);
        drawLegend(window, arial, detectedLines.size(), validIntersections.size());

        window.display();
    }
}
