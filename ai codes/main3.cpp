#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <algorithm>
#include <limits>
#include <cstdlib>
#define M_PI 3.14159265358979323846

struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
};

struct Line {
    double a, b, c; // ax + by + c = 0
    std::vector<Point> points;
    
    Line(double a = 0, double b = 0, double c = 0) : a(a), b(b), c(c) {}
};

struct LidarData {
    double angle_min, angle_max, angle_increment;
    double range_min, range_max;
    std::vector<double> ranges;
};

// TOML Parser - Custom implementation
class TomlParser {
public:
    static LidarData parse(const std::string& filename) {
        LidarData data;
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Cannot open file " << filename << std::endl;
            exit(1);
        }

        std::string line;
        bool in_ranges = false;
        std::string ranges_str = "";

        while (std::getline(file, line)) {
            // Remove whitespace
            line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());
            
            if (line.empty() || line[0] == '#' || line[0] == '[') continue;

            if (line.find("angle_min=") == 0) {
                data.angle_min = parseValue(line);
            }
            else if (line.find("angle_max=") == 0) {
                data.angle_max = parseValue(line);
            }
            else if (line.find("angle_increment=") == 0) {
                data.angle_increment = parseValue(line);
            }
            else if (line.find("range_min=") == 0) {
                data.range_min = parseValue(line);
            }
            else if (line.find("range_max=") == 0) {
                data.range_max = parseValue(line);
            }
            else if (line.find("ranges=") == 0) {
                in_ranges = true;
                ranges_str = line.substr(7); // Remove "ranges="
            }
            else if (in_ranges) {
                ranges_str += line;
            }
        }

        // Parse ranges array
        parseRanges(ranges_str, data.ranges);
        file.close();
        return data;
    }

private:
    static double parseValue(const std::string& line) {
        size_t pos = line.find('=');
        if (pos != std::string::npos) {
            return std::stod(line.substr(pos + 1));
        }
        return 0.0;
    }

    static void parseRanges(std::string str, std::vector<double>& ranges) {
        // Remove brackets
        str.erase(std::remove(str.begin(), str.end(), '['), str.end());
        str.erase(std::remove(str.begin(), str.end(), ']'), str.end());
        
        std::stringstream ss(str);
        std::string token;
        
        while (std::getline(ss, token, ',')) {
            try {
                double val = std::stod(token);
                ranges.push_back(val);
            } catch (...) {
                ranges.push_back(-1.0); // Invalid value
            }
        }
    }
};

// Convert polar to Cartesian coordinates
std::vector<Point> polarToCartesian(const LidarData& data) {
    std::vector<Point> points;
    double angle = data.angle_min;
    
    for (size_t i = 0; i < data.ranges.size(); i++) {
        double range = data.ranges[i];
        
        // Filter invalid points
        if (range > 0 && range >= data.range_min && range <= data.range_max) {
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            points.push_back(Point(x, y));
        }
        
        angle += data.angle_increment;
    }
    
    return points;
}

// Calculate distance between two points
double distance(const Point& p1, const Point& p2) {
    return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

// Calculate distance from point to line
double pointToLineDistance(const Point& p, const Line& line) {
    return std::abs(line.a * p.x + line.b * p.y + line.c) / 
           std::sqrt(line.a * line.a + line.b * line.b);
}

// RANSAC Line Detection
std::vector<Line> detectLines(const std::vector<Point>& points, int iterations = 1000, 
                         double threshold = 0.05, int minPoints = 8) {
    std::vector<Line> lines;
    std::vector<bool> used(points.size(), false);
    
    for (int iter = 0; iter < 10; iter++) { // Detect up to 10 lines
        if (std::count(used.begin(), used.end(), false) < minPoints) break;
        
        Line bestLine;
        std::vector<int> bestInliers;
        int maxInliers = 0;
        
        // RANSAC iterations
        for (int i = 0; i < iterations; i++) {
            // Randomly select two unused points
            std::vector<int> availableIndices;
            for (size_t j = 0; j < points.size(); j++) {
                if (!used[j]) availableIndices.push_back(j);
            }
            
            if (availableIndices.size() < 2) break;
            
            int idx1 = availableIndices[std::rand() % availableIndices.size()];
            int idx2 = availableIndices[std::rand() % availableIndices.size()];
            
            if (idx1 == idx2) continue;
            
            Point p1 = points[idx1];
            Point p2 = points[idx2];
            
            // Calculate line equation ax + by + c = 0
            double a = p2.y - p1.y;
            double b = p1.x - p2.x;
            double c = p2.x * p1.y - p1.x * p2.y;
            
            // Normalize
            double norm = std::sqrt(a * a + b * b);
            if (norm < 1e-10) continue;
            a /= norm; b /= norm; c /= norm;
            
            Line currentLine(a, b, c);
            std::vector<int> inliers;
            
            // Count inliers
            for (size_t j = 0; j < points.size(); j++) {
                if (!used[j] && pointToLineDistance(points[j], currentLine) < threshold) {
                    inliers.push_back(j);
                }
            }
            
            if (inliers.size() > static_cast<size_t>(maxInliers)) {
                maxInliers = inliers.size();
                bestInliers = inliers;
                bestLine = currentLine;
            }
        }
        
        if (maxInliers >= minPoints) {
            // Add points to line
            for (int idx : bestInliers) {
                bestLine.points.push_back(points[idx]);
                used[idx] = true;
            }
            lines.push_back(bestLine);
        } else {
            break;
        }
    }
    
    return lines;
}

// Find intersection of two lines
bool findIntersection(const Line& l1, const Line& l2, Point& intersection) {
    double det = l1.a * l2.b - l2.a * l1.b;
    
    if (std::abs(det) < 1e-10) return false; // Parallel lines
    
    intersection.x = (l1.b * l2.c - l2.b * l1.c) / det;
    intersection.y = (l2.a * l1.c - l1.a * l2.c) / det;
    
    return true;
}

// Calculate angle between two lines (in degrees)
double angleBetweenLines(const Line& l1, const Line& l2) {
    double dot = l1.a * l2.a + l1.b * l2.b;
    double mag1 = std::sqrt(l1.a * l1.a + l1.b * l1.b);
    double mag2 = std::sqrt(l2.a * l2.a + l2.b * l2.b);
    
    double cosAngle = std::abs(dot) / (mag1 * mag2);
    cosAngle = std::min(1.0, std::max(-1.0, cosAngle));
    
    double angle = std::acos(cosAngle) * 180.0 / M_PI;
    
    // Return the acute angle
    if (angle > 90) angle = 180 - angle;
    
    return angle;
}

// Generate SVG for visualization
void generateSVG(const std::vector<Point>& points, const std::vector<Line>& lines,
                 const std::vector<std::pair<Point, double>>& intersections, 
                 const std::string& filename = "lidar_visualization.svg") {
    std::ofstream svg(filename);
    
    // Find bounds
    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();
    
    for (const auto& p : points) {
        minX = std::min(minX, p.x);
        maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
    }
    
    double margin = 50;
    double width = 800, height = 800;
    double scale = std::min((width - 2 * margin) / (maxX - minX), 
                            (height - 2 * margin) / (maxY - minY));
    
    auto transform = [&](double x, double y) -> std::pair<double, double> {
        return {margin + (x - minX) * scale, height - margin - (y - minY) * scale};
    };
    
    svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    svg << "<svg width=\"" << width << "\" height=\"" << height 
        << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    svg << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
    
    // Draw points
    for (const auto& p : points) {
        auto coords = transform(p.x, p.y);
        svg << "<circle cx=\"" << coords.first << "\" cy=\"" << coords.second 
            << "\" r=\"2\" fill=\"blue\" opacity=\"0.5\"/>\n";
    }
    
    // Draw lines
    for (const auto& line : lines) {
        if (line.points.size() < 2) continue;
        
        double minT = std::numeric_limits<double>::max();
        double maxT = std::numeric_limits<double>::lowest();
        
        for (const auto& p : line.points) {
            double t = (std::abs(line.a) > std::abs(line.b)) ? p.x : p.y;
            minT = std::min(minT, t);
            maxT = std::max(maxT, t);
        }
        
        Point p1, p2;
        if (std::abs(line.a) > std::abs(line.b)) {
            p1.x = minT - 0.5;
            p1.y = -(line.a * p1.x + line.c) / line.b;
            p2.x = maxT + 0.5;
            p2.y = -(line.a * p2.x + line.c) / line.b;
        } else {
            p1.y = minT - 0.5;
            p1.x = -(line.b * p1.y + line.c) / line.a;
            p2.y = maxT + 0.5;
            p2.x = -(line.b * p2.y + line.c) / line.a;
        }
        
        auto coords1 = transform(p1.x, p1.y);
        auto coords2 = transform(p2.x, p2.y);
        svg << "<line x1=\"" << coords1.first << "\" y1=\"" << coords1.second 
            << "\" x2=\"" << coords2.first << "\" y2=\"" << coords2.second 
            << "\" stroke=\"red\" stroke-width=\"2\"/>\n";
    }
    
    // Draw robot position
    auto robotCoords = transform(0, 0);
    svg << "<circle cx=\"" << robotCoords.first << "\" cy=\"" << robotCoords.second 
        << "\" r=\"8\" fill=\"green\" stroke=\"black\" stroke-width=\"2\"/>\n";
    svg << "<text x=\"" << robotCoords.first + 10 << "\" y=\"" << robotCoords.second - 10 
        << "\" font-size=\"14\" fill=\"green\">Robot (0,0)</text>\n";
    
    // Draw intersections
    for (const auto& item : intersections) {
        const Point& inter = item.first;
        double angle = item.second;
        
        auto interCoords = transform(inter.x, inter.y);
        svg << "<circle cx=\"" << interCoords.first << "\" cy=\"" << interCoords.second 
            << "\" r=\"6\" fill=\"orange\" stroke=\"black\" stroke-width=\"2\"/>\n";
        
        double dist = distance(Point(0, 0), inter);
        svg << "<text x=\"" << interCoords.first + 10 << "\" y=\"" << interCoords.second - 10 
            << "\" font-size=\"12\" fill=\"orange\">"
            << "(" << inter.x << "," << inter.y << ") "
            << angle << "° d=" << dist << "m</text>\n";
        
        // Draw line from robot to intersection
        svg << "<line x1=\"" << robotCoords.first << "\" y1=\"" << robotCoords.second 
            << "\" x2=\"" << interCoords.first << "\" y2=\"" << interCoords.second 
            << "\" stroke=\"purple\" stroke-width=\"1\" stroke-dasharray=\"5,5\"/>\n";
    }
    
    svg << "</svg>";
    svg.close();
    
    std::cout << "\nVisualization saved to: " << filename << std::endl;
}

int main(int argc, char* argv[]) {
    std::string filename = "lidar1.toml";
    
    if (argc > 1) {
        filename = argv[1];
    }
    
    std::cout << "=== LIDAR Precision Docking System ===" << std::endl;
    std::cout << "Reading file: " << filename << std::endl;
    
    // Parse TOML file
    LidarData data = TomlParser::parse(filename);
    
    std::cout << "LIDAR Parameters:" << std::endl;
    std::cout << "  Angle range: " << data.angle_min << " to " << data.angle_max << " rad" << std::endl;
    std::cout << "  Angle increment: " << data.angle_increment << " rad" << std::endl;
    std::cout << "  Range: " << data.range_min << " to " << data.range_max << " m" << std::endl;
    std::cout << "  Total measurements: " << data.ranges.size() << std::endl;
    
    // Convert to Cartesian coordinates
    std::vector<Point> points = polarToCartesian(data);
    std::cout << "Valid points after filtering: " << points.size() << std::endl;
    
    // Detect lines using RANSAC
    std::cout << "\nDetecting lines using RANSAC..." << std::endl;
    std::vector<Line> lines = detectLines(points);
    std::cout << "Detected " << lines.size() << " lines" << std::endl;
    
    for (size_t i = 0; i < lines.size(); i++) {
        std::cout << "  Line " << i + 1 << ": " << lines[i].points.size() 
             << " points (eq: " << lines[i].a << "x + " 
             << lines[i].b << "y + " << lines[i].c << " = 0)" << std::endl;
    }
    
    // Find intersections and calculate angles
    std::cout << "\nAnalyzing intersections..." << std::endl;
    std::vector<std::pair<Point, double>> validIntersections;
    double angleThreshold = 60.0; // degrees
    
    for (size_t i = 0; i < lines.size(); i++) {
        for (size_t j = i + 1; j < lines.size(); j++) {
            Point intersection;
            if (findIntersection(lines[i], lines[j], intersection)) {
                double angle = angleBetweenLines(lines[i], lines[j]);
                
                if (angle >= angleThreshold) {
                    double dist = distance(Point(0, 0), intersection);
                    validIntersections.push_back({intersection, angle});
                    
                    std::cout << "  Lines " << i + 1 << " & " << j + 1 
                         << ": Intersection at (" << intersection.x << ", " << intersection.y 
                         << "), Angle: " << angle << "°, Distance from robot: " << dist << " m" << std::endl;
                }
            }
        }
    }
    
    if (validIntersections.empty()) {
        std::cout << "\nNo intersections found with angle >= " << angleThreshold << " degrees" << std::endl;
    } else {
        std::cout << "\nFound " << validIntersections.size() 
             << " intersection(s) with angle >= " << angleThreshold << " degrees" << std::endl;
    }
    
    // Generate visualization
    generateSVG(points, lines, validIntersections);
    
    return 0;
}