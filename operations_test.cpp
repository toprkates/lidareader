#include <iostream> 
#include <fstream> 
#include <string> 
#include <vector>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <random>
#include <limits>

struct Point2D
{
    double x; 
    double y;
};

struct LidarScanParams
{
    double angle_min = 0.0;
    double angle_max = 0.0;
    double angle_increment = 0.0;
    double range_min = 0.0;
    double range_max = 0.0;
};

// Doğru denklemi: ax + by + c = 0 formatında
struct Line
{
    double a, b, c;
    std::vector<int> pointIndices; // Bu doğruyu oluşturan nokta indeksleri
    
    // İki nokta arasındaki doğru denklemini hesapla
    void computeFromPoints(const Point2D& p1, const Point2D& p2)
    {
        a = p2.y - p1.y;
        b = p1.x - p2.x;
        c = -(a * p1.x + b * p1.y);
        
        // Normalizasyon
        double norm = sqrt(a*a + b*b);
        if (norm > 1e-10) {
            a /= norm;
            b /= norm;
            c /= norm;
        }
    }
    
    // Noktanın doğruya uzaklığını hesapla
    double distanceToPoint(const Point2D& p) const
    {
        return fabs(a * p.x + b * p.y + c) / sqrt(a*a + b*b);
    }
};

struct Intersection
{
    Point2D point;
    int line1_idx;
    int line2_idx;
    double angle_degrees;
    double distance_to_robot;
};

void trim(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

bool parseKeyValue(const std::string& line, const std::string& key, double& value)
{
    std::stringstream ss(line);  
    std::string crrKey;
    char sep;
    
    ss >> crrKey;
    if (crrKey == key) 
    {
        ss >> sep;
        if (sep == '=')
        {
            ss >> value;
            return true;
        }
    }
    return false; 
}

// RANSAC algoritması ile doğru tespiti
std::vector<Line> detectLinesRANSAC(const std::vector<Point2D>& points, 
                                     int minPoints = 8,
                                     double distanceThreshold = 0.05,
                                     int maxIterations = 1000)
{
    std::vector<Line> detectedLines;
    std::vector<bool> used(points.size(), false);
    std::random_device rd;
    std::mt19937 gen(rd());
    
    while (true)
    {
        // Kullanılmamış noktaları bul
        std::vector<int> availableIndices;
        for (size_t i = 0; i < points.size(); ++i) {
            if (!used[i]) {
                availableIndices.push_back(i);
            }
        }
        
        if (availableIndices.size() < minPoints) {
            break;
        }
        
        Line bestLine;
        std::vector<int> bestInliers;
        int maxInliers = 0;
        
        // RANSAC iterasyonları
        for (int iter = 0; iter < maxIterations; ++iter)
        {
            // Rastgele 2 nokta seç
            std::uniform_int_distribution<> dis(0, availableIndices.size() - 1);
            int idx1 = availableIndices[dis(gen)];
            int idx2 = availableIndices[dis(gen)];
            
            if (idx1 == idx2) continue;
            
            // Bu iki noktadan doğru oluştur
            Line candidateLine;
            candidateLine.computeFromPoints(points[idx1], points[idx2]);
            
            // Inlier noktaları bul
            std::vector<int> inliers;
            for (int idx : availableIndices)
            {
                if (candidateLine.distanceToPoint(points[idx]) < distanceThreshold) {
                    inliers.push_back(idx);
                }
            }
            
            // En iyi modeli güncelle
            if (inliers.size() > maxInliers)
            {
                maxInliers = inliers.size();
                bestInliers = inliers;
                bestLine = candidateLine;
            }
        }
        
        // Yeterli inlier varsa doğruyu kaydet
        if (maxInliers >= minPoints)
        {
            bestLine.pointIndices = bestInliers;
            detectedLines.push_back(bestLine);
            
            // Kullanılan noktaları işaretle
            for (int idx : bestInliers) {
                used[idx] = true;
            }
        }
        else
        {
            break;
        }
    }
    
    return detectedLines;
}

// İki doğrunun kesişim noktasını hesapla
bool computeIntersection(const Line& line1, const Line& line2, Point2D& intersection)
{
    // a1*x + b1*y + c1 = 0
    // a2*x + b2*y + c2 = 0
    
    double det = line1.a * line2.b - line2.a * line1.b;
    
    // Paralel doğrular
    if (fabs(det) < 1e-10) {
        return false;
    }
    
    intersection.x = (line1.b * line2.c - line2.b * line1.c) / det;
    intersection.y = (line2.a * line1.c - line1.a * line2.c) / det;
    
    return true;
}

// İki doğru arasındaki açıyı hesapla (derece cinsinden)
double computeAngleBetweenLines(const Line& line1, const Line& line2)
{
    // Doğruların eğimlerini hesapla
    // ax + by + c = 0 -> y = -(a/b)x - c/b
    // Eğim m = -a/b
    
    double m1 = (fabs(line1.b) > 1e-10) ? -line1.a / line1.b : std::numeric_limits<double>::infinity();
    double m2 = (fabs(line2.b) > 1e-10) ? -line2.a / line2.b : std::numeric_limits<double>::infinity();
    
    double angle_rad;
    
    if (std::isinf(m1) || std::isinf(m2)) {
        // Dikey doğru durumu
        if (std::isinf(m1) && std::isinf(m2)) {
            angle_rad = 0; // Her ikisi de dikey
        } else if (std::isinf(m1)) {
            angle_rad = atan(fabs(m2));
        } else {
            angle_rad = atan(fabs(m1));
        }
    } else {
        angle_rad = atan(fabs((m1 - m2) / (1 + m1 * m2)));
    }
    
    double angle_deg = angle_rad * 180.0 / 3.141592653589793;
    
    // Açıyı 0-90 derece arasına normalize et
    if (angle_deg > 90) {
        angle_deg = 180 - angle_deg;
    }
    
    return angle_deg;
}

// Nokta ile robot arasındaki mesafeyi hesapla (robot 0,0'da)
double distanceToRobot(const Point2D& point)
{
    return sqrt(point.x * point.x + point.y * point.y);
}

// Ana program
int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Hata: TOML dosya adi belirtilmedi. " << "Kullanim: " << argv[0] << " <dosya_adi.toml>" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Hata: " << filename << " Dosyasi acilamadi." << std::endl;
        return 1;
    }
    
    LidarScanParams params;
    std::vector<double> ranges;
    std::string line;
    bool inScanSection = false;  
    bool readingRanges = false;

    while (std::getline(file, line)) 
    {
        size_t brckt;
        double rangeVal;
        trim(line);
        if (line.empty()) {
            continue;
        }

        if (line == "[scan]") { 
            inScanSection = true; 
            continue; 
        } else if (line[0] == '[') { 
            inScanSection = false; 
            continue; 
        }

        if (readingRanges) {
            if (line.find(']') != std::string::npos) {
                readingRanges = false; 
                line = line.substr(0, line.find(']'));
            }

            std::replace(line.begin(), line.end(), ',', ' ');
            std::stringstream ss(line);
            double rangevalue;
            while (ss >> rangevalue) {
                ranges.push_back(rangevalue);
            }
        } 
        else if (inScanSection)
        {
            if (!parseKeyValue(line, "angle_min", params.angle_min) &&
                !parseKeyValue(line, "angle_max", params.angle_max) &&
                !parseKeyValue(line, "angle_increment", params.angle_increment) &&
                !parseKeyValue(line, "range_min", params.range_min) &&
                !parseKeyValue(line, "range_max", params.range_max)) {
                
                if (line.rfind("ranges", 0) == 0) {
                    readingRanges = true;
                    brckt = line.find('[');
                    if (brckt != std::string::npos) {
                        line = line.substr(brckt + 1);
                        std::replace(line.begin(), line.end(), ',', ' ');
                        std::stringstream ss(line);
                        while (ss >> rangeVal) {
                            ranges.push_back(rangeVal);
                        }
                    }
                }
            }
        }
    }

    file.close();
    
    // Filtreleme ve Kartezyen koordinatlara dönüştürme
    std::vector<Point2D> points;
    for (size_t i = 0; i < ranges.size(); ++i)
    {
        double curRange = ranges[i];

        if (curRange < params.range_min || curRange > params.range_max) {
            continue;
        }

        double curAngle = params.angle_min + i * params.angle_increment;
        Point2D point;
        point.x = curRange * std::cos(curAngle);
        point.y = curRange * std::sin(curAngle);
        points.push_back(point);
    }
    
    std::cout << "=== ADIM 1: Veri Okuma ve Filtreleme ===" << std::endl;
    std::cout << "Filtrelenmis ve Donusturulmus Gecerli Noktalar: " << points.size() << " adet" << std::endl;
    
    // ADIM 2: Doğru Tespiti (RANSAC)
    std::cout << "\n=== ADIM 2: Dogru Tespiti (RANSAC) ===" << std::endl;
    std::vector<Line> detectedLines = detectLinesRANSAC(points, 8, 0.05, 1000);
    std::cout << "Tespit edilen dogru sayisi: " << detectedLines.size() << std::endl;
    
    for (size_t i = 0; i < detectedLines.size(); ++i) {
        std::cout << "Dogru " << (i+1) << ": " << detectedLines[i].pointIndices.size() 
                  << " nokta (" << std::fixed << std::setprecision(4)
                  << detectedLines[i].a << "x + " 
                  << detectedLines[i].b << "y + " 
                  << detectedLines[i].c << " = 0)" << std::endl;
    }
    
    // ADIM 3-4-5: Kesişim Analizi, Açı Kontrolü ve Mesafe Hesabı
    std::cout << "\n=== ADIM 3-4-5: Kesisim Analizi ve Aci Kontrolu ===" << std::endl;
    std::vector<Intersection> validIntersections;
    double minAngleThreshold = 60.0; // 60 derece ve üstü
    
    for (size_t i = 0; i < detectedLines.size(); ++i)
    {
        for (size_t j = i + 1; j < detectedLines.size(); ++j)
        {
            Point2D intersectionPoint;
            if (computeIntersection(detectedLines[i], detectedLines[j], intersectionPoint))
            {
                double angle = computeAngleBetweenLines(detectedLines[i], detectedLines[j]);
                double dist = distanceToRobot(intersectionPoint);
                
                std::cout << "\nDogru " << (i+1) << " ve Dogru " << (j+1) << " kesisiyor:" << std::endl;
                std::cout << "  Kesisim Noktasi: (" << std::fixed << std::setprecision(4) 
                          << intersectionPoint.x << ", " << intersectionPoint.y << ")" << std::endl;
                std::cout << "  Aci: " << angle << " derece" << std::endl;
                std::cout << "  Robot'a Mesafe: " << dist << " m" << std::endl;
                
                // Açı kontrolü
                if (angle >= minAngleThreshold)
                {
                    Intersection inter;
                    inter.point = intersectionPoint;
                    inter.line1_idx = i;
                    inter.line2_idx = j;
                    inter.angle_degrees = angle;
                    inter.distance_to_robot = dist;
                    validIntersections.push_back(inter);
                    
                    std::cout << "  >>> GECERLI (>= 60 derece)" << std::endl;
                }
            }
        }
    }
    
    std::cout << "\n=== OZET ===" << std::endl;
    std::cout << "Toplam Nokta: " << points.size() << std::endl;
    std::cout << "Tespit Edilen Dogru: " << detectedLines.size() << std::endl;
    std::cout << "Gecerli Kesisim (>= 60 derece): " << validIntersections.size() << std::endl;
    
    // ============================================================
    // GRAFİK İÇİN VERİLER - BURADAN AŞAĞISINI GRAFİK KODUNUZA EKLEYIN
    // ============================================================
    
    std::cout << "\n=== GRAFIK ICIN HAZIR VERI YAPILARI ===" << std::endl;
    std::cout << "Grafik kodunuza asagidaki verileri baglayin:\n" << std::endl;
    
    std::cout << "// 1. TUM NOKTALAR (gri veya acik renk noktalar olarak cizin)" << std::endl;
    std::cout << "// std::vector<Point2D> points; --> " << points.size() << " nokta" << std::endl;
    std::cout << "// Her nokta: points[i].x, points[i].y\n" << std::endl;
    
    std::cout << "// 2. TESPIT EDILEN DOGRULAR (farkli renklerle cizin)" << std::endl;
    std::cout << "// std::vector<Line> detectedLines; --> " << detectedLines.size() << " dogru" << std::endl;
    std::cout << "// Her dogru icin:" << std::endl;
    std::cout << "//   - detectedLines[i].pointIndices -> bu dogruyu olusturan noktalarin indeksleri" << std::endl;
    std::cout << "//   - Bu indekslerdeki points[] noktalarini birlestirerek dogru cizin" << std::endl;
    std::cout << "//   - Her dogruyu farkli renkle boyayin (ornek: yesil tonlari)" << std::endl;
    std::cout << "//   - Her dogruya etiket ekleyin: 'd1', 'd2', vb.\n" << std::endl;
    
    std::cout << "// 3. GECERLI KESISIM NOKTALARI (buyuk isaretleyicilerle cizin)" << std::endl;
    std::cout << "// std::vector<Intersection> validIntersections; --> " << validIntersections.size() << " kesisim" << std::endl;
    std::cout << "// Her kesisim icin:" << std::endl;
    std::cout << "//   - validIntersections[i].point.x, .y -> kesisim koordinatlari" << std::endl;
    std::cout << "//   - validIntersections[i].angle_degrees -> aci bilgisi" << std::endl;
    std::cout << "//   - validIntersections[i].distance_to_robot -> mesafe bilgisi" << std::endl;
    std::cout << "//   - Kesisim noktasini buyuk daire veya yildiz ile isaretleyin" << std::endl;
    std::cout << "//   - Yanina aci ve mesafe bilgisi yazin\n" << std::endl;
    
    std::cout << "// 4. ROBOT KONUMU (kirmizi buyuk nokta)" << std::endl;
    std::cout << "// Point2D robot = {0.0, 0.0}; // Robot her zaman orijinde" << std::endl;
    std::cout << "// Robot'u kirmizi buyuk daire ile isaretleyin\n" << std::endl;
    
    std::cout << "// 5. ROBOT'TAN KESISIMLERE CIZGILER (kesikli kirmizi cizgiler)" << std::endl;
    std::cout << "// Her gecerli kesisim icin robot (0,0)'dan kesisim noktasina kesikli cizgi cizin" << std::endl;
    std::cout << "// Cizgi uzerine mesafe degerini yazin\n" << std::endl;
    
    std::cout << "// 6. LEJANT (sag ust kose)" << std::endl;
    std::cout << "// - Tum Noktalar (gri)" << std::endl;
    std::cout << "// - Robot (kirmizi)" << std::endl;
    std::cout << "// - d1 noktasi (X nokta) - d1 ile gosterilen dogru" << std::endl;
    std::cout << "// - d2 noktasi (X nokta) - d2 ile gosterilen dogru" << std::endl;
    std::cout << "// - ... (her dogru icin)" << std::endl;
    std::cout << "// - 60+ derece Kesisim (buyuk daire/yildiz)" << std::endl;
    std::cout << "// - Mesafe Cizgisi (kesikli kirmizi)" << std::endl;
    
    return 0;
}