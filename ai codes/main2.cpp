// --- Gerekli Standart Kütüphanelerin Dahil Edilmesi ---
#include <iostream>      // Konsola girdi/çıktı işlemleri için (std::cout, std::cerr)
#include <fstream>       // Dosya okuma/yazma işlemleri için (std::ifstream)
#include <string>        // Metin işlemleri için (std::string)
#include <vector>        // Dinamik diziler (vektörler) kullanmak için (std::vector)
#include <sstream>       // Metinleri akış gibi işleyip parçalamak için (std::stringstream)
#include <cmath>         // Matematiksel fonksiyonlar için (cos, sin)
#include <iomanip>       // Çıktı formatlaması için (std::fixed, std::setprecision)
#include <algorithm>     // Standart algoritma fonksiyonları için (std::find_if, std::replace)

// Kartezyen (X, Y) koordinatlarını tutmak için basit bir veri yapısı (struct)
struct Point2D {
    double x; // Noktanın X koordinatı
    double y; // Noktanın Y koordinatı
};

// TOML dosyasından okunacak tarama (scan) parametrelerini bir arada tutan veri yapısı
struct LidarScanParams {
    double angle_min = 0.0;       // Taramanın başlangıç açısı (radyan)
    double angle_max = 0.0;       // Taramanın bitiş açısı (radyan)
    double angle_increment = 0.0; // Her ölçüm arasındaki açı artışı (radyan)
    double range_min = 0.0;       // Minimum geçerli mesafe (metre)
    double range_max = 0.0;       // Maksimum geçerli mesafe (metre)
};

// Bir metnin (string) başındaki ve sonundaki boşluk karakterlerini temizleyen yardımcı fonksiyon
void trim(std::string& s) {
    // Baştaki boşlukları sil
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
    // Sondaki boşlukları sil
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// "anahtar = değer" formatındaki bir satırı ayrıştırıp değeri ilgili değişkene atayan yardımcı fonksiyon
template<typename T>
bool parseKeyValue(const std::string& line, const std::string& key, T& value) {
    std::stringstream ss(line); // Satırı bir akışa dönüştürerek kolayca işlenmesini sağlar
    std::string current_key;    // Satırdan okunan anahtar kelime
    char separator;             // '=' ayıracını tutmak için
    
    ss >> current_key; // Satırın ilk kelimesini (anahtarı) oku
    if (current_key == key) { // Eğer okunan anahtar, aradığımız anahtar ise
        ss >> separator; // '=' karakterini oku ve atla
        if (separator == '=') {
            ss >> value; // Eşittirden sonraki değeri oku ve verilen 'value' değişkenine ata
            return true; // İşlem başarılı
        }
    }
    return false; // Aranan anahtar bulunamadı veya format yanlış
}

// --- Ana Program Fonksiyonu ---
int main(int argc, char* argv[]) {
    // Programın parametrik çalışması için dosya adının komut satırından verilip verilmediğini kontrol et
    if (argc < 2) { // Eğer argüman sayısı 2'den azsa (programın adı + dosya adı)
        std::cerr << "Hata: Lutfen bir TOML dosya adi belirtin." << std::endl; // Hata mesajı yazdır
        std::cerr << "Kullanim: " << argv[0] << " <dosya_adi.toml>" << std::endl; // Doğru kullanımı göster
        return 1; // Hata koduyla programdan çık
    }

    std::string filename = argv[1];   // Komut satırından gelen ikinci argümanı (dosya adını) al
    std::ifstream file(filename);     // Dosyayı okuma modunda açmak için bir dosya akışı nesnesi oluştur

    // Dosyanın başarılı bir şekilde açılıp açılmadığını kontrol et
    if (!file.is_open()) {
        std::cerr << "Hata: Dosya acilamadi -> " << filename << std::endl; // Hata mesajı yazdır
        return 1; // Hata koduyla programdan çık
    }

    // Okunacak verileri saklamak için değişkenleri ve yapıları tanımla
    LidarScanParams params;             // Tarama parametrelerini tutacak nesne
    std::vector<double> ranges;         // 'ranges' dizisindeki tüm mesafe ölçümlerini tutacak vektör
    std::string line;                   // Dosyadan satır satır okuma yapmak için kullanılacak string
    
    // Dosyayı ayrıştırırken hangi bölümde olduğumuzu takip etmek için durum değişkenleri
    bool in_scan_section = false;  // Mevcut satırın [scan] bölümünde olup olmadığını belirtir
    bool reading_ranges = false;   // 'ranges' dizisini okumaya başlayıp başlamadığımızı belirtir

    // --- Dosyayı Manuel Olarak Satır Satır Ayrıştırma (Parsing) Bloğu ---
    while (std::getline(file, line)) { // Dosyanın sonuna gelene kadar her satırı 'line' değişkenine oku
        trim(line); // Okunan satırın başındaki ve sonundaki boşlukları temizle
        
        // Eğer satır boşsa veya yorum satırıysa (# ile başlıyorsa), bu satırı atla ve bir sonrakine geç
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // Satırın bir bölüm başlığı olup olmadığını kontrol et
        if (line == "[scan]") { // Eğer '[scan]' bölümüne geldiysek
            in_scan_section = true; // 'scan' bölümünde olduğumuzu işaretle
            continue; // Bu satırla işimiz bitti, bir sonrakine geç
        } else if (line[0] == '[') { // Eğer başka bir bölüm başlığına geldiysek
            in_scan_section = false; // Artık 'scan' bölümünde değiliz
            continue; // Bu satırla işimiz bitti, bir sonrakine geç
        }

        // 'ranges' dizisinin değerlerini okuma mantığı
        if (reading_ranges) {
            // Eğer satırda dizinin sonunu belirten ']' karakteri varsa
            if (line.find(']') != std::string::npos) {
                reading_ranges = false; // 'ranges' dizisini okumayı bitirdiğimizi işaretle
                line = line.substr(0, line.find(']')); // Satırın sadece ']' karakterinden önceki kısmını al
            }

            // Satırdaki sayıları kolayca ayrıştırabilmek için virgülleri (,) boşlukla (' ') değiştir
            std::replace(line.begin(), line.end(), ',', ' '); 
            std::stringstream ss(line); // Değiştirilmiş satırı bir string akışına dönüştür
            double range_value;
            while (ss >> range_value) { // Akıştan okunacak sayı olduğu sürece
                ranges.push_back(range_value); // Okunan sayıyı 'ranges' vektörüne ekle
            }
        } 
        // Eğer '[scan]' bölümündeysek ve henüz 'ranges' dizisini okumuyorsak
        else if (in_scan_section) {
            // Anahtar-değer çiftlerini oku (angle_min, range_max vb.)
            if (!parseKeyValue(line, "angle_min", params.angle_min) &&
                !parseKeyValue(line, "angle_max", params.angle_max) &&
                !parseKeyValue(line, "angle_increment", params.angle_increment) &&
                !parseKeyValue(line, "range_min", params.range_min) &&
                !parseKeyValue(line, "range_max", params.range_max)) {
                
                // Eğer yukarıdaki anahtarlardan hiçbiri değilse, 'ranges' dizisinin başlangıcı mı diye kontrol et
                if (line.rfind("ranges", 0) == 0) { // Satır "ranges" ile başlıyorsa
                    reading_ranges = true; // 'ranges' dizisini okumaya başladığımızı işaretle
                    size_t start_pos = line.find('['); // Dizinin başlangıcını belirten '[' karakterini bul
                    if (start_pos != std::string::npos) {
                        line = line.substr(start_pos + 1); // Satırın '[' karakterinden sonraki kısmını al
                        // Bu ilk satırdaki sayıları da işle
                        std::replace(line.begin(), line.end(), ',', ' ');
                        std::stringstream ss(line);
                        double range_value;
                        while (ss >> range_value) {
                            ranges.push_back(range_value);
                        }
                    }
                }
            }
        }
    }

    file.close(); // Dosya ile işimiz bitti, kapatıyoruz.

    // --- Okunan Verileri İşleme ve Dönüştürme Bloğu ---
    std::vector<Point2D> points; // Hesaplanan geçerli noktaları saklayacağımız vektör
    
    // 'ranges' vektöründeki her bir mesafe ölçümü için döngü
    for (size_t i = 0; i < ranges.size(); ++i) {
        double current_range = ranges[i]; // Mevcut mesafe değerini al

        // 1. Filtreleme: Geçersiz değerleri (NaN benzeri veya menzil dışı) atla
        if (current_range < params.range_min || current_range > params.range_max) {
            continue; // Bu ölçüm geçersiz, döngünün bir sonraki adımına geç
        }

        // 2. Açı Hesaplama: Bu ölçümün hangi açıya ait olduğunu hesapla
        double current_angle = params.angle_min + i * params.angle_increment;

        // 3. Kutupsal Koordinatlardan Kartezyen Koordinatlara Dönüştürme
        // Robotun konumu merkez (0, 0) olarak kabul edilir. Formüller: x = r*cos(θ), y = r*sin(θ)
        Point2D point; // Yeni bir nokta nesnesi oluştur
        point.x = current_range * std::cos(current_angle); // X koordinatını hesapla
        point.y = current_range * std::sin(current_angle); // Y koordinatını hesapla
        
        points.push_back(point); // Hesaplanan geçerli noktayı 'points' vektörüne ekle
    }
    
    // --- Sonuçları Ekrana Yazdırma Bloğu ---
    std::cout << "Filtrelenmis ve Donusturulmus Gecerli Noktalar (" << points.size() << " adet):" << std::endl;
    // Çıktıdaki ondalık sayıların formatını ayarla (noktadan sonra 4 basamak göster)
    std::cout << std::fixed << std::setprecision(4); 

    // 'points' vektöründeki her bir nokta için döngü
    for (const auto& p : points) {
        // Her noktanın X ve Y koordinatlarını formatlı bir şekilde yazdır
        std::cout << "(X: " << p.x << ", Y: " << p.y << ")" << std::endl;
    }

    return 0; // Program başarıyla tamamlandı
}