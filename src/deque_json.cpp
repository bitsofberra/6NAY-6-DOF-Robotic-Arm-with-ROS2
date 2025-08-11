#include <iostream>
#include <fstream>
#include <deque>
#include "pubsub_srvcli/include/pubsub_srvcli/json.hpp"

using json = nlohmann::json;

struct Vektor {
    double x, y, z;
};

int main() {
    std::ifstream dosya("data.json");
    if (!dosya.is_open()) {
        std::cerr << "data.json dosyası bulunamadı.\n";
        return 1;
    }

    json j;
    dosya >> j;
    dosya.close();

    std::deque<Vektor> veriler;

    // JSON verilerini deque içine yükle
    for (const auto& item : j) {
        veriler.push_back({ item["x"], item["y"], item["z"] });
    }

    // İlk okunan verileri göster
    std::cout << "Başlangıç verileri:\n";
    for (auto& v : veriler) {
        std::cout << "(" << v.x << ", " << v.y << ", " << v.z << ")\n";
    }

    // Dinamik ekleme
    std::cout << "\nYeni veri ekleniyor...\n";
    veriler.push_back({ 7.0, 8.0, 9.0 });
    veriler.push_back({ 1.0, 2.0, 3.0 });
    veriler.push_back({ 4.0, 5.0, 6.0 });
    veriler.push_back({ 7.0, 8.0, 9.0 });

    // Sadece son 5 veriyi tutmak (kaydırmalı pencere mantığı)
    if (veriler.size() > 5) {
        veriler.pop_front();
    }

    // Güncel verileri göster
    std::cout << "Güncel deque:\n";
    for (auto& v : veriler) {
        std::cout << "(" << v.x << ", " << v.y << ", " << v.z << ")\n";
    }

    // JSON dosyasını yeni verilerle güncelle
    json yeni_json = json::array();
    for (auto& v : veriler) {
        yeni_json.push_back({ {"x", v.x}, {"y", v.y}, {"z", v.z} });
    }

    std::ofstream cikti("veri.json");
    cikti << yeni_json.dump(4); // 4: okunabilir format
    cikti.close();

    return 0;
}
