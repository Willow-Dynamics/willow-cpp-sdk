#include "../include/willow.hpp"
#include <iostream>

using namespace willow;

int main() {
    std::cout << "Starting Example 1: Basic Detection\n";

    // 1. Mock V4.0 .int8 Buffer Payload
    std::vector<uint8_t> buffer(25, 0);
    uint32_t magic = 40, mask = TORSO;
    float scale = 1.0f, overlap = 0.25f, dtw = 3.0f, tempo = 0.20f;

    // Utilize bit-shifting to safely pack data (mirroring internal Little-Endian parser)
    auto write_u32 = [&](uint32_t val, int offset) {
        buffer[offset] = val & 0xFF;
        buffer[offset+1] = (val >> 8) & 0xFF;
        buffer[offset+2] = (val >> 16) & 0xFF;
        buffer[offset+3] = (val >> 24) & 0xFF;
    };
    
    // Safely type-pun float to uint32_t and write strictly as Little-Endian
    auto write_f32 = [&](float val, int offset) {
        uint32_t temp;
        std::memcpy(&temp, &val, sizeof(float)); 
        write_u32(temp, offset); 
    };
    
    write_u32(magic, 0); 
    write_u32(mask, 4);
    write_f32(scale, 8);
    write_f32(overlap, 12);
    write_f32(dtw, 16);
    write_f32(tempo, 20);
    buffer[24] = 127; // Mock RDM value

    // 2. Initialize Engine
    try {
        Model model = Model::load_from_memory(buffer);
        Detector detector(model);

        // 3. Process frames
        Skeleton skel(75, {0.0f, 0.0f, 0.0f, 1.0f});
        skel[11] = {-1.0f, 0.0f, 0.0f, 1.0f}; 
        skel[12] = {1.0f, 0.0f, 0.0f, 1.0f};  
        skel[23] = {-1.0f, -2.0f, 0.0f, 1.0f}; 
        skel[24] = {1.0f, -2.0f, 0.0f, 1.0f};  

        DetectionResult result;
        for (int i = 0; i < 3; i++) {
            if (detector.push_frame(skel, result)) {
                std::cout << "Action Confirmed at Frame " << i << "!\n";
                std::cout << "Confidence: " << result.confidence << "\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Willow SDK Exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}