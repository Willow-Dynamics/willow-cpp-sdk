#include "../include/willow.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace willow;

Skeleton fetch_live_camera_skeleton() {
    // Mocking a live camera feed returning a 75-point skeleton
    Skeleton skel(75, {0.0f, 0.0f, 0.0f, 1.0f});
    skel[11] = {-1.0f, 0.0f, 0.0f, 1.0f}; 
    skel[12] = {1.0f, 0.0f, 0.0f, 1.0f};  
    skel[23] = {-1.0f, -2.0f, 0.0f, 1.0f}; 
    skel[24] = {1.0f, -2.0f, 0.0f, 1.0f};
    return skel;
}

int main() {
    std::cout << "Starting Example 2: Live Camera Stream\n";

    // 1. Mock V4.0 .int8 Buffer Payload
    std::vector<uint8_t> buffer(25, 0);
    uint32_t magic = 40, mask = TORSO;
    float scale = 1.0f, overlap = 0.25f, dtw = 3.0f, tempo = 0.20f;

    // Safely pack data enforcing Little-Endian byte order
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
    
    write_u32(magic, 0); write_u32(mask, 4);
    write_f32(scale, 8); write_f32(overlap, 12);
    write_f32(dtw, 16);  write_f32(tempo, 20);
    buffer[24] = 64; // Mock RDM value

    try {
        // 2. Initialize Engine
        Detector detector(Model::load_from_memory(buffer));
        DetectionResult result;

        // 3. Process Live Stream Loop
        for (int i = 0; i < 10; i++) {
            Skeleton skel = fetch_live_camera_skeleton();

            // --- THE DUAL PATH PIPELINE ---

            // PATH A: VISUAL RENDERING 
            // Transform coordinates for your 3D engine (e.g., Unity Y-Up)
            Skeleton unity_skel = skel;
            for (auto& pt : unity_skel) pt = Transforms::to_unity(pt);
            // Example: my_unity_renderer.update_avatar(unity_skel);

            // PATH B: MATHEMATICAL DETECTION
            // Pass the RAW MediaPipe skeleton to the Willow Detector
            // CRITICAL: The detector relies on the original Y-Down coordinate space!
            if (detector.push_frame(skel, result)) {
                std::cout << "\n[!] ACTION DETECTED [!]\n";
                std::cout << "  Start Frame: " << result.start_idx << "\n";
                std::cout << "  End Frame:   " << result.end_idx << "\n";
                std::cout << "  Confidence:  " << result.confidence * 100.0f << "%\n";
            } else {
                std::cout << "."; std::flush(std::cout);
            }
            
            // Simulate ~30 FPS camera latency
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    } catch (const std::exception& e) {
        std::cerr << "\n[FATAL] Live Stream Interrupted by SDK Error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\nStream Ended.\n";
    return 0;
}