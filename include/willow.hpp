/**
 * @file willow.hpp
 * @brief Willow 5 Runtime - C++ Edge SDK
 * @version 5.2 (Production Hardened)
 * @license MIT
 * 
 * This file is a dependency-free, header-only library replicating the 
 * Willow V4.0 Cloud Oracle mathematics. It enforces strict RAM-Only 
 * execution to protect proprietary .int8 signatures and utilizes a 
 * zero-allocation ring buffer for embedded performance.
 */

 #ifndef WILLOW_HPP
 #define WILLOW_HPP
 
 #include <vector>
 #include <cmath>
 #include <cstdint>
 #include <cstring>
 #include <stdexcept>
 #include <algorithm>
 #include <iostream>
 #include <string>
 #include <map>
 #include <fstream>
 #include <memory>
 #include <limits>
 
 #ifdef WILLOW_ENABLE_CURL
 #include <curl/curl.h>
 #endif
 
 namespace willow {
 
     // ========================================================================
     // 1. CORE DATA STRUCTURES & TOPOLOGY
     // ========================================================================
 
     struct Point3D {
         float x, y, z, visibility;
     };
 
     struct Quaternion {
         float w, x, y, z;
     };
 
     using Skeleton = std::vector<Point3D>; // Must be exactly 75 points
 
     struct DetectionResult {
         int start_idx;
         int end_idx;
         float confidence;
         float speed_variance_pct;
     };
 
     enum ZoneBitmask {
         HEAD  = 1 << 0,
         TORSO = 1 << 1,
         ARMS  = 1 << 2,
         HANDS = 1 << 3,
         LEGS  = 1 << 4,
         FEET  = 1 << 5
     };
 
     const std::map<ZoneBitmask, std::vector<int>> TOPOLOGY_ZONES = {
         {HEAD,  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}},
         {TORSO, {11, 12, 23, 24}},
         {ARMS,  {13, 14, 15, 16}},
         {HANDS, {33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53,
                  54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74}},
         {LEGS,  {25, 26, 27, 28}},
         {FEET,  {29, 30, 31, 32}}
     };
 
     // ========================================================================
     // 2. INTERNAL MATH UTILITIES
     // ========================================================================
     
     namespace math {
         inline Point3D vec_sub(const Point3D& a, const Point3D& b) {
             // We preserve the visibility of the primary point (a) during subtraction
             return {a.x - b.x, a.y - b.y, a.z - b.z, a.visibility};
         }
 
         inline Point3D vec_cross(const Point3D& a, const Point3D& b) {
             return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x, std::min(a.visibility, b.visibility)};
         }
 
         inline float vec_dot(const Point3D& a, const Point3D& b) {
             return a.x*b.x + a.y*b.y + a.z*b.z;
         }
 
         inline float vec_mag(const Point3D& a) {
             return std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
         }
 
         inline Point3D vec_norm(const Point3D& a) {
             float m = vec_mag(a);
             if (m < 1e-6f) return {0.0f, 0.0f, 0.0f, a.visibility};
             // Preserve original visibility during normalization
             return {a.x/m, a.y/m, a.z/m, a.visibility};
         }
     }
 
     // ========================================================================
     // 3. MODEL DEFINITION & PARSERS
     // ========================================================================
 
     class Model {
     public:
         uint32_t version;
         uint32_t zone_mask;
         float scale_factor;
         float overlap_tolerance;
         float dtw_sensitivity;
         float tempo_variance;
         std::vector<std::vector<float>> signature;
         std::vector<int> active_indices;
 
         /**
          * @brief SECURE DEFAULT: Ingests an .int8 payload directly from a memory buffer.
          * Ensures Little-Endian interpretation regardless of host hardware.
          */
         static Model load_from_memory(const std::vector<uint8_t>& buffer) {
             if (buffer.size() < 24) throw std::runtime_error("Buffer too small for Willow V4.0 header.");
 
             Model m;
             size_t offset = 0;
 
             // Explicit bit-shifting guarantees Strict Little-Endian parsing,
             // preventing Endianness corruption on ARM/PowerPC edge devices.
             auto read_u32 = [&](uint32_t& val) {
                 val = static_cast<uint32_t>(buffer[offset]) |
                       (static_cast<uint32_t>(buffer[offset+1]) << 8) |
                       (static_cast<uint32_t>(buffer[offset+2]) << 16) |
                       (static_cast<uint32_t>(buffer[offset+3]) << 24);
                 offset += 4;
             };
 
             auto read_f32 = [&](float& val) {
                 uint32_t temp;
                 read_u32(temp);
                 std::memcpy(&val, &temp, sizeof(float)); // Safe type-punning
             };
 
             read_u32(m.version);
             if (m.version != 40) throw std::runtime_error("Unsupported model version. Expected 40 (V4.0).");
 
             read_u32(m.zone_mask);
             read_f32(m.scale_factor);
             read_f32(m.overlap_tolerance);
             
             read_f32(m.dtw_sensitivity);
             // Division-By-Zero safeguard
             if (m.dtw_sensitivity <= 1e-5f) m.dtw_sensitivity = 1e-5f; 
 
             read_f32(m.tempo_variance);
 
             // Build active topology
             for (const auto& pair : TOPOLOGY_ZONES) {
                 if (m.zone_mask & pair.first) {
                     m.active_indices.insert(m.active_indices.end(), pair.second.begin(), pair.second.end());
                 }
             }
             
             // Filter duplicates to prevent index array explosions if zones overlap
             std::sort(m.active_indices.begin(), m.active_indices.end());
             m.active_indices.erase(std::unique(m.active_indices.begin(), m.active_indices.end()), m.active_indices.end());
 
             int num_features = (m.active_indices.size() * (m.active_indices.size() - 1)) / 2;
             if (num_features == 0) throw std::runtime_error("Zero valid features identified by zone mask.");
             
             int num_frames = (buffer.size() - 24) / num_features;
 
             if ((buffer.size() - 24) % num_features != 0) {
                 throw std::runtime_error("Corrupted payload: Int8 array does not align with active topology.");
             }
 
             // De-quantize Int8 -> Float32
             m.signature.resize(num_frames, std::vector<float>(num_features));
             for (int f = 0; f < num_frames; ++f) {
                 for (int i = 0; i < num_features; ++i) {
                     int8_t val;
                     std::memcpy(&val, buffer.data() + offset++, 1);
                     m.signature[f][i] = (static_cast<float>(val) / 127.0f) * m.scale_factor;
                 }
             }
             return m;
         }
 
         /**
          * @brief OPTIONAL LOCAL LOAD: For local prototyping.
          */
         static Model load_from_file(const std::string& filepath) {
             std::ifstream file(filepath, std::ios::binary | std::ios::ate);
             if (!file.is_open()) throw std::runtime_error("Could not open file: " + filepath);
             std::streamsize size = file.tellg();
             file.seekg(0, std::ios::beg);
             std::vector<uint8_t> buffer(size);
             if (file.read(reinterpret_cast<char*>(buffer.data()), size)) return load_from_memory(buffer);
             throw std::runtime_error("Failed to read file: " + filepath);
         }
     };
 
     // ========================================================================
     // 4. CLIENT (PROVISIONING)
     // ========================================================================
 
     class Client {
     private:
         std::string api_key;
         std::string customer_id;
         std::string base_url = "https://api.willowdynamics.com/v1/models/";
 
     public:
         Client(const std::string& key, const std::string& cid) : api_key(key), customer_id(cid) {}
 
         #ifdef WILLOW_ENABLE_CURL
         static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp) {
             size_t realsize = size * nmemb;
             auto *mem = static_cast<std::vector<uint8_t>*>(userp);
             auto *data = static_cast<uint8_t*>(contents);
             mem->insert(mem->end(), data, data + realsize);
             return realsize;
         }
 
         // Custom Deleters for strict RAII Compliance
         struct CurlDeleter {
             void operator()(CURL* curl) const { if (curl) curl_easy_cleanup(curl); }
         };
         struct CurlSlistDeleter {
             void operator()(curl_slist* slist) const { if (slist) curl_slist_free_all(slist); }
         };
 
         std::vector<uint8_t> fetch_model(const std::string& model_id) const {
             std::unique_ptr<CURL, CurlDeleter> curl(curl_easy_init());
             if (!curl) throw std::runtime_error("Failed to initialize CURL.");
 
             std::vector<uint8_t> chunk;
             std::string url = base_url + model_id + ".int8";
             
             curl_slist* headers_raw = NULL;
             headers_raw = curl_slist_append(headers_raw, ("Authorization: Bearer " + api_key).c_str());
             headers_raw = curl_slist_append(headers_raw, ("X-Customer-ID: " + customer_id).c_str());
             std::unique_ptr<curl_slist, CurlSlistDeleter> headers(headers_raw);
 
             curl_easy_setopt(curl.get(), CURLOPT_URL, url.c_str());
             curl_easy_setopt(curl.get(), CURLOPT_HTTPHEADER, headers.get());
             curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
             curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, (void *)&chunk);
             
             // CRITICAL FIX: Force curl to throw an error on 401/404 HTTP Responses
             // Prevents feeding HTML error pages into the .int8 binary parser.
             curl_easy_setopt(curl.get(), CURLOPT_FAILONERROR, 1L);
 
             CURLcode res = curl_easy_perform(curl.get());
             if (res != CURLE_OK) {
                 throw std::runtime_error("curl_easy_perform() failed: " + std::string(curl_easy_strerror(res)));
             }
             
             return chunk;
         }
         #else
         std::vector<uint8_t> fetch_model(const std::string& model_id) const {
             throw std::runtime_error("Client fetch requires libcurl. Compile with -DWILLOW_ENABLE_CURL.");
         }
         #endif
     };
 
     // ========================================================================
     // 5. TRANSFORMS (COORDINATE BRIDGES)
     // ========================================================================
 
     class Transforms {
     public:
         // Oracle Standard: MediaPipe (Y-Down, Z-Forward) -> Unity (Y-Up, Z-Forward)
         static Point3D to_unity(const Point3D& pt) { return { pt.x, -pt.y, pt.z, pt.visibility }; }
 
         // Oracle Standard: MediaPipe (Y-Down, Z-Forward) -> ROS (Z-Up, X-Forward)
         static Point3D to_ros(const Point3D& pt) { return { pt.z, -pt.x, -pt.y, pt.visibility }; }
     };
 
     // ========================================================================
     // 6. DETECTOR (ORACLE-PARITY ENGINE)
     // ========================================================================
 
     class Detector {
     private:
         Model model;
         
         // Zero-Allocation Ring Buffers
         std::vector<float> D_prev;
         std::vector<int> S_prev;
         std::vector<float> D_curr;
         std::vector<int> S_curr;
         std::vector<float> rdm_buffer;
         
         float sim_t2 = 0.0f;
         float sim_t1 = 0.0f;
         float sim_t0 = 0.0f;
         
         int length_t1 = 0;
         int current_frame = 0;
         const float VISIBILITY_THRESHOLD = 0.5f;
 
         std::vector<DetectionResult> confirmed_detections;
 
         float calculate_torso_length(const Skeleton& skel) const {
             auto s_mid = Point3D{ (skel[11].x + skel[12].x)/2.0f, (skel[11].y + skel[12].y)/2.0f, (skel[11].z + skel[12].z)/2.0f, 1.0f };
             auto h_mid = Point3D{ (skel[23].x + skel[24].x)/2.0f, (skel[23].y + skel[24].y)/2.0f, (skel[23].z + skel[24].z)/2.0f, 1.0f };
             float dx = s_mid.x - h_mid.x;
             float dy = s_mid.y - h_mid.y;
             float dz = s_mid.z - h_mid.z;
             float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
             return (dist < 0.01f) ? 0.01f : dist;
         }
 
         void extract_rdm(const Skeleton& skel, std::vector<float>& out_rdm) const {
             out_rdm.clear(); // Resets size to 0 but keeps capacity intact
             float scale = 1.0f;
             
             if (model.zone_mask & TORSO) {
                 scale = calculate_torso_length(skel);
             }
 
             for (size_t i = 0; i < model.active_indices.size(); ++i) {
                 for (size_t j = i + 1; j < model.active_indices.size(); ++j) {
                     auto p1 = skel[model.active_indices[i]];
                     auto p2 = skel[model.active_indices[j]];
                     
                     // Visibility Masking. Prevents occluded sensors 
                     // from injecting massive false distances into the RDM.
                     if (p1.visibility < VISIBILITY_THRESHOLD || p2.visibility < VISIBILITY_THRESHOLD ||
                        (p1.x == 0.0f && p1.y == 0.0f && p1.z == 0.0f) ||
                        (p2.x == 0.0f && p2.y == 0.0f && p2.z == 0.0f)) {
                         out_rdm.push_back(0.0f);
                     } else {
                         float dx = p1.x - p2.x;
                         float dy = p1.y - p2.y;
                         float dz = p1.z - p2.z;
                         out_rdm.push_back(std::sqrt(dx*dx + dy*dy + dz*dz) / scale);
                     }
                 }
             }
         }
 
     public:
         Detector(const Model& m) : model(m) {
             int M = model.signature.size();
             
             D_prev.resize(M + 1, std::numeric_limits<float>::infinity());
             S_prev.resize(M + 1, 0);
             D_curr.resize(M + 1, std::numeric_limits<float>::infinity());
             S_curr.resize(M + 1, 0);
             
             D_prev[0] = 0.0f;
             
             int num_features = (m.active_indices.size() * (m.active_indices.size() - 1)) / 2;
             rdm_buffer.reserve(num_features);
         }
 
         bool push_frame(const Skeleton& skel, DetectionResult& out_result) {
             if (skel.size() != 75) throw std::invalid_argument("Skeleton must contain 75 joints.");
 
             // Zero-allocation RDM extraction
             extract_rdm(skel, rdm_buffer); 
             int M = model.signature.size();
 
             // Zero-allocation buffer reset
             std::fill(D_curr.begin(), D_curr.end(), std::numeric_limits<float>::infinity());
             std::fill(S_curr.begin(), S_curr.end(), 0);
 
             D_curr[0] = 0.0f;
             S_curr[0] = current_frame;
 
             for (int j = 1; j <= M; ++j) {
                 float dist = 0.0f;
                 for (size_t k = 0; k < rdm_buffer.size(); ++k) {
                     float diff = rdm_buffer[k] - model.signature[j-1][k];
                     dist += diff * diff;
                 }
                 float cost = std::sqrt(dist);
 
                 float prev_cost;
                 if (D_prev[j-1] <= D_prev[j] && D_prev[j-1] <= D_curr[j-1]) {
                     prev_cost = D_prev[j-1]; S_curr[j] = S_prev[j-1];
                 } else if (D_prev[j] <= D_curr[j-1]) {
                     prev_cost = D_prev[j]; S_curr[j] = S_prev[j];
                 } else {
                     prev_cost = D_curr[j-1]; S_curr[j] = S_curr[j-1];
                 }
                 D_curr[j] = cost + prev_cost;
             }
 
             float final_cost = D_curr[M] / M;
             int length = current_frame - S_curr[M];
 
             // Safely normalizes score due to constructor Div-By-Zero clamping
             sim_t0 = std::max(0.0f, std::min(1.0f, 1.0f - (final_cost / model.dtw_sensitivity)));
 
             bool detection_fired = false;
 
             // NMS: 1-Frame Delayed Falling Edge Peak Detection
             if (sim_t1 >= 0.50f && sim_t1 > sim_t0 && sim_t1 > sim_t2) {
                 
                 int start_idx = (current_frame - 1) - length_t1;
                 int end_idx = current_frame - 1;
                 
                 bool speed_violation = false;
                 float speed_variance = 0.0f;
                 if (model.tempo_variance > 0.0f) {
                     speed_variance = std::abs((float)length_t1 - (float)M) / (float)M;
                     if (speed_variance > model.tempo_variance) speed_violation = true;
                 }
 
                 if (!speed_violation) {
                     bool overlaps = false;
                     int dur = std::max(1, end_idx - start_idx);
 
                     for (const auto& prev_det : confirmed_detections) {
                         int latest_start = std::max(start_idx, prev_det.start_idx);
                         int earliest_end = std::min(end_idx, prev_det.end_idx);
                         int overlap_dur = std::max(0, earliest_end - latest_start);
                         
                         if (overlap_dur > 0) {
                             float overlap_ratio = (float)overlap_dur / (float)dur;
                             if (overlap_ratio > model.overlap_tolerance) {
                                 overlaps = true;
                                 break;
                             }
                         }
                     }
 
                     if (!overlaps) {
                         out_result = { start_idx, end_idx, sim_t1, speed_variance };
                         confirmed_detections.push_back(out_result);
                         detection_fired = true;
                     }
                 }
             }
 
             // Zero-allocation pointer swap
             std::swap(D_prev, D_curr);
             std::swap(S_prev, S_curr);
             
             sim_t2 = sim_t1;
             sim_t1 = sim_t0;
             length_t1 = length;
             current_frame++;
 
             return detection_fired;
         }
     };
 
     // ========================================================================
     // 7. EVALUATOR (PHYSICS KINEMATICS)
     // ========================================================================
 
     class Evaluator {
     public:
         // Derivatives are computed as true 3D Vectors to preserve 
         // centripetal acceleration and directional changes.
 
         static Point3D calculate_velocity(const Point3D& p1, const Point3D& p2, float dt) {
             if (dt <= 0.0f) return {0.0f, 0.0f, 0.0f, 0.0f};
             Point3D v = math::vec_sub(p2, p1);
             return {v.x / dt, v.y / dt, v.z / dt, std::min(p1.visibility, p2.visibility)};
         }
 
         static Point3D calculate_acceleration(const Point3D& v1, const Point3D& v2, float dt) {
             if (dt <= 0.0f) return {0.0f, 0.0f, 0.0f, 0.0f};
             Point3D a = math::vec_sub(v2, v1);
             return {a.x / dt, a.y / dt, a.z / dt, std::min(v1.visibility, v2.visibility)};
         }
 
         static Point3D calculate_jerk(const Point3D& a1, const Point3D& a2, float dt) {
             if (dt <= 0.0f) return {0.0f, 0.0f, 0.0f, 0.0f};
             Point3D j = math::vec_sub(a2, a1);
             return {j.x / dt, j.y / dt, j.z / dt, std::min(a1.visibility, a2.visibility)};
         }
 
         // Helper to extract magnitude when final scalar metrics are needed
         static float get_magnitude(const Point3D& vector) {
             return math::vec_mag(vector);
         }
     };
 
     // ========================================================================
     // 8. RETARGETER (KINEMATICS & QUATERNIONS)
     // ========================================================================
 
     class Retargeter {
     public:
         static Quaternion compute_lookat_rotation(const Point3D& position, const Point3D& target, const Point3D& up_ref) {
             Point3D forward = math::vec_norm(math::vec_sub(target, position));
             
             // Gimbal Lock / Colinear Collapse Safeguard
             Point3D right = math::vec_cross(up_ref, forward);
             if (math::vec_mag(right) < 1e-5f) {
                 // Forward vector is parallel to up_ref. We swap up_ref to prevent basis collapse.
                 Point3D alt_up = {1.0f, 0.0f, 0.0f, 1.0f}; // Fallback to X-axis
                 if (std::abs(forward.x) > 0.99f) {
                     alt_up = {0.0f, 0.0f, 1.0f, 1.0f}; // If forward is along X, fallback to Z
                 }
                 right = math::vec_cross(alt_up, forward);
             }
             
             right = math::vec_norm(right);
             Point3D up = math::vec_norm(math::vec_cross(forward, right));
 
             // std::max(0.0f, ...) safeguards against negative trace square roots
             float trace = right.x + up.y + forward.z;
             Quaternion q;
 
             if (trace > 0.0f) {
                 float s = 0.5f / std::sqrt(std::max(0.0f, trace + 1.0f));
                 q.w = 0.25f / s;
                 q.x = (up.z - forward.y) * s;
                 q.y = (forward.x - right.z) * s;
                 q.z = (right.y - up.x) * s;
             } else {
                 if (right.x > up.y && right.x > forward.z) {
                     float s = 2.0f * std::sqrt(std::max(0.0f, 1.0f + right.x - up.y - forward.z));
                     q.w = (up.z - forward.y) / s;
                     q.x = 0.25f * s;
                     q.y = (up.x + right.y) / s;
                     q.z = (forward.x + right.z) / s;
                 } else if (up.y > forward.z) {
                     float s = 2.0f * std::sqrt(std::max(0.0f, 1.0f + up.y - right.x - forward.z));
                     q.w = (forward.x - right.z) / s;
                     q.x = (up.x + right.y) / s;
                     q.y = 0.25f * s;
                     q.z = (forward.y + up.z) / s;
                 } else {
                     float s = 2.0f * std::sqrt(std::max(0.0f, 1.0f + forward.z - right.x - up.y));
                     q.w = (right.y - up.x) / s;
                     q.x = (forward.x + right.z) / s;
                     q.y = (forward.y + up.z) / s;
                     q.z = 0.25f * s;
                 }
             }
             return q;
         }
     };
 
 } // namespace willow
 
 #endif // WILLOW_HPP