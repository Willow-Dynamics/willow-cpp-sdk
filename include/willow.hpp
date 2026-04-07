/**
 * @file willow.hpp
 * @brief Willow 5 Runtime - C++ Edge SDK
 * @version 5.4.0 (Production Hardened)
 * @license MIT
 * 
 * This file is a dependency-free, header-only library replicating the 
 * Willow V4.0 and V4.1 Cloud Oracle mathematics.
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

    using Skeleton = std::vector<Point3D>; // Accepts 75 or 76 points

    struct DetectionResult {
        int start_idx;
        int end_idx;
        float confidence;
        float speed_variance_pct;
    };

    enum ZoneBitmask {
        HEAD   = 1 << 0,
        TORSO  = 1 << 1,
        ARMS   = 1 << 2,
        HANDS  = 1 << 3,
        LEGS   = 1 << 4,
        FEET   = 1 << 5,
        OBJECT = 1 << 6  // V4.1 Physics Reference Object
    };

    const std::map<ZoneBitmask, std::vector<int>> TOPOLOGY_ZONES = {
        {HEAD,   {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}},
        {TORSO,  {11, 12, 23, 24}},
        {ARMS,   {13, 14, 15, 16}},
        {HANDS,  {33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53,
                  54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74}},
        {LEGS,   {25, 26, 27, 28}},
        {FEET,   {29, 30, 31, 32}},
        {OBJECT, {75}}
    };

    // ========================================================================
    // 2. INTERNAL MATH UTILITIES
    // ========================================================================
    
    namespace math {
        inline Point3D vec_sub(const Point3D& a, const Point3D& b) {
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
            return {a.x/m, a.y/m, a.z/m, a.visibility};
        }
    }

    // ========================================================================
    // 3. MODEL DEFINITION & PARSERS
    // ========================================================================

    class Model {
    public:
        uint32_t version;
        bool is_physics;
        int calib_method;
        uint32_t zone_mask;
        float scale_factor;
        float overlap_tolerance;
        float dtw_sensitivity;
        float tempo_variance;
        std::vector<std::vector<float>> signature;
        std::vector<int> active_indices;

        static Model load_from_memory(const std::vector<uint8_t>& buffer) {
            if (buffer.size() < 24) throw std::runtime_error("Buffer too small for Willow header.");

            Model m;
            size_t offset = 0;

            auto read_u32 = [&](uint32_t& val) {
                val = static_cast<uint32_t>(buffer[offset]) |
                      (static_cast<uint32_t>(buffer[offset+1]) << 8) |
                      (static_cast<uint32_t>(buffer[offset+2]) << 16) |
                      (static_cast<uint32_t>(buffer[offset+3]) << 24);
                offset += 4;
            };

            auto read_f32 = [&](float& val) {
                uint32_t temp; read_u32(temp);
                std::memcpy(&val, &temp, sizeof(float)); 
            };

            read_u32(m.version);
            
            if (m.version == 40) {
                m.is_physics = false;
                m.calib_method = -1;
                read_u32(m.zone_mask);
                read_f32(m.scale_factor);
                read_f32(m.overlap_tolerance);
                read_f32(m.dtw_sensitivity);
                if (m.dtw_sensitivity <= 1e-5f) m.dtw_sensitivity = 1e-5f; 
                read_f32(m.tempo_variance);
            } else if (m.version == 41) {
                if (buffer.size() < 28) throw std::runtime_error("Buffer too small for Willow V4.1 header.");
                m.is_physics = true;
                read_u32(m.zone_mask);
                float calib_f; read_f32(calib_f);
                m.calib_method = static_cast<int>(calib_f);
                read_f32(m.scale_factor);
                read_f32(m.overlap_tolerance);
                read_f32(m.dtw_sensitivity);
                if (m.dtw_sensitivity <= 1e-5f) m.dtw_sensitivity = 1e-5f; 
                read_f32(m.tempo_variance);
            } else {
                throw std::runtime_error("Unsupported model version. Expected 40 (V4.0) or 41 (V4.1).");
            }

            for (const auto& pair : TOPOLOGY_ZONES) {
                if (m.zone_mask & pair.first) {
                    m.active_indices.insert(m.active_indices.end(), pair.second.begin(), pair.second.end());
                }
            }
            
            std::sort(m.active_indices.begin(), m.active_indices.end());
            m.active_indices.erase(std::unique(m.active_indices.begin(), m.active_indices.end()), m.active_indices.end());

            int num_features = (m.active_indices.size() * (m.active_indices.size() - 1)) / 2;
            if (num_features == 0) throw std::runtime_error("Zero valid features identified by zone mask.");
            
            int num_frames = (buffer.size() - offset) / num_features;

            if ((buffer.size() - offset) % num_features != 0) {
                throw std::runtime_error("Corrupted payload: Int8 array does not align with active topology.");
            }

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

        struct CurlDeleter { void operator()(CURL* curl) const { if (curl) curl_easy_cleanup(curl); } };
        struct CurlSlistDeleter { void operator()(curl_slist* slist) const { if (slist) curl_slist_free_all(slist); } };

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
        static Point3D to_unity(const Point3D& pt) { return { pt.x, -pt.y, pt.z, pt.visibility }; }
        static Point3D to_ros(const Point3D& pt) { return { pt.z, -pt.x, -pt.y, pt.visibility }; }
    };

    // ========================================================================
    // 6. DETECTOR (ORACLE-PARITY ENGINE)
    // ========================================================================

    class Detector {
    private:
        Model model;
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

        std::vector<DetectionResult> confirmed_detections;

        float calculate_scale(const Skeleton& skel) const {
            if (model.is_physics) return 1.0f; // PHYSICS MODE: Absolute meters

            float scale = 1.0f;
            if (model.zone_mask & TORSO) {
                float dx = skel[11].x - skel[23].x;
                float dy = skel[11].y - skel[23].y;
                float dz = skel[11].z - skel[23].z;
                scale = std::sqrt(dx*dx + dy*dy + dz*dz);
            } else if (model.zone_mask & LEGS) {
                float dx = skel[25].x - skel[27].x;
                float dy = skel[25].y - skel[27].y;
                float dz = skel[25].z - skel[27].z;
                scale = std::sqrt(dx*dx + dy*dy + dz*dz);
            } else if (model.zone_mask & ARMS) {
                float dx = skel[13].x - skel[15].x;
                float dy = skel[13].y - skel[15].y;
                float dz = skel[13].z - skel[15].z;
                scale = std::sqrt(dx*dx + dy*dy + dz*dz);
            } else if (model.zone_mask & HANDS) {
                float dx = skel[33].x - skel[42].x;
                float dy = skel[33].y - skel[42].y;
                float dz = skel[33].z - skel[42].z;
                scale = std::sqrt(dx*dx + dy*dy + dz*dz);
            } else if (model.zone_mask & FEET) {
                float dx = skel[29].x - skel[31].x;
                float dy = skel[29].y - skel[31].y;
                float dz = skel[29].z - skel[31].z;
                scale = std::sqrt(dx*dx + dy*dy + dz*dz);
            } else if (model.zone_mask & HEAD) {
                float dx = skel[1].x - skel[4].x;
                float dy = skel[1].y - skel[4].y;
                float dz = skel[1].z - skel[4].z;
                scale = std::sqrt(dx*dx + dy*dy + dz*dz);
            }
            return (scale < 0.01f) ? 0.01f : scale;
        }

        void extract_rdm(const Skeleton& skel, std::vector<float>& out_rdm) const {
            out_rdm.clear();
            float scale = calculate_scale(skel);
            const float CONF_THRESH_SQ = 0.25f; 

            for (size_t i = 0; i < model.active_indices.size(); ++i) {
                for (size_t j = i + 1; j < model.active_indices.size(); ++j) {
                    auto p1 = skel[model.active_indices[i]];
                    auto p2 = skel[model.active_indices[j]];
                    
                    float pair_conf = p1.visibility * p2.visibility;
                    
                    if (pair_conf < CONF_THRESH_SQ ||
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
            // CRITICAL MEMORY SAFEGUARD: Prevent Segfaults on Node 76
            if (model.zone_mask & OBJECT) {
                if (skel.size() < 76) {
                    throw std::invalid_argument("Physics Models require a 76-point skeleton (Node 76 = Tracked Object).");
                }
            } else {
                if (skel.size() != 75 && skel.size() != 76) {
                    throw std::invalid_argument("Skeleton must contain exactly 75 or 76 joints.");
                }
            }

            extract_rdm(skel, rdm_buffer); 
            int M = model.signature.size();

            std::fill(D_curr.begin(), D_curr.end(), std::numeric_limits<float>::infinity());
            std::fill(S_curr.begin(), S_curr.end(), 0);

            D_curr[0] = 0.0f;
            S_curr[0] = current_frame;

            for (int j = 1; j <= M; ++j) {
                float dist = 0.0f;
                int valid_dims = 0;
                int total_dims = rdm_buffer.size();
                
                for (int k = 0; k < total_dims; ++k) {
                    float val_test = rdm_buffer[k];
                    float val_seed = model.signature[j-1][k];
                    
                    if (val_test != 0.0f && val_seed != 0.0f) {
                        float diff = val_test - val_seed;
                        dist += diff * diff;
                        valid_dims++;
                    }
                }
                
                float cost = 999.0f;
                if (valid_dims > 0) {
                    dist = (dist / static_cast<float>(valid_dims)) * static_cast<float>(total_dims);
                    cost = std::sqrt(dist);
                }

                float prev_cost;
                if (D_prev[j-1] <= D_prev[j] && D_prev[j-1] <= D_curr[j-1]) {
                    prev_cost = D_prev[j-1]; 
                    S_curr[j] = S_prev[j-1];
                } else if (D_prev[j] <= D_curr[j-1]) {
                    prev_cost = D_prev[j]; 
                    S_curr[j] = S_prev[j];
                } else {
                    prev_cost = D_curr[j-1]; 
                    S_curr[j] = S_curr[j-1];
                }
                D_curr[j] = cost + prev_cost;
            }

            float final_cost = D_curr[M] / M;
            int length = current_frame - S_curr[M];

            sim_t0 = std::max(0.0f, std::min(1.0f, 1.0f - (final_cost / model.dtw_sensitivity)));
            bool detection_fired = false;

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
            
            Point3D right = math::vec_cross(up_ref, forward);
            if (math::vec_mag(right) < 1e-5f) {
                Point3D alt_up = {1.0f, 0.0f, 0.0f, 1.0f}; 
                if (std::abs(forward.x) > 0.99f) {
                    alt_up = {0.0f, 0.0f, 1.0f, 1.0f}; 
                }
                right = math::vec_cross(alt_up, forward);
            }
            
            right = math::vec_norm(right);
            Point3D up = math::vec_norm(math::vec_cross(forward, right));

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