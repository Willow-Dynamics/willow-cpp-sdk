/**
 * Compile with: g++ -std=c++14 03_cloud_provisioning.cpp -o ex03 -lcurl -DWILLOW_ENABLE_CURL
 */
 #include "../include/willow.hpp"
 #include <iostream>
 #include <cstdlib> 
 
 using namespace willow;
 
 int main() {
     std::cout << "Starting Example 3: Secure Cloud Provisioning\n";
     
     #ifdef WILLOW_ENABLE_CURL
     try {
         // SECURE DEFAULT: Read credentials from environment variables instead of hardcoding.
         // This enforces a zero-trust policy against API keys committed to source control.
         const char* env_api_key = std::getenv("WILLOW_API_KEY");
         const char* env_cust_id = std::getenv("WILLOW_CUSTOMER_ID");
 
         if (env_api_key == nullptr || env_cust_id == nullptr) {
             throw std::runtime_error(
                 "Missing Credentials. Please export WILLOW_API_KEY and WILLOW_CUSTOMER_ID "
                 "in your environment before running this application."
             );
         }
         
         std::string api_key(env_api_key);
         std::string customer_id(env_cust_id);
 
         Client client(api_key, customer_id);
         std::cout << "Authenticating and fetching model securely into RAM...\n";
         
         // This performs an HTTPS GET request to the Willow Cloud Oracle.
         // The resulting vector<uint8_t> contains the encrypted .int8 model.
         // It NEVER touches the physical disk, adhering strictly to the RAM-Only DRM policy.
         std::vector<uint8_t> secure_buffer = client.fetch_model("tactical_reload_v1");
         
         // The SDK parses the header, de-quantizes the signature, and initializes the model
         Model model = Model::load_from_memory(secure_buffer);
         
         std::cout << "========================================\n";
         std::cout << "SUCCESS: Model securely provisioned.\n";
         std::cout << "Version:          " << model.version << "\n";
         std::cout << "Target Zone Mask: " << model.zone_mask << "\n";
         std::cout << "Active Features:  " << (model.active_indices.size() * (model.active_indices.size() - 1)) / 2 << " per frame\n";
         std::cout << "========================================\n";
 
     } catch(const std::exception& e) {
         std::cerr << "\n[FATAL ERROR] Provisioning Failed: " << e.what() << "\n";
         return 1;
     }
     #else
     std::cout << "libcurl not enabled. Cannot fetch model. Please compile with -DWILLOW_ENABLE_CURL.\n";
     #endif
 
     return 0;
 }