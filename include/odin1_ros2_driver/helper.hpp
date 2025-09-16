#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <builtin_interfaces/msg/time.hpp>

// usb device
static std::string TARGET_VENDOR = "2207";
static std::string TARGET_PRODUCT = "0019";

// Improved USB version detection using sysfs
bool isUsb3OrHigher(const std::string& vendor_id, const std::string& product_id)
{
    try {
        // USB devices directory
        std::filesystem::path usb_devices_dir("/sys/bus/usb/devices");
        
        if (!std::filesystem::exists(usb_devices_dir)) {
            std::cerr << "USB devices directory not found" << std::endl;
            return false;
        }
        
        // Iterate through USB device directories
        bool device_found = false;
        for (const auto& device_entry : std::filesystem::directory_iterator(usb_devices_dir)) {
            if (!device_entry.is_directory()) continue;
            
            std::filesystem::path device_path = device_entry.path();
            
            // Check if vendor and product files exist
            std::filesystem::path vendor_file = device_path / "idVendor";
            std::filesystem::path product_file = device_path / "idProduct";
            std::filesystem::path speed_file = device_path / "speed";
            
            if (!std::filesystem::exists(vendor_file) || 
                !std::filesystem::exists(product_file) || 
                !std::filesystem::exists(speed_file)) {
                continue;
            }
            
            // Read vendor ID
            std::ifstream vendor_stream(vendor_file);
            std::string device_vendor;
            if (!std::getline(vendor_stream, device_vendor)) continue;
            
            // Read product ID
            std::ifstream product_stream(product_file);
            std::string device_product;
            if (!std::getline(product_stream, device_product)) continue;
            
            // Check if this is our target device
            if (device_vendor == vendor_id && device_product == product_id) {
                // Read speed
                std::ifstream speed_stream(speed_file);
                std::string speed_str;
                if (!std::getline(speed_stream, speed_str)) continue;
                
                try {
                    int speed = std::stoi(speed_str);
                    // USB 3.0 and higher have speeds >= 5000 Mbps
                    // USB 2.0 is typically 480 Mbps or 12 Mbps (full speed) or 1.5 Mbps (low speed)
                    if (speed >= 5000) 
                        device_found = true;
                } catch (const std::exception& e) {
                    std::cerr << "Error parsing speed: " << e.what() << std::endl;
                    continue;
                }
            }
        }
        
        return device_found;
        
    } catch (const std::exception& e) {
        std::cerr << "Error in USB version detection: " << e.what() << std::endl;
        return false;
    }
}

inline rclcpp::Time ns_to_ros_time(uint64_t ns) {
    const uint64_t NANOSECONDS_PER_SECOND = 1000000000ULL;
    return rclcpp::Time(ns / NANOSECONDS_PER_SECOND, ns % NANOSECONDS_PER_SECOND);
}

inline uint64_t ros_time_to_ns(const rclcpp::Time &t) {
    return static_cast<uint64_t>(t.seconds() * 1e9);
}

#define GD_ACCL_G 9.7833f
#define ACC_1G_ms2 9.8
#define ACC_SEN_SCALE 4096
#define PAI 3.14159265358979323846
#define GYRO_SEN_SCALE 16.4f
#define DTOF_NUM_ROW_PER_GROUP 6
// Common functions
inline float accel_convert(int16_t raw, int sen_scale) {
    return (raw * GD_ACCL_G / sen_scale);
}
inline float gyro_convert(int16_t raw, float sen_scale) {
    return (raw * PAI) / (sen_scale * 180); 
}