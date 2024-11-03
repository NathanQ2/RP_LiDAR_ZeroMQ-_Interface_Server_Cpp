#include <iostream>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>

#include <sl_lidar.h>
#include <sl_lidar_driver.h>
#include <sl_types.h>

#include "Packet.h"

using boost::asio::ip::tcp;

int main(int argc, const char** argv)
{
    const uint32_t timeout = 5000;

    if (argc != 4) {
        std::cerr << "ERR: Invalid number of arguments, got " << argc << " but expected 3" << std::endl;
        return -1;
    }

    const char* ip = argv[2];
    const char* port = argv[3];

    boost::asio::io_context ioContext;
    tcp::resolver resolver(ioContext);
    auto endpoints = resolver.resolve(ip, port);

    tcp::socket socket(ioContext);
    auto endpoint = boost::asio::connect(socket, endpoints);

    const char* device = argv[1];
    std::cout << "Opening channel on device: " << device << std::endl;
    sl::Result<sl::IChannel*> channelResult = sl::createSerialPortChannel(device, 115200);
    sl::Result<sl::ILidarDriver*> driverResult = sl::createLidarDriver();
    if (!channelResult) {
        std::cerr << "Error opening channel!" << std::endl;

        return -1;
    }
    if (!driverResult) {
        std::cerr << "Error creating driver!" << std::endl;

        return -1;
    }

    sl::IChannel* channel = channelResult.value;
    sl::ILidarDriver* driver = driverResult.value;

    sl_result res = driver->connect(channel);

    if (!SL_IS_OK(res)) {
        std::cerr << "Error connecting to device!" << std::endl;
        std::cerr << res << std::endl;

        return -1;
    }

    sl_lidar_response_device_info_t deviceInfo;
    res = driver->getDeviceInfo(deviceInfo, timeout);
    if(!SL_IS_OK(res)) {
        std::cerr << "Error retrieving device info!" << std::endl;
        std::cerr << res << std::endl;

        // -- cleanup --
        channel->close();
        driver->disconnect();

        delete channel;
        delete driver;

        return -1;
    }

    std::cout << "Device Model: " << deviceInfo.model << "\nDevice Hardware Version: " << deviceInfo.hardware_version << "\nDevice Firmware Version: " << deviceInfo.firmware_version << std::endl;

    std::vector<sl::LidarScanMode> scanModes;
    res = driver->getAllSupportedScanModes(scanModes, timeout);
    std::cout << "Available Scan Modes:" << std::endl;
    for(sl::LidarScanMode scanMode : scanModes) {
        std::cout << "Scan Mode: " << scanMode.scan_mode << " Scan ID: " << scanMode.id << " Max Distance: " << scanMode.max_distance << std::endl;
    }

    if (scanModes.empty() || !SL_IS_OK(res)) {
        std::cerr << "No scan modes detected!" << std::endl;

        return -1;
    }

    // driver->startScanExpress(false, scanModes[scanModes.size() - 1].id);
    driver->startScanExpress(false, scanModes[0].id);

    sl_lidar_response_measurement_node_hq_t nodes[291];
    size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);
    sl_u64 timestamp;

    std::cout << "Scanning" << std::endl;

    while (true) {
        res = driver->grabScanDataHqWithTimeStamp(nodes, nodeCount, timestamp);
        if (!SL_IS_OK(res)) {
            std::cerr << "Failed to grab scan data." << std::endl;
            std::cerr << res << std::endl;
        }

        for (int i = 0; i < nodeCount; i++) {
            float angle_in_degrees =  nodes[i].angle_z_q14 * 90.f / (1 << 14);
            float distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
//            std::cout << "angle: " << angle_in_degrees << "deg dist: " << distance_in_meters << std::endl;
        }

        Packet packet{};
        packet.timestamp = timestamp;
//        std::cout << "Timestmap: " << timestamp << std::endl;
//        std::cout << "angle 1: " << (uint16_t)nodes[0].angle_z_q14 << std::endl;
//        std::cout << "dist 1: " << (uint32_t)nodes[0].dist_mm_q2 << std::endl;


        std::copy(std::begin(nodes), std::end(nodes), std::begin(packet.nodes));

        // Write to socket
        try {
            boost::asio::write(socket, boost::asio::buffer(&packet.timestamp, sizeof(packet.timestamp)));
            boost::asio::write(socket, boost::asio::buffer(packet.nodes, sizeof(packet.nodes)));
        }
        catch (const boost::system::system_error& e) {
            std::cerr << "Error writing to socket: " << e.what() << "\nError Code: " << e.code() << std::endl;

            return -1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    std::cout << "Press enter to close." << std::endl;
    std::cin.get();


    // -- cleanup --
    channel->close();
    driver->disconnect();

    delete channel;
    delete driver;

    return 0;
}
