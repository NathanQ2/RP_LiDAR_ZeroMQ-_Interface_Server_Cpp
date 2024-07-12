#pragma once

struct Packet {
    uint64_t timestamp;
    sl_lidar_response_measurement_node_hq_t nodes[291];
};