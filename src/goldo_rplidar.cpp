#include <cstddef>
#include "rplidar.h"
#include <zmq.h>
#include <cmath>
#include <memory>
#include <string>
#include <iostream>

using namespace rp::standalone::rplidar;

RPlidarDriver* g_driver = nullptr;

/*
bool startMotorCallback(goldo_rplidar::StartMotor::Request& request,
                        goldo_rplidar::StartMotor::Response& response) {
  g_driver->startMotor();
  g_driver->startScan(0, 1);
  return true;
};

bool stopMotorCallback(goldo_rplidar::StopMotor::Request& request,
                       goldo_rplidar::StopMotor::Response& response) {
  g_driver->stop();
  g_driver->stopMotor();
  return true;
};*/

struct Point
{
    float x;
    float y;
};

class RPLidar
{
public:
    RPLidar();
    
    bool connectLidar(const std::string& port_name);
    
    void run();
    
    
    float rhoCorrection(float rho);
    
    void startMotor();
    void stopMotor();
    
    
    void checkSockets();
    void checkLidar();
    int pointZone(float x, float y);
    bool checkAdversary();
    
    void sendScan();
    
    void initZmq();
    
    static constexpr float c_theta_factor {-3.141592653589793f * 0.5f / (1 << 14)};
    static constexpr float c_rho_factor{(1e-3f / 4.0f)};
    static constexpr int c_nb_points = 720;
    
    void* m_zmq_context;
    void* m_pub_socket;
    void* m_sub_socket;
    
    std::unique_ptr<RPlidarDriver> m_rplidar_driver;
    rplidar_response_measurement_node_hq_t m_nodes[16384];
    
    float m_theta_offset{0};
    
    float m_pose_x{0};
    float m_pose_y{0};
    float m_pose_yaw{0};
    
    bool m_detect_zones[8]; // front near, front far, right near, right far, back near, back far, left near, left far
    
    size_t m_count{0};
    Point m_points[c_nb_points];
    
    float m_cfg_dist_limits[3]{0.1f,0.3f,1.0f}; // too far ( in robot), near, far
};

RPLidar::RPLidar() :
  m_rplidar_driver(RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT))
{
    
    
}

void RPLidar::initZmq()
{
  m_zmq_context = zmq_init(1);
  
  m_pub_socket = zmq_socket(m_zmq_context, ZMQ_PUB);  
  m_sub_socket = zmq_socket(m_zmq_context, ZMQ_SUB);

  zmq_bind(m_pub_socket, "tcp://*:3102");
  zmq_bind(m_sub_socket, "tcp://*:3101");   

  zmq_setsockopt(m_sub_socket,ZMQ_SUBSCRIBE, "", 0); 
};

bool RPLidar::connectLidar(const std::string& port_name)
{    
    u_result  res = m_rplidar_driver->connect(port_name.c_str(), 115200);
    if(!IS_OK(res))
    {
        std::cout << "failed to connect to rplidar\n";
        return false;
    } else
    {
        std::cout << "connected to rplidar\n";
        rplidar_response_device_info_t device_info;
        res = m_rplidar_driver->getDeviceInfo(device_info);
        if(IS_OK(res))
        {            
            std::cout << "model: " << (int)device_info.model << " serial: " << device_info.serialnum << "\n";
            auto firmware_version_major = device_info.firmware_version>>8;
            auto firmware_version_minor = device_info.firmware_version & 0xFF;
            std::cout << "hardware version: " << (int)device_info.hardware_version << "\n";
            std::cout << "firmware version: " << (int)firmware_version_major << "." << (int)firmware_version_minor << "\n";
        } else
        {
            std::cout << "failed to get device info\n";
            return false;
        }        
    };
    return true;
};

void RPLidar::run()
{
    while(true)
    {
        checkSockets();
        checkLidar();
    };
}

void RPLidar::startMotor()
{
  std::cout << "start\n";
  m_rplidar_driver->startMotor();
  m_rplidar_driver->startScan(0, 1);
}

void RPLidar::stopMotor() 
{
  m_rplidar_driver->stop();
  m_rplidar_driver->stopMotor();
};

void RPLidar::checkSockets()
{  
  uint32_t events;
  int64_t more;
  size_t option_len;
  
  option_len = sizeof(events);
  zmq_getsockopt(m_sub_socket, ZMQ_EVENTS, &events, &option_len);

  while(events & ZMQ_POLLIN)
  {   
    uint8_t command;
    auto bytes_read = zmq_recv(m_sub_socket, &command , 1, 0);
    switch(command)
    {
        case 1:
            startMotor();
            zmq_recv(m_sub_socket, nullptr , 0, 0);
            break;
        case 2:
            stopMotor();
            zmq_recv(m_sub_socket, nullptr , 0, 0);
            break;
        case 3:
            zmq_recv(m_sub_socket, &m_theta_offset , sizeof(m_theta_offset), 0);
            break;
        case 4:
            zmq_recv(m_sub_socket, &m_pose_x , 12, 0);
            break;
        case 5:
            zmq_recv(m_sub_socket, &m_cfg_dist_limits , 12, 0);
            break;
        default:
            zmq_recv(m_sub_socket, nullptr , 0, 0);
    };
    option_len = sizeof(events);
    zmq_getsockopt(m_sub_socket, ZMQ_EVENTS, &events, &option_len);    
  };        
};

void RPLidar::checkLidar()
{
    auto count = sizeof(m_nodes);
    auto op_result = m_rplidar_driver->grabScanDataHq(m_nodes, count, 500);
    if (IS_OK(op_result)) {
      m_rplidar_driver->ascendScanData(m_nodes, count);
      m_count = count;
      for (unsigned i = 0; i < count; i++) {
        double theta = m_nodes[i].angle_z_q14 * c_theta_factor + m_theta_offset;
        double rho = m_nodes[i].dist_mm_q2 * c_rho_factor;        
       
        m_points[i].x = rho * cosf(theta + m_pose_yaw) + m_pose_x;
        m_points[i].y = rho * sinf(theta + m_pose_yaw) + m_pose_y;        
      }
      checkAdversary();
      sendScan();
    };    
};

int RPLidar::pointZone(float x, float y)
{
    // exclude points outside
    if(x < 0.1f || x > 1.9f || y < -1.4f || y > 1.4f)
    {
        return -1;
    };
    float dx = x - m_pose_x;
    float dy = y - m_pose_y;
    
    float x_rel = dx * cos(m_pose_yaw) + dy * sin(m_pose_yaw);
    float y_rel = -dx * sin(m_pose_yaw) + dy * cos(m_pose_yaw);
    
    float d = sqrtf(x_rel * x_rel + y_rel * y_rel);
   
    if(d <= m_cfg_dist_limits[0] || d > m_cfg_dist_limits[2])
    {
        return -1;
    };
    
    int quadrant=0;
    bool c1 = x_rel >= y_rel;
    bool c2 = x_rel >= -y_rel;
    
    if(c1 && c2) quadrant = 0; // front
    if(!c1 && !c2) quadrant = 2; // back
    if(c1 && !c2) quadrant = 1; // left
    if(!c1 && c2) quadrant = 3; // right
    
    if(d <= m_cfg_dist_limits[1])
    {
        return quadrant;
    } else
    {
        return quadrant + 4;
    };    
};

bool RPLidar::checkAdversary()
{
    int counts[8] = {0,0,0,0,0,0,0,0};
    uint8_t detect[8];
    for(int i=0; i < m_count; i++)
    {
        auto z = pointZone(m_points[i].x, m_points[i].y);
        if(z >= 0)
        {
            counts[z]++;
        };
    };
    
    for(int i = 0; i < 8; i++)
    {
        detect[i] = counts[i] >= 5;
    };
        
    uint8_t type = 42;
    zmq_send(m_pub_socket, &type, 1, ZMQ_SNDMORE );
    zmq_send(m_pub_socket, &detect, 8, 0);
    return true;
}

void RPLidar::sendScan()
{
  uint32_t events;
  size_t option_len;
  
  option_len = sizeof(events);
  zmq_getsockopt(m_pub_socket, ZMQ_EVENTS, &events, &option_len);
  
  if(!(events & ZMQ_POLLOUT))
  {
      return;
  };
  
  uint8_t type = 1;
  zmq_send(m_pub_socket, &type, 1, ZMQ_SNDMORE );
  zmq_send(m_pub_socket, &m_pose_x, 12, ZMQ_SNDMORE );
  zmq_send(m_pub_socket, m_points, 8 * m_count, 0);
    
};

RPLidar g_lidar;

int main(int argc, char** argv) {
  if(!g_lidar.connectLidar("/dev/goldorak/ttyLidar"))
  {
      return -1;
  }
  g_lidar.initZmq();
  g_lidar.run();
  return 0;
}
