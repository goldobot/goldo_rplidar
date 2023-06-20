#include <unistd.h>

#include <cstddef>
#include "rplidar.h"
#include <zmq.h>
#include <cmath>
#include <memory>
#include <string>
#include <iostream>

#include "robot_detection_info.hpp"
#include "lidar_detect.hpp"

#include "goldo_gpio.hpp"

using namespace rp::standalone::rplidar;
#if 1 /* FIXME : DEBUG : GOLDO */
using namespace goldobot;
#endif

extern char* rp_shmem;


// front near, front far, right near, right far, back near, back far, left near, left far
#define FRONT_NEAR   0
#define LEFT_NEAR    1
#define BACK_NEAR    2
#define RIGHT_NEAR   3
#define FRONT_FAR    4
#define LEFT_FAR     5
#define BACK_FAR     6
#define RIGHT_FAR    7

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

struct PolPoint
{
    float rho;
    float theta;
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
    int pointZonePolar(float x, float y, float rho, float theta);
    bool checkNearAdversary();
    float getEffectiveDetectionLimit(float az);
    void trackAdversaries();
    
    void sendScan();
    
    void initZmq();

#if 1 /* FIXME : DEBUG : GOLDO */
    detected_robot_info_t m_autotest_obst;
    void initAutotest();
    void sendAutotest();
#endif

    void sendLidarTracks();
    
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
    PolPoint m_pol_points[c_nb_points];
    
    float m_cfg_dist_limits[3]{0.1f,0.3f,0.6f}; // too near (in robot), near, far

    bool m_enable_autotest{false};
    bool m_enable_send_scan{false};

    bool           m_strat_enable_flag{false};
    unsigned char  m_strat_curr_cmd{'u'};
    float          m_strat_speed_val{0.0};
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
            std::cout << "failed to get device info, error: " << res << "\n";
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

enum class MessageIdIn: uint8_t
{
    Unknown=0,
    StartMotor,
    StopMotor,
    SetThetaOffset,
    SetRobotPose,
    SetDistanceLimits,
    SetEnableAutotest,
    SetEnableSendScan
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
    MessageIdIn command;
    uint8_t val{0};
    auto bytes_read = zmq_recv(m_sub_socket, (uint8_t*)&command , 1, 0);
    switch(command)
    {
        case MessageIdIn::StartMotor:
            startMotor();
            zmq_recv(m_sub_socket, nullptr , 0, 0);
            break;
        case MessageIdIn::StopMotor:
            stopMotor();
            zmq_recv(m_sub_socket, nullptr , 0, 0);
            break;
        case MessageIdIn::SetThetaOffset:
            zmq_recv(m_sub_socket, &m_theta_offset , sizeof(m_theta_offset), 0);
            break;
        case MessageIdIn::SetRobotPose:
            zmq_recv(m_sub_socket, &m_pose_x , 12, 0);
            break;
        case MessageIdIn::SetDistanceLimits:
            zmq_recv(m_sub_socket, &m_cfg_dist_limits , 12, 0);
            std::cout << "MessageIdIn::SetDistanceLimits:\n";
            std::cout << "  m_cfg_dist_limits[0]="<<m_cfg_dist_limits[0]<<"\n";
            std::cout << "  m_cfg_dist_limits[1]="<<m_cfg_dist_limits[1]<<"\n";
            std::cout << "  m_cfg_dist_limits[2]="<<m_cfg_dist_limits[2]<<"\n";
            break;
        case MessageIdIn::SetEnableAutotest:
            zmq_recv(m_sub_socket, &val , 1, 0);
            m_enable_autotest = val> 0;
            std::cout << "set autotest enable: " << m_enable_autotest << "\n";
            break;
        case MessageIdIn::SetEnableSendScan:
            zmq_recv(m_sub_socket, &val , 1, 0);
            m_enable_send_scan = val> 0;
            std::cout << "set send scan enable: " << m_enable_send_scan << "\n";
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
      int j = 0;
      for (unsigned i = 0; i < count; i++) {
        double theta = m_nodes[i].angle_z_q14 * c_theta_factor + m_theta_offset;
        double rho = m_nodes[i].dist_mm_q2 * c_rho_factor;        
       
        if(rho >= 0.05)
        {
          m_points[j].x = rho * cosf(theta + m_pose_yaw) + m_pose_x;
          m_points[j].y = rho * sinf(theta + m_pose_yaw) + m_pose_y;
          m_pol_points[j].rho = rho;
          m_pol_points[j].theta = theta;
          j++;
        }
      }

      //std::cout << "\n";

      m_strat_enable_flag = (rp_shmem[0]!=0x00)?true:false;
      m_strat_curr_cmd    = rp_shmem[1];
      m_strat_speed_val   = *((float *)((unsigned char *)&rp_shmem[4]));
# if 0 /* FIXME : DEBUG */
      if (fabs(speed_val)>0.000001) {
        printf ("TEST : rp_shmem = %x\n", rp_shmem);
        printf ("       enable_flag = %x\n", m_strat_enable_flag);
        printf ("       curr_cmd    = %c (%x)\n", m_strat_curr_cmd, m_strat_curr_cmd);
        printf ("       speed_val   = %f\n", m_strat_speed_val);
      }
#endif

      checkNearAdversary();
      if(m_enable_send_scan)
      {
        sendScan();
      }
      if(m_enable_autotest)
      {
        sendAutotest();
      } else 
      {    
        trackAdversaries();
      }
    }
};

int RPLidar::pointZonePolar(float x, float y, float rho, float theta)
{
    float detect_dist = getEffectiveDetectionLimit(0.0);

    // normalize theta
    while (theta>M_PI) theta -= M_PI;
    while (theta<=(-M_PI)) theta += M_PI;

    // exclude points outside
    if(x < 0.1f || x > 2.9f || y < -0.9f || y > 0.9f)
    {
        return -1;
    };

    if((rho <= m_cfg_dist_limits[0]) || (rho > detect_dist))
    {
        return -1;
    };
    
    int quadrant=0;

    if((theta>=(-M_PI/4)) && (theta<=(M_PI/4))) quadrant = 0; // front
    if((theta<=(-3.0*M_PI/4)) || (theta>=(3.0*M_PI/4))) quadrant = 2; // back
    if((theta>(M_PI/4)) && (theta<(3.0*M_PI/4))) quadrant = 1; // left
    if((theta>(-3.0*M_PI/4)) && (theta<(-M_PI/4))) quadrant = 3; // right
    
    return quadrant;
};

int RPLidar::pointZone(float x, float y)
{
    float detect_dist = getEffectiveDetectionLimit(0.0);

    // exclude points outside
    if((x < 0.1f) || (x > 2.9f) || (y < -0.9f) || (y > 0.9f))
    {
        return -1;
    };
    float dx = x - m_pose_x;
    float dy = y - m_pose_y;
    
    float x_rel = dx * cos(m_pose_yaw) + dy * sin(m_pose_yaw);
    float y_rel = -dx * sin(m_pose_yaw) + dy * cos(m_pose_yaw);
    
    float d = sqrtf(x_rel * x_rel + y_rel * y_rel);
   
    if(d <= m_cfg_dist_limits[0] || d > detect_dist)
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
    
    return quadrant;
};

float RPLidar::getEffectiveDetectionLimit(float az)
{
    az = az; /* FIXME : TODO (azymuth dependent detection limit) */

    float detect_dist = m_cfg_dist_limits[1] + (m_cfg_dist_limits[2]-m_cfg_dist_limits[1])*fabs(m_strat_speed_val)/1.0;

    return detect_dist;
};

bool RPLidar::checkNearAdversary()
{
    int counts[8] = {0,0,0,0,0,0,0,0};
    uint8_t detect[8];

    for(int i=0; i < m_count; i++)
    {
        /* FIXME : TODO : remove old code */
        //auto z = pointZone(m_points[i].x, m_points[i].y);
        auto z = pointZonePolar(m_points[i].x, m_points[i].y, m_pol_points[i].rho, m_pol_points[i].theta);
        if(z >= 0)
        {
            counts[z]++;
        };
    };

    for(int i = 0; i < 8; i++)
    {
        detect[i] = counts[i] >= 6;
    };
        
#if 1 /* FIXME : TODO : improve usage of the GPIO (direct obstacle signaling to the Nucleo)        */
      /*                temporary hack to improve reaction time after the detection of an obstacle */
    {
      goldo_gpio_check_shmem();

      bool adversary_detected = false;

      if ((m_strat_speed_val > 0.05) && (detect[FRONT_NEAR]>0))
      {
        adversary_detected = true;
      }
      if ((m_strat_speed_val < -0.05) && (detect[BACK_NEAR]>0))
      {
        adversary_detected = true;
      }

      if (!m_strat_enable_flag)
      {
        adversary_detected = false;
      }

      if (adversary_detected)
      {
#if 0 /* FIXME : DEBUG */
        struct timespec my_tp;
        unsigned int my_time_ms;

        clock_gettime(1, &my_tp);
        my_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;

        std::cout << "RPLidar: adversary detected\n";
        std::cout << "  T="<<my_time_ms<<"\n";
        std::cout << "  pose=<"<<m_pose_x<<","<<m_pose_y<<">\n";
        std::cout << "  m_strat_speed_val="<<m_strat_speed_val<<"\n";
        std::cout << "  counts :   F  L  B  R\n";
        std::cout << "           "<<(int)counts[FRONT_NEAR]<< "  "<<(int)counts[LEFT_NEAR]<<"  "<<(int)counts[BACK_NEAR]<<"  "<<(int)counts[RIGHT_NEAR]<<"\n";
        std::cout << "  detect :   F  L  B  R\n";
        std::cout << "           "<<(int)detect[FRONT_NEAR]<< "  "<<(int)detect[LEFT_NEAR]<<"  "<<(int)detect[BACK_NEAR]<<"  "<<(int)detect[RIGHT_NEAR]<<"\n";
#endif
        goldo_gpio_set();
      }
      else
      {
        //std::cout << "RPLidar: no obstacle\n";
        goldo_gpio_clr();
      }
    }
#endif

    uint8_t type = 42;
    zmq_send(m_pub_socket, &type, 1, ZMQ_SNDMORE );
    zmq_send(m_pub_socket, &detect, 8, 0);
    return true;
}

void RPLidar::trackAdversaries()
{
  struct timespec my_tp;
  clock_gettime(1, &my_tp);
  int my_thread_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;
  /* reset des slots de detection du tracker d'adversaire */ 
  LidarDetect::instance().clearSlots();
  /* envoi des plots lidar au tracker d'adversaire (+filtrage geometrique) */ 
  for (unsigned i = 0; i < m_count; i++) {
    float x = m_points[i].x;
    float y = m_points[i].y;

    /* FIXME : TODO : limites du terrain en variables de conf.. */
    if ((x >  0.10) && (x <  2.95) && 
        (y > -0.95) && (y <  0.95)) { /* si a l'interieur du terrain */
      LidarDetect::instance().recordNewLidarSample(my_thread_time_ms, x*1000.0, y*1000.0);
    }
  }
  /* detection des clusters de plots representant potentiellement un adversaire */ 
  LidarDetect::instance().updateDetection();
  /* envoi des tracks lidar a goldo_main */ 
  sendLidarTracks();
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

#if 1 /* FIXME : DEBUG : GOLDO */
void RPLidar::initAutotest()
{
  struct timespec my_tp;
  clock_gettime(1, &my_tp);
  int my_thread_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;

  m_autotest_obst.timestamp_ms = my_thread_time_ms;
  m_autotest_obst.id = 0;
  m_autotest_obst.x_mm = 100.0;
  m_autotest_obst.y_mm = 0.0;
  m_autotest_obst.vx_mm_sec = 40.0;
  m_autotest_obst.vy_mm_sec = 0.0;
  m_autotest_obst.ax_mm_sec_2 = 0.0;
  m_autotest_obst.ay_mm_sec_2 = 0.0;
  m_autotest_obst.detect_quality = 20;
};

void RPLidar::sendAutotest()
{
  struct timespec my_tp;
  clock_gettime(1, &my_tp);
  int my_thread_time_ms = my_tp.tv_sec*1000 + my_tp.tv_nsec/1000000;
  int delta_t_ms = my_thread_time_ms - m_autotest_obst.timestamp_ms;
  double delta_t_s = 0.001*delta_t_ms;

  m_autotest_obst.timestamp_ms = my_thread_time_ms;
  m_autotest_obst.x_mm = m_autotest_obst.x_mm + delta_t_s*m_autotest_obst.vx_mm_sec;

  if (m_autotest_obst.x_mm>1200.0)
  {
    m_autotest_obst.vx_mm_sec = 0.0;
  }

  robot_detection_msg_t my_message;
  my_message.timestamp_ms   = m_autotest_obst.timestamp_ms;
  my_message.id             = m_autotest_obst.id;
  my_message.x_mm_X4        = m_autotest_obst.x_mm * 4.0;
  my_message.y_mm_X4        = m_autotest_obst.y_mm * 4.0;
  my_message.vx_mm_sec      = m_autotest_obst.vx_mm_sec;
  my_message.vy_mm_sec      = m_autotest_obst.vy_mm_sec;
  my_message.ax_mm_sec_2    = m_autotest_obst.ax_mm_sec_2;
  my_message.ay_mm_sec_2    = m_autotest_obst.ay_mm_sec_2;
  my_message.detect_quality = m_autotest_obst.detect_quality;

  uint8_t type = 2;
  zmq_send(m_pub_socket, &type, 1, ZMQ_SNDMORE );
  zmq_send(m_pub_socket, &my_message, sizeof(my_message), 0);

  usleep(10000);
};
#endif

void RPLidar::sendLidarTracks()
{
  robot_detection_msg_t my_message;

  for (int i=0; i<3; i++)
  {
    detected_robot_info_t& detect = 
      LidarDetect::instance().detected_robot(i);
    if (detect.detect_quality>1)
    {
      my_message.timestamp_ms   = detect.timestamp_ms;
      my_message.id             = detect.id;
      my_message.x_mm_X4        = detect.x_mm * 4.0;
      my_message.y_mm_X4        = detect.y_mm * 4.0;
      my_message.vx_mm_sec      = detect.vx_mm_sec;
      my_message.vy_mm_sec      = detect.vy_mm_sec;
      my_message.ax_mm_sec_2    = detect.ax_mm_sec_2;
      my_message.ay_mm_sec_2    = detect.ay_mm_sec_2;
      my_message.detect_quality = detect.detect_quality;

      uint8_t type = 2;
      zmq_send(m_pub_socket, &type, 1, ZMQ_SNDMORE );
      zmq_send(m_pub_socket, &my_message, sizeof(my_message), 0);
    }
  }
};


RPLidar g_lidar;

int main(int argc, char** argv) {
  if(!g_lidar.connectLidar("/dev/goldorak/ttyLidar"))
  {
      return -1;
  }
  g_lidar.initZmq();
#if 1 /* FIXME : DEBUG : GOLDO */
  g_lidar.initAutotest();
#endif

  goldo_gpio_init();

  g_lidar.run();
  return 0;
}
