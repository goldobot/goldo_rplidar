#pragma once
#include <cstdint>
#include <cstddef>

#include "robot_detection_info.hpp"


namespace goldobot
{

  typedef struct _beacon_zone {
    double x_min_mm;
    double y_min_mm;
    double x_max_mm;
    double y_max_mm;
  } beacon_zone_t;

  typedef struct _lidar_sample {
    unsigned int timestamp_ms;
    double x_mm;
    double y_mm;
  } lidar_sample_t;

  typedef struct _current_detection_slot {
    int nb_rplidar_samples;
    unsigned int timestamp_ms;
    double x_mm;
    double y_mm;
  } current_detection_slot_t;


  class LidarDetect
  {
  public:
    static LidarDetect& instance();

    LidarDetect();

    int init();

    void clearSlots();

    bool sampleInBeaconZone(double x_mm, double y_mm);

    void recordNewLidarSample(unsigned int ts_ms, double x_mm, double y_mm);

    void updateDetection();

    void set_nb_of_send_detect(int _v) {if (_v<=MAX_NB_OF_DETECTED_ROBOTS) m_nb_of_send_detect=_v;};

    void set_quality_threshold(unsigned int _v) {m_quality_threshold=_v;};

    detected_robot_info_t& detected_robot(int _obst_idx);

  private:
    static constexpr int MAX_NB_OF_DETECTION_SLOTS = 400;
    static constexpr int MAX_NB_OF_DETECTED_ROBOTS = 3;
    static constexpr double OBSTACLE_SIZE_MM = 140.0;
    static constexpr int MAX_NB_OF_CACHED_SAMPLES = 10000;

    unsigned int m_quality_threshold = 1;

    int m_nb_of_send_detect = MAX_NB_OF_DETECTED_ROBOTS;

    unsigned int m_cur_ts_ms = 0;

    lidar_sample_t m_sample_cache[MAX_NB_OF_CACHED_SAMPLES];

    int m_last_free_cache;

    current_detection_slot_t m_detect_slot[MAX_NB_OF_DETECTION_SLOTS];

    detected_robot_info_t m_detect_candidate[MAX_NB_OF_DETECTED_ROBOTS];

    detected_robot_info_t m_detect_t_0[MAX_NB_OF_DETECTED_ROBOTS]; /* detectes pendant le scan courant(t0) */
    detected_robot_info_t m_detect_t_1[MAX_NB_OF_DETECTED_ROBOTS]; /* detectes a t0-1*dt */
    detected_robot_info_t m_detect_t_2[MAX_NB_OF_DETECTED_ROBOTS]; /* detectes a t0-2*dt */
    bool m_detect_lock; /* FIXME : TODO : improve synchronisation */

    detected_robot_info_t m_detect_export[MAX_NB_OF_DETECTED_ROBOTS]; /* copie de m_detect_t_* pour visibilite depuis classes externes */

    int m_beacon_zone_cnt;
    beacon_zone_t m_beacon_zone[10];

    unsigned int m_rplidar_plot_lifetime_ms = 300;

    static double dist(double x0, double y0, double x1, double y1);

    static double dist(detected_robot_info_t &R0, detected_robot_info_t &R1);

    static LidarDetect s_instance;
  };

}
