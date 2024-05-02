void test_checkSockets_robot_telemetry()
{
  std::cout << "MessageIdIn::RobotTelemetry:\n";
  std::cout << "  m_pose_x            ="<<m_pose_x<<"\n";
  std::cout << "  m_pose_y            ="<<m_pose_y<<"\n";
  std::cout << "  m_pose_yaw          ="<<m_pose_yaw<<"\n";
  std::cout << "  m_strat_speed_val   ="<<m_strat_speed_val<<"\n";
  std::cout << "  pose_yaw_rate       ="<<m_last_telemetry.pose_yaw_rate<<"\n";
  std::cout << "  pose_acc            ="<<m_last_telemetry.pose_acc<<"\n";
  std::cout << "  pose_angular_acc    ="<<m_last_telemetry.pose_angular_acc<<"\n";
  std::cout << "  left_encoder        ="<<m_last_telemetry.left_encoder<<"\n";
  std::cout << "  right_encoder       ="<<m_last_telemetry.right_encoder<<"\n";
  std::cout << "  left_pwm            ="<<m_last_telemetry.left_pwm<<"\n";
  std::cout << "  right_pwm           ="<<m_last_telemetry.right_pwm<<"\n";
  std::cout << "  state               ="<<m_last_telemetry.state<<"\n";
  std::cout << "  error               ="<<m_last_telemetry.error<<"\n";

  {
    uint8_t test_detect[8] = {1,0,0,1,0,0,0,0};
    uint8_t test_detect_type = 42;

    uint8_t mask = 0x01;

    for (int i=0; i<8; i++)
    {
      //if ((m_last_telemetry.state&mask)!=0x00) test_detect[i] = 1;
      mask = mask<<1;
    }
    zmq_send(m_pub_socket, &test_detect_type, 1, ZMQ_SNDMORE );
    zmq_send(m_pub_socket, &test_detect, 8, 0);
  }

  initAutotest();
  m_autotest_obst.x_mm = m_last_telemetry.pose_x*1000.0;
  m_autotest_obst.y_mm = m_last_telemetry.pose_y*1000.0;
  m_autotest_obst.vx_mm_sec = m_strat_speed_val*1000.0/2;
  m_autotest_obst.vy_mm_sec = m_strat_speed_val*1000.0/3;
  sendAutotest();
}

void test_checkLidar()
{
  if (fabs(speed_val)>0.000001) {
    printf ("TEST : rp_shmem = %x\n", rp_shmem);
    printf ("       enable_flag = %x\n", m_strat_enable_flag);
    printf ("       curr_cmd    = %c (%x)\n", m_strat_curr_cmd, m_strat_curr_cmd);
    printf ("       speed_val   = %f\n", m_strat_speed_val);
  }
}

void test_checkNearAdversary()
{
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
}
