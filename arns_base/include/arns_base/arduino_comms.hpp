#ifndef ARNS_HARDWARE_COMMS_HPP
#define ARNS_HARDWARE_COMMS_HPP

// #include <cstring>
// #include <cstdlib>
#include <libserial/SerialPort.h>


LibSerial::BaudRate convert_baud_rate(int baud_rate);

class ArduinoComms
{

public:

  ArduinoComms();

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

  void disconnect();

  bool connected() const;


  std::string send_msg(const std::string &msg_to_send, bool print_output = false);


  void send_empty_msg();

  void read_encoder_values(int &val_1, int &val_2);
  void set_motor_values(int val_1, int val_2);

  void set_pid_values(int k_p, int k_d, int k_i, int k_o);

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif