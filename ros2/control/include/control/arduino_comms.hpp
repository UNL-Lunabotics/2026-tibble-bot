#ifndef TIBBLE_ARDUINO_COMMS_HPP
#define TIBBLE_ARDUINO_COMMS_HPP

#include <string>
#include <cstdint>
#include <iostream>
#include <vector>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

namespace tibble_hwc
{
    class ArduinoComms
    {
    public:
        ArduinoComms() = default;
        
        ~ArduinoComms() {
            disconnect();
        }

        inline void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms) {
            serial_device_ = serial_device;
            baud_rate_ = baud_rate;
            timeout_ms_ = timeout_ms;
        }
        
        inline void connect() {
            // Open the serial port in Read/Write, No Controlling Terminal, and Non-Blocking mode
            serial_fd_ = open(serial_device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            
            if (serial_fd_ < 0) {
                std::cerr << "[ArduinoComms] Error " << errno << " opening " << serial_device_ << ": " << strerror(errno) << std::endl;
                return;
            }

            struct termios tty;
            if (tcgetattr(serial_fd_, &tty) != 0) {
                std::cerr << "[ArduinoComms] Error from tcgetattr: " << strerror(errno) << std::endl;
                return;
            }

            // Set Baud Rate (Assuming 115200 for your RoboClaw passthrough/Teensy setup)
            cfsetospeed(&tty, B9600);
            cfsetispeed(&tty, B9600);

            // 8N1 standard serial setup (8 bits, no parity, 1 stop bit)
            tty.c_cflag &= ~PARENB; // Clear parity bit
            tty.c_cflag &= ~CSTOPB; // Clear stop field (one stop bit)
            tty.c_cflag &= ~CSIZE;  // Clear all bits that set the data size 
            tty.c_cflag |= CS8;     // 8 bits per byte
            tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
            tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

            // Raw mode (disable canonical mode, echo, etc.)
            tty.c_lflag &= ~ICANON;
            tty.c_lflag &= ~ECHO; 
            tty.c_lflag &= ~ECHOE;
            tty.c_lflag &= ~ECHONL;
            tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
            tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
            tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
            tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
            tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

            // Non-blocking read setup
            tty.c_cc[VTIME] = 0; // Wait for up to 0 deciseconds
            tty.c_cc[VMIN] = 0;  // Return immediately with whatever is available

            if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
                std::cerr << "[ArduinoComms] Error from tcsetattr: " << strerror(errno) << std::endl;
                return;
            }

            // Flush the OS buffer just to be safe
            tcflush(serial_fd_, TCIOFLUSH);
            is_connected_ = true;
            receive_buffer_.clear();
            
            std::cout << "[ArduinoComms] Successfully connected to " << serial_device_ << std::endl;
        }

        inline void disconnect() {
            if (is_connected_ && serial_fd_ >= 0) {
                close(serial_fd_);
                is_connected_ = false;
                std::cout << "[ArduinoComms] Disconnected from " << serial_device_ << std::endl;
            }
        }

        inline bool is_connected() const {
            return is_connected_;
        }

        inline void send_commands(int la_1_pwm, int la_2_pwm, int vibe_pwm, int excav_pwm, int latch_angle) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "c %d %d %d %d %d\n", la_1_pwm, la_2_pwm, vibe_pwm, excav_pwm, latch_angle);
            send_string(std::string(buffer));
        }

        inline void send_stop_command() {
            send_string("s\n");
        }

        inline void send_reset_command() {
            send_string("r\n");
        }

        inline void read_encoder_values(int32_t &la1_enc, int32_t &la2_enc, int32_t &excav_enc) {
            if (!is_connected_) return;

            char read_buf[256];
            int n = read(serial_fd_, read_buf, sizeof(read_buf));

            if (n > 0) {
                receive_buffer_.append(read_buf, n);

                size_t pos;
                std::string last_valid_line = "";

                // Drain the buffer, searching for '\n'
                while ((pos = receive_buffer_.find('\n')) != std::string::npos) {
                    last_valid_line = receive_buffer_.substr(0, pos);
                    receive_buffer_.erase(0, pos + 1); // Remove the parsed line from the buffer
                }

                // If found at least one complete line, parse it
                if (!last_valid_line.empty() && last_valid_line[0] == 'e') {
                    int temp_la1 = 0, temp_la2 = 0, temp_excav = 0;
                    
                    if (sscanf(last_valid_line.c_str(), "e %d %d %d", &temp_la1, &temp_la2, &temp_excav) == 3) {
                        // Successfully parsed 3 values, update the references passed in!
                        la1_enc = static_cast<int32_t>(temp_la1);
                        la2_enc = static_cast<int32_t>(temp_la2);
                        excav_enc = static_cast<int32_t>(temp_excav);
                    }
                }
            }
        }

    private:
        std::string serial_device_;
        int32_t baud_rate_;
        int32_t timeout_ms_;
        
        int serial_fd_{-1}; 
        bool is_connected_{false};
        std::string receive_buffer_;

        inline void send_string(const std::string &msg) {
            if (!is_connected_) return;
            // Write the raw bytes to the file descriptor
            write(serial_fd_, msg.c_str(), msg.length());
        }
    };

} // namespace tibble_hwc

#endif // TIBBLE_ARDUINO_COMMS_HPP