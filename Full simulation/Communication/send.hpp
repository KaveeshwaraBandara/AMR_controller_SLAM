#include <iostream>
#include <string>
#include <unistd.h>     // For usleep()
#include <fcntl.h>      // For file control (O_RDWR, O_NOCTTY, etc.)
#include <termios.h>    // For serial port settings

class SerialPort {
private:
    int fileDescriptor;

public:
  
    SerialPort(const char* portName, int baudRate) {
        // Open the serial port (non-blocking read/write, no controlling terminal)
        fileDescriptor = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
        
        if (fileDescriptor == -1) {
            std::cerr << "Error: Could not open serial port " << portName << std::endl;
            exit(1);
        }

        // Configure serial port settings
        struct termios tty;
        if (tcgetattr(fileDescriptor, &tty) != 0) {
            std::cerr << "Error getting serial port attributes" << std::endl;
            exit(1);
        }

        // Set baud rate (input & output)
        cfsetispeed(&tty, baudRate);
        cfsetospeed(&tty, baudRate);

        // 8N1 mode (8 data bits, no parity, 1 stop bit)
        tty.c_cflag &= ~PARENB;    // No parity
        tty.c_cflag &= ~CSTOPB;    // 1 stop bit
        tty.c_cflag &= ~CSIZE;     // Clear data size bits
        tty.c_cflag |= CS8;        // 8 data bits
        tty.c_cflag &= ~CRTSCTS;   // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

        // Non-canonical mode (raw data)
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST; // Raw output

        // Apply settings
        if (tcsetattr(fileDescriptor, TCSANOW, &tty) != 0) {
            std::cerr << "Error setting serial port attributes" << std::endl;
            exit(1);
        }
    }

    ~SerialPort() {
        close(fileDescriptor);
    }

 /*   void sendData(const std::string& data) {
        write(fileDescriptor, data.c_str(), data.size());
    }
    */
    void sendCommand(double v, double omega) {
        std::string message;
        message = "set " + std::to_string(v) + " " + std::to_string(omega) + "\n";
        write(fileDescriptor, message.c_str(), message.size());
        std::cout << "Sent: " << message;
        if(v == 0 && omega == 0){
         message = "off\n";
         write(fileDescriptor, message.c_str(), message.size());
      } 
    }
};
