
namespace Utils{

int open_port(const char* port);

bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);

void close_port(int fd);

};
