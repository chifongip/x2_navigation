// Test-only interposer: exercise real bridge failure paths without changing production APIs.
#include <cerrno>
#include <cstddef>
#include <cstdlib>
#include <cstring>

extern "C" int zmq_send(void *, const void *, std::size_t, int)
{
  const char * failure = std::getenv("X2_TEST_ZMQ_SEND_FAILURE");
  errno = failure && std::strcmp(failure, "EAGAIN") == 0 ? EAGAIN : EIO;
  return -1;
}
