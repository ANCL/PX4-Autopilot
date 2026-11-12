// Thread-safe, explicit trajectory-start handshake between shell and SE3.
// Only a small builtin plan index crosses threads, not a large Plan copy.

#include "Plan.hpp"
#include "Catalog.hpp"

#include <pthread.h>

namespace trajectory_generator
{
namespace
{

pthread_mutex_t g_request_mutex = PTHREAD_MUTEX_INITIALIZER;
bool g_se3_holding{false};
int g_requested_plan{-1};

} // namespace

void set_se3_start_allowed(bool allowed)
{
  pthread_mutex_lock(&g_request_mutex);
  g_se3_holding = allowed;

  if (!allowed) {
    g_requested_plan = -1;
  }

  pthread_mutex_unlock(&g_request_mutex);
}

bool request_se3_start(const char *name)
{
  const int index = builtin_plan_index(name);

  if (index < 0) {
    return false;
  }

  pthread_mutex_lock(&g_request_mutex);
  const bool accepted = g_se3_holding && g_requested_plan < 0;

  if (accepted) {
    g_requested_plan = index;
  }

  pthread_mutex_unlock(&g_request_mutex);
  return accepted;
}

bool consume_start_request(Plan &plan)
{
  pthread_mutex_lock(&g_request_mutex);
  const int index = g_se3_holding ? g_requested_plan : -1;

  if (index >= 0) {
    g_requested_plan = -1;
    g_se3_holding = false;
  }

  pthread_mutex_unlock(&g_request_mutex);

  return index >= 0 && build_builtin_plan(index, plan);
}

} // namespace trajectory_generator
