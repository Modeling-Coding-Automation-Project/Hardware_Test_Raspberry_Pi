#include "app.hpp"

int main(void) {
  // set real-time priority
  struct sched_param param;
  param.sched_priority = 99;

  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    perror("sched_setscheduler");
    return 1;
  }

  // run MPC test
  python_mpc_tester.test_mpc();

  return 0;
}
