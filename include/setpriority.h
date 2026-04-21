#pragma once

#include <thread>
#include <iostream>
#include <pthread.h>
#include <sched.h>

void set_thread_priority(std::thread &t, int priority);