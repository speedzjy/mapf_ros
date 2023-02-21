#pragma once

#ifndef UTILITY_H
#define UTILITY_H

#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <mutex>

#include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/heap/fibonacci_heap.hpp>

// print color macro
#define NONE         "\033[0m"
#define RED          "\033[1;31m"
#define GREEN        "\033[1;32m"
#define BLUE         "\033[1;34m"
#define DARY_GRAY    "\033[1;30m"
#define CYAN         "\033[1;36m"
#define LIGHT_CYAN   "\033[0;36m"
#define PURPLE       "\033[1;35m"
#define LIGHT_PURPLE "\033[0;35m"
#define BROWN        "\033[1;33m"
#define YELLOW       "\033[1;33m"
#define LIGHT_GRAY   "\033[0;37m"
#define WHITE        "\033[1;37m"

#endif