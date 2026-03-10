#ifndef LOGGER_H
#define LOGGER_H


class Logger {
public:
    static int numNodesExplored;
    static int numMulticostAllocated;

    static void reset() {
        Logger::numMulticostAllocated = 0;
        Logger::numNodesExplored = 0;
    }
};

#endif