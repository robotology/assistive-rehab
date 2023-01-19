#ifndef MANAGERTUG_SPEECHPARAM_H
#define MANAGERTUG_SPEECHPARAM_H


#include <sstream>
#include <string>

class SpeechParam
{
    std::ostringstream ss;
public:
    explicit SpeechParam(const int d);
    explicit SpeechParam(const double g);
    explicit SpeechParam(const std::string &s);
    std::string get() const;
};


#endif //MANAGERTUG_SPEECHPARAM_H