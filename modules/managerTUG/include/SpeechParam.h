#pragma once

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
