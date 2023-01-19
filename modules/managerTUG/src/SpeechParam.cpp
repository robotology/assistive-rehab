#include "SpeechParam.h"


SpeechParam::SpeechParam(const int d) { ss<<d; }


SpeechParam::SpeechParam(const double g) { ss<<g; }


SpeechParam::SpeechParam(const std::string &s) { ss<<s; }


std::string SpeechParam::get() const { return ss.str(); }

