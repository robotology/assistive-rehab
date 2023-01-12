#ifndef MANAGERTUG_SPEECH_H
#define MANAGERTUG_SPEECH_H

#include <vector>
#include <string>
#include <memory>

#include "SpeechParam.h"


class Speech
{
    bool wait{true};
    bool skip{true};
    std::string key{""};
    std::vector<std::shared_ptr<SpeechParam>> params;
public:
    explicit Speech(const std::string &key, const bool &wait=true, const bool &skip=true);

    void dontWait();

    void dontSkip();

    void setKey(const std::string &k);

    std::string getKey() const;

    void setParams(const std::vector<std::shared_ptr<SpeechParam>> &p);

    std::vector<std::shared_ptr<SpeechParam>> getParams();

    bool hasToWait() const;

    bool hasToSkip() const;

    void reset();
};

#endif //MANAGERTUG_SPEECH_H
