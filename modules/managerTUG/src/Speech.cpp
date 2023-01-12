#include "Speech.h"


Speech::Speech(const std::string &key, const bool &wait, const bool &skip)
    : key(key), wait(wait), skip(skip) { }


void Speech::dontWait() { wait=false; }


void Speech::dontSkip() { skip=false; }


void Speech::setKey(const std::string &k) { key=k; }


std::string Speech::getKey() const { return key; }


void Speech::setParams(const std::vector<std::shared_ptr<SpeechParam>> &p) { this->params=p; }


std::vector<std::shared_ptr<SpeechParam>> Speech::getParams() { return this->params; }


bool Speech::hasToWait() const { return wait; }


bool Speech::hasToSkip() const { return skip; }


void Speech::reset()
{
    key.clear();
    wait=true;
    skip=true;
    params.clear();
}

