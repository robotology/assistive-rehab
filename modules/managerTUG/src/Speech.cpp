class Speech
{
    bool wait{true};
    bool skip{true};
    string key{""};
    vector<shared_ptr<SpeechParam>> params;
public:
    explicit Speech(const string &key, const bool &wait=true, const bool &skip=true)
        : key(key), wait(wait), skip(skip) { }

    void dontWait() { wait=false; }
    void dontSkip() { skip=false; }

    void setKey(const string &k) { key=k; }
    string getKey() const { return key; }

    void setParams(const vector<shared_ptr<SpeechParam>> &p) { this->params=p; }
    vector<shared_ptr<SpeechParam>> getParams() { return this->params; }

    bool hasToWait() const { return wait; }
    bool hasToSkip() const { return skip; }
    void reset()
    {
        key.clear();
        wait=true;
        skip=true;
        params.clear();
    }

};