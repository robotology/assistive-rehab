class SpeechParam
{
    ostringstream ss;
public:
    explicit SpeechParam(const int d) { ss<<d; }
    explicit SpeechParam(const double g) { ss<<g; }
    explicit SpeechParam(const string &s) { ss<<s; }
    string get() const { return ss.str(); }
};
