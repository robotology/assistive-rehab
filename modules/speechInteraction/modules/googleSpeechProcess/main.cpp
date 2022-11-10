/*
 * Copyright (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <vector>
#include <iostream>
#include <deque>
#include <cstdio>
#include <cmath>

#include <fstream>
#include <iterator>
#include <string>
#include <unordered_map>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/SoundFile.h>
#include <yarp/dev/PolyDriver.h>

#include <grpc++/grpc++.h>
#include "google/cloud/language/v1/language_service.grpc.pb.h"

#include "googleSpeechProcess_IDL.h"

using namespace google::cloud::language::v1;

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::os::Bottle>
{
    std::string moduleName;
    std::unordered_map<std::string,std::vector<std::string>> key_map;
    yarp::os::RpcServer handlerPort;
    yarp::os::BufferedPort<yarp::os::Bottle> targetPort;
    yarp::os::BufferedPort<yarp::os::Bottle> statusPort;

    yarp::os::Bottle wordList;

public:
    /********************************************************/

    Processing( const std::string &moduleName, const std::unordered_map<std::string, std::vector<std::string>> &key_map )
    {
        this->moduleName = moduleName;
        this->key_map = key_map;
    }

    /********************************************************/
    ~Processing()
    {

    };

    /********************************************************/
    bool open()
    {
        this->useCallback();
        yarp::os::BufferedPort<yarp::os::Bottle >::open( "/" + moduleName + "/text:i" );
        targetPort.open("/"+ moduleName + "/result:o");
        statusPort.open("/"+ moduleName + "/status:o");

        //yarp::os::Network::connect("/googleSpeech/result:o", yarp::os::BufferedPort<yarp::os::Bottle >::getName().c_str());

        return true;
    }

    /********************************************************/
    void close()
    {
        yarp::os::BufferedPort<yarp::os::Bottle >::close();
        targetPort.close();
        statusPort.close();
    }

    /********************************************************/
    void onRead( yarp::os::Bottle &bot )
    {
        yarp::os::Bottle &outTargets = targetPort.prepare();

        wordList.clear();
        outTargets = queryGoogleSyntax(bot);

        targetPort.write();
        yDebug() << "done querying google";

    }

    /********************************************************/
    yarp::os::Bottle queryGoogleSyntax(yarp::os::Bottle& text)
    {
        yDebug() << "in queryGoogleSyntax";

        yDebug() << "Phrase is " << text.toString().c_str();

        std::string tmp = text.toString();

        std::map< std::string, std::string> dictionary;
        dictionary.insert ( std::pair<std::string,std::string>("á","a") );
        dictionary.insert ( std::pair<std::string,std::string>("à","a") );
        dictionary.insert ( std::pair<std::string,std::string>("é","e") );
        dictionary.insert ( std::pair<std::string,std::string>("è","e") );
        dictionary.insert ( std::pair<std::string,std::string>("í","i") );
        dictionary.insert ( std::pair<std::string,std::string>("ó","o") );
        dictionary.insert ( std::pair<std::string,std::string>("ú","u") );
        dictionary.insert ( std::pair<std::string,std::string>("ñ","n") );

        std::string tmp2 = tmp;
        std::string strAux;
        for (auto it= dictionary.begin(); it != dictionary.end(); it++)
        {
            tmp2=(it->first);
            std::size_t found=tmp.find_first_of(tmp2);

            while (found!=std::string::npos)
            {
                yError() << "in found" << found;
                strAux=(it->second);
                tmp.erase(found,tmp2.length());
                tmp.insert(found,strAux);
                found=tmp.find_first_of(tmp2,found+1);
            }
        }

        yDebug() << "Phrase is now " << tmp.c_str();
        tmp.erase(std::remove(tmp.begin(),tmp.end(),'\"'),tmp.end());

        yDebug() << tmp.size();
        yDebug() << std::isalnum(tmp[1]);

        if (tmp.size() > 1 && std::isalnum(tmp[0])==0)
            tmp = tmp.substr(1, tmp.size() - 2);

        yDebug() << "Phrase is now " << tmp.c_str();

        std::string content = tmp;

        AnalyzeSyntaxRequest request;
        AnalyzeSyntaxResponse response;
        grpc::Status status;
        grpc::ClientContext context;

        setArguments( request.mutable_document(), content );

        // EncodingType //
        request.set_encoding_type( EncodingType::UTF8 );

        yarp::os::Bottle b;
        b.clear();
        auto creds = grpc::GoogleDefaultCredentials();
        auto channel = grpc::CreateChannel("language.googleapis.com", creds);
        std::unique_ptr< LanguageService::Stub> stub( LanguageService::NewStub( channel ) );

        status = stub->AnalyzeSyntax( &context, request, &response );

        yarp::os::Bottle &outStatus = statusPort.prepare();
        outStatus.clear();
        if ( status.ok() )
        {
            yInfo() << "Status returned OK";
            yInfo() << "\n------Response------\n";

            read_language( &response.language() );
            read_sentences( &response.sentences() );
            //read_tokens( &response.tokens() );
            b = read_answer( &response.tokens() );
            if ( b.toString() == "" )
            {
                outStatus.addString("topic not recognized");
            }
            else
            {
                outStatus.addString("everything ok");
            }

        } else if ( !status.ok() )
        {
            yError() << "Status Returned Canceled";
            outStatus.addString(status.error_message());
        }
        statusPort.write();

        return b;
    }

    /********************************************************/
    void setArguments(Document* doc, std::string& content)
    {
        doc->set_type( Document_Type::Document_Type_PLAIN_TEXT );
        doc->set_content( content );
    }

    /********************************************************/
    void read_language( const std::string* lang )
    {

        std::cout << "\n----Language----" << std::endl;
        std::cout << "Language: " << *lang << std::endl;

    }

    /********************************************************/
    void read_sentences( const google::protobuf::RepeatedPtrField< Sentence >* sentences ) {

        // SENTENCES //
        yInfo() << "----Sentences----";
        yInfo() << "Sentences Size: " << sentences->size();
        for( int i = 0; i < sentences->size(); i++ ) {
            yInfo() << "Sentence " << i << " has text: " << sentences->Get( i ).has_text();
            if ( sentences->Get( i ).has_text() ) {
                yInfo() << "Sentence text: " << sentences->Get( i ).text().content();
            }

            yInfo() << "Sentence " << i << " has sentiment: " << sentences->Get( i ).has_sentiment();
            if ( sentences->Get( i ).has_sentiment() ) {
                yInfo() << "\tSentence " << i << " sentiment: "
                    << "\n\t\tMagnitude: "
                    << sentences->Get( i ).sentiment().magnitude() // float
                    << "\n\t\tScore: "
                    << sentences->Get( i ).sentiment().score() // float
                    << " ";
            }
        }
    }

    /********************************************************/
    yarp::os::Bottle read_answer( const google::protobuf::RepeatedPtrField< Token >* tokens ) {

        yarp::os::Bottle &outTargets = targetPort.prepare();

        for ( int i = 0; i < tokens->size(); i++ ) {

            yarp::os::Bottle &words = wordList.addList();
            //yInfo() << tokens->Get( i ).text().content();
            yarp::os::Bottle &content = words.addList();
            content.addString("content");
            content.addString(tokens->Get( i ).text().content());

            //yInfo() << "root" << tokens->Get( i ).dependency_edge().head_token_index();
            yarp::os::Bottle &root = words.addList();
            root.addString("root");
            root.addInt32(tokens->Get( i ).dependency_edge().head_token_index());

            //yInfo() << PartOfSpeech_Tag_Name(tokens->Get( i ).part_of_speech().tag());
            yarp::os::Bottle &tag = words.addList();
            tag.addString("tag");
            tag.addString(PartOfSpeech_Tag_Name(tokens->Get( i ).part_of_speech().tag()));

            //yInfo() << DependencyEdge_Label_Name( tokens->Get( i ).dependency_edge().label() );
            yarp::os::Bottle &label = words.addList();
            label.addString("label");
            label.addString(DependencyEdge_Label_Name( tokens->Get( i ).dependency_edge().label()));

            //yInfo() << tokens->Get( i ).lemma();
            yarp::os::Bottle &lemma = words.addList();
            lemma.addString("lemma");
            lemma.addString(tokens->Get( i ).lemma());
        }

        yInfo() << "wordList " << wordList.toString().c_str();

        yarp::os::Bottle root;
        yarp::os::Bottle verb;
        yarp::os::Bottle noun;
        bool foundVerb = false;
        bool foundPOBJ = false;
        bool foundDOBJ = false;
        bool foundADV = false;
        bool foundACOMP = false;
        bool foundNSUBJ = false;
        bool foundTMOD = false;
        bool foundTMARK = false;
        bool foundADVMOD = false;
        bool foundNPADVMOD = false;
        std::vector<int> verbinc;
        std::vector<int> nouninc;

        outTargets.clear();
        noun.clear();
        nouninc.clear();
        verb.clear();
        verbinc.clear();

        for ( int i = 0; i <wordList.size(); i++)
        {
            root.clear();
            root.addString(wordList.get(i).asList()->find("tag").asString());

            if (strcmp(root.toString().c_str(), "VERB") == 0)
            {
                if (strcmp(wordList.get(i).asList()->find("label").asString().c_str(),"ROOT") == 0)
                {
                    foundVerb = true;
                    verb.addString(wordList.get(i).asList()->find("lemma").asString());
                    verbinc.push_back(i);
                }
            }
            if (strcmp(root.toString().c_str(), "NOUN") == 0)
            {
                if (strcmp(wordList.get(i).asList()->find("label").asString().c_str(),"POBJ") == 0)
                {
                    foundPOBJ = true;
                    noun.addString(wordList.get(i).asList()->find("lemma").asString());
                    nouninc.push_back(i);
                }
                if (strcmp(wordList.get(i).asList()->find("label").asString().c_str(),"DOBJ") == 0)
                {
                    foundDOBJ = true;
                    noun.addString(wordList.get(i).asList()->find("lemma").asString());
                    nouninc.push_back(i);
                }
                if (strcmp(wordList.get(i).asList()->find("label").asString().c_str(),"NSUBJ") == 0)
                {
                    foundNSUBJ = true;
                    noun.addString(wordList.get(i).asList()->find("lemma").asString());
                    nouninc.push_back(i);
                }
                if (strcmp(wordList.get(i).asList()->find("label").asString().c_str(),"TMOD") == 0)
                {
                    foundTMOD = true;
                    noun.addString(wordList.get(i).asList()->find("lemma").asString());
                    nouninc.push_back(i);
                }
                if (strcmp(wordList.get(i).asList()->find("label").asString().c_str(),"NPADVMOD") == 0)
                {
                    foundNPADVMOD = true;
                    noun.addString(wordList.get(i).asList()->find("lemma").asString());
                    nouninc.push_back(i);
                }
            }
            if (strcmp(root.toString().c_str(), "ADV") == 0)
            {
                if (strcmp(wordList.get(i).asList()->find("label").asString().c_str(),"ROOT") == 0)
                {
                    foundADV = true;
                    noun.addString(wordList.get(i).asList()->find("lemma").asString());
                    nouninc.push_back(i);
                }
                if (strcmp(wordList.get(i).asList()->find("label").asString().c_str(),"ACOMP") == 0)
                {
                    foundACOMP = true;
                    noun.addString(wordList.get(i).asList()->find("lemma").asString());
                    nouninc.push_back(i);
                }
                if (strcmp(wordList.get(i).asList()->find("label").asString().c_str(),"ADVMOD") == 0)
                {
                    foundADVMOD = true;
                    noun.addString(wordList.get(i).asList()->find("lemma").asString());
                    nouninc.push_back(i);
                }
            }
            if (strcmp(root.toString().c_str(), "ADJ") == 0)
            {
                if (strcmp(wordList.get(i).asList()->find("label").asString().c_str(),"ACOMP") == 0)
                {
                    foundACOMP = true;
                    noun.addString(wordList.get(i).asList()->find("lemma").asString());
                    nouninc.push_back(i);
                }
            }
            if (strcmp(root.toString().c_str(), "ADP") == 0)
            {
                if (strcmp(wordList.get(i).asList()->find("label").asString().c_str(),"MARK") == 0)
                {
                    foundTMARK = true;
                    noun.addString(wordList.get(i).asList()->find("lemma").asString());
                    nouninc.push_back(i);
                }
            }
        }

        if (foundVerb)
        {
            for (int i=0; i<verbinc.size(); i++)
            {
                int vinc = verbinc[i];
                yInfo() << "have root verb" << verb.get(i).asString().c_str() << wordList.get(vinc).asList()->toString().c_str();
            }
        }

        if (foundPOBJ || foundDOBJ || foundADV || foundACOMP || foundNSUBJ || foundTMOD || foundTMARK || foundADVMOD)
        {
            for (int i=0; i<nouninc.size(); i++)
            {
                int ninc = nouninc[i];
                int nounInt = wordList.get(ninc).asList()->find("root").asInt32();
                std::string nounLabel = wordList.get(nounInt).asList()->find("label").asString();
                yInfo() << "have noun " << noun.get(i).asString().c_str() << wordList.get(ninc).asList()->toString().c_str();
                yInfo() << "have nounLabel " << nounLabel.c_str() << wordList.get(nounInt).asList()->toString().c_str();
            }

            for (auto it=key_map.begin(); it!=key_map.end(); it++)
            {
                std::string key = it->first;
                std::vector<std::string> values = it->second;
                for (int i=0; i<values.size(); i++)
                {
                    std::string vi = values[i];
                    for (int j=0; j<noun.size(); j++)
                    {
                        std::string nounToCompare = noun.get(j).asString();
                        if (strcmp(nounToCompare.c_str(),vi.c_str()) == 0)
                        {
                            yInfo()<< "We need to send bottle with" << key;
                            outTargets.addString(key);
                        }
                    }
                }
            }

            /*if (strcmp(nounLabel.c_str(), "XCOMP") == 0)
            {
                yInfo()<< "We need to send bottle with AID";
            }

            if (strcmp(nounLabel.c_str(), "PREP") == 0)
            {
                int prepInt = wordList.get(nounInt).asList()->find("root").asInt32();
                std::string prepLabel = wordList.get(prepInt).asList()->find("label").asString();
                yInfo() << "have prepLabel " << prepLabel.c_str() << wordList.get(prepInt).asList()->toString().c_str();;;

                if (strcmp(prepLabel.c_str(), "XCOMP") == 0)
                {
                    yInfo()<< "We need to send bottle with AID";
                }

                if (strcmp(prepLabel.c_str(), "ROOT") == 0)
                {
                    yInfo()<< "We need to send bottle with SPEED";
                }
            }

            if (strcmp(nounLabel.c_str(), "ROOT") == 0)
            {
                yInfo()<< "We need to send bottle with SPEED";
            }*/

            //targetPort.write();
        }
        return outTargets;
    }

    /********************************************************/

    void read_tokens( const google::protobuf::RepeatedPtrField< Token >* tokens ) {

        for ( int i = 0; i < tokens->size(); i++ ) {

            std::cout << "\n-------- " << i << std::endl;
            yInfo() << tokens->Get( i ).text().content();
            yInfo() << "root" << tokens->Get( i ).dependency_edge().head_token_index();
            yInfo() << PartOfSpeech_Tag_Name(tokens->Get( i ).part_of_speech().tag());
            yInfo() << DependencyEdge_Label_Name( tokens->Get( i ).dependency_edge().label() );
            yInfo() << "lemma" << tokens->Get( i ).lemma();
        }

        for ( int i = 0; i < tokens->size(); i++ ) {

            std::cout
                << "-- Token " << i << " --"
                << std::endl;

            std::cout
                << "Token " << i << " has text: "
                << tokens->Get( i ).has_text()
                << std::endl;

            if ( tokens->Get( i ).has_text() ) {
                std::cout
                    << "\tToken " << i << " text: "
                    << tokens->Get( i ).text().content() // string
                    << "\n\tScore: "
                    << tokens->Get( i ).text().begin_offset() // int32
                    << std::endl;
            }

            std::cout
                << "Token " << i << " has PartOfSpeech: "
                << tokens->Get( i ).has_part_of_speech()
                << std::endl;

            if ( tokens->Get( i ).has_part_of_speech() ) {
                std::cout
                << "\n\t\tTag: "
                << PartOfSpeech_Tag_Name( tokens->Get( i ).part_of_speech().tag() )
                    //<< "\n\tToken " << i << " PartOfSpeech: "
                    //<< "\n\t\tAspect: "
                    //<< PartOfSpeech_Aspect_Name( tokens->Get( i ).part_of_speech().aspect() )
                    //<< "\n\t\tCase: "
                    //<< PartOfSpeech_Case_Name( tokens->Get( i ).part_of_speech().instance() )
                    //<< "\n\t\tForm: "
                    //<< PartOfSpeech_Form_Name( tokens->Get( i ).part_of_speech().form() )
                    //<< "\n\t\tGender: "
                    //<< PartOfSpeech_Gender_Name( tokens->Get( i ).part_of_speech().gender() )
                    //<< "\n\t\tMood: "
                    //<< PartOfSpeech_Mood_Name( tokens->Get( i ).part_of_speech().mood() )
                    //<< "\n\t\tNumber: "
                    //<< PartOfSpeech_Number_Name( tokens->Get( i ).part_of_speech().number() )
                    //<< "\n\t\tPerson: "
                    //<< PartOfSpeech_Person_Name( tokens->Get( i ).part_of_speech().person() )
                    //<< "\n\t\tProper: "
                    //<< PartOfSpeech_Proper_Name( tokens->Get( i ).part_of_speech().proper() )
                    //<< "\n\t\tReciprocity: "
                    //<< PartOfSpeech_Reciprocity_Name( tokens->Get( i ).part_of_speech().reciprocity() )
                    //<< "\n\t\tTag: "
                    //<< PartOfSpeech_Tag_Name( tokens->Get( i ).part_of_speech().tag() )
                    //<< "\n\t\tTense: "
                    //<< PartOfSpeech_Tense_Name( tokens->Get( i ).part_of_speech().tense() )
                    //<< "\n\t\tVoice: "
                    //<< PartOfSpeech_Voice_Name( tokens->Get( i ).part_of_speech().voice() )

                    << std::endl;
            }

            std::cout
                << "Token " << i << " has DependencyEdge: "
                << tokens->Get( i ).has_dependency_edge()
                << std::endl;

            if ( tokens->Get( i ).has_dependency_edge() ) {
                std::cout
                    << "\tToken " << i << " DependencyEdge: "
                    << "\n\t\tHead Token Index: "
                    << tokens->Get( i ).dependency_edge().head_token_index() // int32
                    << "\n\t\tLabel: "
                    << DependencyEdge_Label_Name( tokens->Get( i ).dependency_edge().label() )
                    << std::endl;
            }
            std::cout
                << "Token " << i << " lemma: "
                << tokens->Get( i ).lemma() // string
                << std::endl
                << std::endl;
        }
    }

    /********************************************************/
    bool start_acquisition()
    {
        return true;
    }

    /********************************************************/
    bool stop_acquisition()
    {
        return true;
    }
};

/********************************************************/
class Module : public yarp::os::RFModule, public googleSpeechProcess_IDL
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    Processing                  *processing;
    friend class                processing;

    std::unordered_map<std::string,std::vector<std::string>> key_map;

    bool                        closing;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

public:

    /********************************************************/
    Module() : closing(false) { }

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;
        yarp::os::Bottle &general = rf.findGroup("general");
        if (general.isNull())
        {
            yError()<<"Unable to find \"general\"";
            return false;
        }
        std::string moduleName = general.check("name", yarp::os::Value("yarp-google-speech-process"), "module name (string)").asString();
        int numKey = general.find("num-keywords").asInt32();
        for (int i=0; i<numKey; i++)
        {
            std::string keywi = "keyword-"+std::to_string(i);
            yarp::os::Bottle &bKeyword = rf.findGroup(keywi);
            if (bKeyword.isNull())
            {
                yError()<<"Unable to find"<<keywi;
                return false;
            }
            if (!bKeyword.check("key") || !bKeyword.check("value"))
            {
                yError()<<"Unable to find \"key\" and/or \"value\"";
                return false;
            }
            std::string key = bKeyword.find("key").asString();
            yarp::os::Bottle *bValue = bKeyword.find("value").asList();
            std::vector<std::string> value(bValue->size());
            for (int j=0; j<bValue->size(); j++)
            {
                value[j] = bValue->get(j).asString();
            }

            key_map[key] = value;
        }

        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        processing = new Processing( moduleName, key_map );

        /* now start the thread to do the work */
        processing->open();

        attach(rpcPort);

        return true;
    }

    /**********************************************************/
    bool close()
    {
        processing->close();
        delete processing;
        return true;
    }

    /**********************************************************/
    bool start()
    {
        processing->start_acquisition();
        return true;
    }

    /**********************************************************/
    bool stop()
    {
        processing->stop_acquisition();
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /********************************************************/
    bool quit()
    {
        closing=true;
        return true;
    }

    /********************************************************/
    bool updateModule()
    {
        return !closing;
    }
};

/********************************************************/
/**************** FAKE MODULES **************************/
/********************************************************/
/********************************************************/

/********************************************************/
class FakeProcessing : public yarp::os::BufferedPort<yarp::os::Bottle>
{
    std::string moduleName;
    std::unordered_map<std::string,std::vector<std::string>> key_map;
    yarp::os::RpcServer handlerPort;
    yarp::os::BufferedPort<yarp::os::Bottle> targetPort;
    yarp::os::BufferedPort<yarp::os::Bottle> statusPort;

    yarp::os::Bottle wordList;

public:
    /********************************************************/

    FakeProcessing( const std::string &moduleName, const std::unordered_map<std::string, std::vector<std::string>> &key_map )
    {
        this->moduleName = moduleName;
        this->key_map = key_map;
    }

    /********************************************************/
    ~FakeProcessing()
    {

    };

    /********************************************************/
    bool open()
    {
        this->useCallback();
        yarp::os::BufferedPort<yarp::os::Bottle >::open( "/" + moduleName + "/text:i" );
        targetPort.open("/"+ moduleName + "/result:o");
        statusPort.open("/"+ moduleName + "/status:o");

        //yarp::os::Network::connect("/googleSpeech/result:o", yarp::os::BufferedPort<yarp::os::Bottle >::getName().c_str());

        return true;
    }

    /********************************************************/
    void close()
    {
        yarp::os::BufferedPort<yarp::os::Bottle >::close();
        targetPort.close();
        statusPort.close();
    }

    /********************************************************/
    void onRead( yarp::os::Bottle &bot )
    {
        yarp::os::Bottle &outTargets = targetPort.prepare();

        wordList.clear();
        outTargets = fakeQueryGoogleSyntax(bot);

        targetPort.write();
        yDebug() << "Done FAKE querying google";

    }

    /********************************************************/
    yarp::os::Bottle fakeQueryGoogleSyntax(yarp::os::Bottle& text)
    {
        yDebug() << "in fakeQueryGoogleSyntax";

        yDebug() << "Phrase is " << text.toString().c_str();

        yarp::os::Bottle &outStatus = statusPort.prepare();
        outStatus.clear();
        yDebug() << "Status returned OK";
        yDebug() << "\n------Response------\n";

        outStatus.addString("everything ok");
        statusPort.write();

        yarp::os::Bottle b;
        b.clear();

        return b;
    }

    /********************************************************/
    bool start_acquisition()
    {
        return true;
    }

    /********************************************************/
    bool stop_acquisition()
    {
        return true;
    }
};
class FakeModule : public yarp::os::RFModule, public googleSpeechProcess_IDL
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    FakeProcessing             *processing;
    friend class                processing;

    std::unordered_map<std::string,std::vector<std::string>> key_map;

    bool                        closing;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

public:

    /********************************************************/
    FakeModule() : closing(false) { }

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;
        yarp::os::Bottle &general = rf.findGroup("general");
        if (general.isNull())
        {
            yError()<<"Unable to find \"general\"";
            return false;
        }
        std::string moduleName = general.check("name", yarp::os::Value("yarp-google-speech-process"), "module name (string)").asString();
        int numKey = general.find("num-keywords").asInt32();
        for (int i=0; i<numKey; i++)
        {
            std::string keywi = "keyword-"+std::to_string(i);
            yarp::os::Bottle &bKeyword = rf.findGroup(keywi);
            if (bKeyword.isNull())
            {
                yError()<<"Unable to find"<<keywi;
                return false;
            }
            if (!bKeyword.check("key") || !bKeyword.check("value"))
            {
                yError()<<"Unable to find \"key\" and/or \"value\"";
                return false;
            }
            std::string key = bKeyword.find("key").asString();
            yarp::os::Bottle *bValue = bKeyword.find("value").asList();
            std::vector<std::string> value(bValue->size());
            for (int j=0; j<bValue->size(); j++)
            {
                value[j] = bValue->get(j).asString();
            }

            key_map[key] = value;
        }

        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        processing = new FakeProcessing( moduleName, key_map );

        /* now start the thread to do the work */
        processing->open();

        attach(rpcPort);

        return true;
    }

    /**********************************************************/
    bool close()
    {
        processing->close();
        delete processing;
        return true;
    }

    /**********************************************************/
    bool start()
    {
        processing->start_acquisition();
        return true;
    }

    /**********************************************************/
    bool stop()
    {
        processing->stop_acquisition();
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /********************************************************/
    bool quit()
    {
        closing=true;
        return true;
    }

    /********************************************************/
    bool updateModule()
    {
        return !closing;
    }
};

/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    yarp::os::ResourceFinder rf;

    rf.setDefaultContext( "googleSpeechProcess" );
    rf.setDefaultConfigFile( "config.ini" );
    rf.setDefault("name","googleSpeechProcess");
    rf.configure(argc,argv);

    if(rf.check("offline"))
    {
        FakeModule fakemodule;
        return fakemodule.runModule(rf);
    }
    else
    {
        Module module;
        return module.runModule(rf);
    }

    return EXIT_SUCCESS;
}
