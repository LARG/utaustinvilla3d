#include <errno.h>
#include <signal.h>
#include <string>
#include <iostream>
#include <fstream>
#include <rcssnet/tcpsocket.hpp>
//#include <rcssnet/udpsocket.hpp>
#include <rcssnet/exception.hpp>
#include <netinet/in.h>
#include "behaviors/behavior.h"
#include "behaviors/naobehavior.h"
#include "optimization/optimizationbehaviors.h"
#include "behaviors/pkbehaviors.h"
#include "behaviors/gazebobehavior.h"
#include "stats/recordstatsbehavior.h"

using namespace rcss::net;
using namespace std;


TCPSocket gSocket;
//UDPSocket gSocket;
string gHost = "127.0.0.1";
int gPort = 3100;

// Variables for connecting to the monitor port
TCPSocket mSocket;
string mHost = "127.0.0.1";
int mPort = -1;

unsigned long long key = 0;

// bool to indicate whether to continue the agent mainloop
static bool gLoop = true;

// Global variable of agent's body type to be exported
// (should probably make this not global and store this value in a better way)
int agentBodyType = 0;

// SIGINT handler prototype
extern "C" void handler(int sig)
{
    if (sig == SIGINT)
        gLoop = false;
}

void PrintGreeting()
{
    cout << "UT Austin Villa 3D Simulation Team Base Code\n";

}

void PrintHelp()
{
    cout << "\nusage: agentspark [options]" << endl;
    cout << "\noptions:" << endl;
    cout << " --help\tprints this message." << endl;
    cout << " --host=<IP>\tIP of the server." << endl;
    cout << " --port <port>\tport of the server." << endl;
    cout << " --type <type number>\theterogeneous model type number to use." << endl;
    cout << " --rsg <rsg>\trsg file for the nao model." << endl;
    cout << " --team <TeamName>\tName of Team." << endl;
    cout << " --unum <UNum>\tUniform Number of Player." << endl;
    cout << " --paramsfile <filename>\tname of a parameters file to be loaded" << endl;
    cout << " --pkgoalie\tgoalie for penalty kick shootout" << endl;
    cout << " --pkshooter\tshooter for penalty kick shootout" << endl;
    cout << " --gazebo\tagent for Gazebo RoboCup 3D simulation plugin" << endl;
    cout << " --optimize <agent-type>\toptimization agent type" << endl;
    cout << " --recordstats\t record game statistics" << endl;
    cout << " --mhost=<IP>\tIP of the monitor for sending draw commands" << endl;
    cout << " --mport <port>\tport of the monitor for training command parser" << endl;

    cout << "\n";
}


/*
 * Read in parameters from inputsFile, which should be formatted
 * with a set of parameters as key value pairs from strings to
 * floats.  The parameter name should be separated from its value
 * with a tab and parameters should be separated from each other
 * with a single newline.  Parameters will be loaded into the
 * namedParams map.
 */
map<string, string> namedParams;
void LoadParams(const string& inputsFile) {
    istream *input;
    ifstream infile;
    istringstream inString;

    infile.open(inputsFile.c_str(), ifstream::in);

    if(!infile) {
        cerr << "Could not open parameter file " << inputsFile << endl;
        exit(1);
    }

    input = &(infile);

    string name;
    bool fBlockComment = false;
    while(!input->eof())
    {

        // Skip comments and empty lines
        std::string str;
        std::getline(*input, str);
        if (str.length() >= 2 && str.substr(0,2) == "/*") {
            fBlockComment = true;
        } else if (str == "*/") {
            fBlockComment = false;
        }
        if(fBlockComment || str == "" || str[0] == '#' ) {
            continue;
        }

        // otherwise parse strings
        stringstream s(str);
        std::string key;
        std::string value;
        std::getline(s, key, '\t');      //read thru tab
        std::getline(s, value);          //read thru newline
        if(value.empty()) {
            continue;
        }
        namedParams[key] = value;
    }

    infile.close();
}


string teamName;
int uNum;
string outputFile(""); // For optimization
string agentType("naoagent");
string rsg("rsg/agent/nao/nao.rsg");
void ReadOptions(int argc, char* argv[])
{

    teamName = "UTAustinVilla_Base";
    uNum = 0; // Value of 0 means choose next available number

    for( int i = 0; i < argc; i++)
    {
        if ( strcmp( argv[i], "--help" ) == 0 )
        {
            PrintHelp();
            exit(0);
        }
        else if ( strncmp( argv[i], "--host", 6 ) == 0 )
        {
            string tmp=argv[i];

            if ( tmp.length() <= 7 ) // minimal sanity check
            {
                PrintHelp();
                exit(0);
            }
            gHost = tmp.substr(7);
        }
        else if ( strncmp( argv[i], "--mhost", 7 ) == 0 )
        {
            string tmp=argv[i];

            if ( tmp.length() <= 8 ) // minimal sanity check
            {
                PrintHelp();
                exit(0);
            }
            mHost = tmp.substr(8);
        }
        else if ( strncmp( argv[i], "--port", 6) == 0 ) {
            if (i == argc - 1) {
                PrintHelp();
                exit(0);
            }
            gPort = atoi(argv[i+1]);
        }
        else if ( strncmp( argv[i], "--mport", 7) == 0 ) {
            if (i == argc - 1) {
                PrintHelp();
                exit(0);
            }
            mPort = atoi(argv[i+1]);
        }
        else if(strcmp(argv[i], "--team") == 0) {
            if(i == argc - 1) {
                PrintHelp();
                exit(0);
            }

            teamName = argv[i + 1];
        }
        else if(strcmp(argv[i], "--unum") == 0) {
            if(i == argc - 1) {
                PrintHelp();
                exit(0);
            }
            uNum = atoi(argv[i + 1]);
        }
        else if(strcmp(argv[i], "--paramsfile") == 0) {
            if(i == argc - 1) {
                PrintHelp();
                exit(0);
            }
            string inputsFile = argv[i+1];
            LoadParams(inputsFile);
        }
        else if (strcmp(argv[i], "--experimentout") == 0) {
            if(i == argc - 1) {
                PrintHelp();
                exit(0);
            }
            outputFile = argv[i+1];
        }
        else if (strcmp(argv[i], "--optimize") == 0) {
            if(i == argc - 1) {
                PrintHelp();
                exit(0);
            }
            agentType = argv[i+1];
        }
        else if (strcmp(argv[i], "--type") == 0) {
            if(i == argc - 1) {
                PrintHelp();
                exit(0);
            }
            rsg = "rsg/agent/nao/nao_hetero.rsg " + string(argv[i+1]);
            agentBodyType = atoi(argv[i+1]);
        }
        else if (strcmp(argv[i], "--rsg") == 0) {
            if(i == argc - 1) {
                PrintHelp();
                exit(0);
            }
            rsg = argv[i+1];
        }
        else if (strcmp(argv[i], "--pkgoalie") == 0) {
            agentType = "pkgoalie";
        }
        else if (strcmp(argv[i], "--pkshooter") == 0) {
            agentType = "pkshooter";
        }
        else if (strcmp(argv[i], "--gazebo") == 0) {
            agentType = "gazebo";
        }
        else if (strcmp(argv[i], "--recordstats") == 0) {
            agentType = "recordstats";
        }
    } // for-loop
}

bool Init()
{
    cout << "connecting to TCP " << gHost << ":" << gPort << "\n";
    //cout << "connecting to UDP " << gHost << ":" << gPort << "\n";

    try
    {
        Addr local(INADDR_ANY,INADDR_ANY);
        gSocket.bind(local);
    }

    catch (BindErr error)
    {
        cerr << "failed to bind socket with '"
             << error.what() << "'" << endl;

        gSocket.close();
        return false;
    }

    try
    {
        Addr server(gPort,gHost);
        gSocket.connect(server);
    }

    catch (ConnectErr error)
    {
        cerr << "connection failed with: '"
             << error.what() << "'" << endl;
        gSocket.close();
        return false;
    }

    // Connect to the monitor port so that we can use the training command parser
    if (mPort != -1) {
        try
        {
            Addr local(INADDR_ANY,INADDR_ANY);
            mSocket.bind(local);
        }

        catch (BindErr error)
        {
            cerr << "failed to bind socket with '"
                 << error.what() << "'" << endl;

            mSocket.close();
            return false;
        }

        try
        {
            Addr server(mPort,gHost);
            mSocket.connect(server);
        }

        catch (ConnectErr error)
        {
            cerr << "connection failed with: '"
                 << error.what() << "'" << endl;
            mSocket.close();
            return false;
        }
    }


    return true;
}

void Done()
{
    gSocket.close();
    cout << "closed connection to " << gHost << ":" << gPort << "\n";
    if (mPort != -1) {
        mSocket.close();
    }
}

bool SelectInput()
{
    fd_set readfds;
    struct timeval tv = {60,0};
    FD_ZERO(&readfds);
    FD_SET(gSocket.getFD(),&readfds);

    while(1) {
        switch(select(gSocket.getFD()+1,&readfds, 0, 0, &tv)) {
        case 1:
            return 1;
        case 0:
            cerr << "(SelectInput) select failed " << strerror(errno) << endl;
            abort();
            return 0;
        default:
            if(errno == EINTR)
                continue;
            cerr << "(SelectInput) select failed " << strerror(errno) << endl;
            abort();
            return 0;
        }
    }
}

void PutMessage(const string& msg)
{
    if (msg.empty())
    {
        return;
    }

    // prefix the message with it's payload length
    unsigned int len = htonl(msg.size());
    string prefix((const char*)&len,sizeof(unsigned int));
    string str = prefix + msg;
    if ( static_cast<ssize_t>(str.size()) != write(gSocket.getFD(), str.data(), str.size())) {
        LOG_STR("could not put entire message: " + msg);
    }
}

void PutMonMessage(const string& msg)
{
    if (msg.empty())
    {
        return;
    }

    // prefix the message with it's payload length
    unsigned int len = htonl(msg.size());
    string prefix((const char*)&len,sizeof(unsigned int));
    string str = prefix + msg;
    if ( static_cast<ssize_t>(str.size()) != write(mSocket.getFD(), str.data(), str.size())) {
        LOG_STR("could not put entire monitor message: " + msg);
    }
}



bool GetMessage(string& msg)
{
    static char buffer[16 * 1024];

    unsigned int bytesRead = 0;
    while(bytesRead < sizeof(unsigned int))
    {
        SelectInput();
        int readResult = read(gSocket.getFD(), buffer + bytesRead, sizeof(unsigned int) - bytesRead);
        if(readResult < 0)
            continue;
        if (readResult == 0) {
            // [patmac] Kill ourselves if we disconnect from the server
            // for instance when the server is killed.  This helps to
            // prevent runaway agents.
            cerr << "Lost connection to server" << endl;
            Done();
            exit(1);
        }
        bytesRead += readResult;
    }

    //cerr << "buffer = |" << string(buffer+1) << "|\n";
    //cerr << "bytesRead = |" << bytesRead << "|\n";
    //cerr << "Size of buffer = |" << sizeof(buffer) << "|\n";
    //cerr << "buffer = |" << buffer << "|\n";
    //cerr << "buffer[5] = |" << buffer[5] << "|\n";
    //printf ("xxx-%s\n", buffer+5);

    // msg is prefixed with it's total length
    union int_char_t {
        char *c;
        unsigned int *i;
    };
    int_char_t size;
    size.c = buffer;
    unsigned int msgLen = ntohl(*(size.i));
    // cerr << "GM 6 / " << msgLen << " (bytesRead " << bytesRead << ")\n";
    if(sizeof(unsigned int) + msgLen > sizeof(buffer)) {
        cerr << "too long message; aborting" << endl;
        abort();
    }

    // read remaining message segments
    unsigned int msgRead = bytesRead - sizeof(unsigned int);

    //cerr << "msgRead = |" << msgRead << "|\n";

    char *offset = buffer + bytesRead;

    while (msgRead < msgLen)
    {
        if (! SelectInput())
        {
            return false;
        }

        unsigned readLen = sizeof(buffer) - msgRead;
        if(readLen > msgLen - msgRead)
            readLen = msgLen - msgRead;

        int readResult = read(gSocket.getFD(), offset, readLen);
        if(readResult < 0)
            continue;
        msgRead += readResult;
        offset += readResult;
        //cerr << "msgRead = |" << msgRead << "|\n";
    }

    // zero terminate received data
    (*offset) = 0;

    msg = string(buffer+sizeof(unsigned int));

    // DEBUG
    //cout << msg << endl;

    static string lastMsg = "";
    if (msg.compare(lastMsg) == 0) {
        cerr << "Duplicate message received from server -- has the server killed us?\n";
        Done();
        exit(1);
    }
    lastMsg = msg;

    return true;
}

void Run()
{
    Behavior *behavior;
    if (agentType == "naoagent") {
        behavior = new NaoBehavior(teamName, uNum, namedParams, rsg);
    }
    else if (agentType == "pkgoalie") {
        behavior = new PKGoalieBehavior(teamName, uNum, namedParams, rsg);
    }
    else if (agentType == "pkshooter") {
        behavior = new PKShooterBehavior(teamName, uNum, namedParams, rsg);
    }
    else if (agentType == "gazebo") {
        agentBodyType = GAZEBO_AGENT_TYPE;
        behavior = new GazeboBehavior(teamName, uNum, namedParams, rsg);
    }
    else if (agentType == "fixedKickAgent") {
        cerr << "creating OptimizationBehaviorFixedKick" << endl;
        behavior = new OptimizationBehaviorFixedKick(  teamName,
                uNum,
                namedParams,
                rsg,
                outputFile);
    }
    else if (agentType == "walkForwardAgent") {
        cerr << "creating OptimizationBehaviorWalkForward" << endl;
        behavior = new OptimizationBehaviorWalkForward(  teamName,
                uNum,
                namedParams,
                rsg,
                outputFile);
    }
    else if ( agentType == "recordstats") {
        behavior = new RecordStatsBehavior(teamName, uNum, namedParams, rsg,
                                           outputFile);
    }
    else {
        throw "unknown agent type";
    }

    PutMessage(behavior->Init()+"(syn)");

    string msg;
    while (gLoop)
    {
        GetMessage(msg);
        string msgToServer = behavior->Think(msg);
        // To support agent sync mode
        msgToServer.append("(syn)");
        PutMessage(msgToServer);
        if (mPort != -1) {
            PutMonMessage(behavior->getMonMessage());
        }
    }
}

int
main(int argc, char* argv[])
{
    // registering the handler, catching SIGINT signals
    signal(SIGINT, handler);

    // Actually print out the errors that are thrown.
    try
    {
        PrintGreeting();
        ReadOptions(argc,argv);

        if (! Init())
        {
            return 1;
        }

        Run();
        Done();
    }
    catch (char const* c)
    {
        cerr << "-------------ERROR------------" << endl;
        cerr << c << endl;
        cerr << "-----------END ERROR----------" << endl;
    }
    catch (string s)
    {
        cerr << "-------------ERROR------------" << endl;
        cerr << s << endl;
        cerr << "-----------END ERROR----------" << endl;
    }
}
